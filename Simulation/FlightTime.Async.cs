using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using SeaPower;
using UnityEngine;

namespace AutoTOT
{
    /// <summary>
    /// Runs the integration loop on worker threads so a burst of refreshes does not land on the frame that
    /// asks for them: setup on the main thread, workers run the pure loop, only the main thread writes the
    /// cache. See docs/ARCHITECTURE.md "Async solver design".
    /// </summary>
    internal static partial class FlightTime
    {
        private sealed class SolveRequest
        {
            internal TofKey Key;
            internal SolveInput Input;
            internal AmmunitionParameters Ap;
            // Held so a declined result can fall back on the main thread, where the fallback tiers
            // (which need ObjectBase) are legal to call.
            internal ObjectBase Unit;
            internal string AmmoId;
            internal ObjectBase Target;
            internal float Result;
            // When the input snapshot was built on the main thread: the supersede check compares CapturedSim,
            // and publishAge is measured from CapturedReal. CapturedUnscaled is the same instant on the cache's
            // clock, so a published entry reports the age of its geometry rather than of its publication.
            internal float CapturedSim;
            internal float CapturedReal;
            internal float CapturedUnscaled;
            // Stopwatch timestamp at which a worker took the request, 0 while it is still queued. Written by the
            // worker and read by the main thread, through Interlocked on both sides.
            internal long StartedTicks;
        }

        private static readonly ConcurrentQueue<SolveRequest> _pending = new ConcurrentQueue<SolveRequest>();
        private static readonly ConcurrentQueue<SolveRequest> _done = new ConcurrentQueue<SolveRequest>();
        // Main-thread only: the request each in-flight key waits on. Stops a burst queueing the same key every frame,
        // and lets DrainCompleted publish only that request: after an eviction and re-queue, or a mission reset, an
        // older request for the same key can finish first, and its geometry is the staler one.
        private static readonly Dictionary<TofKey, SolveRequest> _inFlight = new Dictionary<TofKey, SolveRequest>();
        private static readonly SemaphoreSlim _signal = new SemaphoreSlim(0);
        private static Thread[] _workers;

        /// <summary>
        /// Verification mode: solve every queued request a second time on the main thread and compare. Solve
        /// is deterministic over its input, so any difference means the snapshot did not carry some value.
        /// Yields one check per queued simulation rather than the few usable accuracy samples a mission
        /// gives. Doubles the simulation work, so it is a correctness run, never a timing run.
        /// </summary>
        internal static bool VerifySolve;
        internal static long VerifyChecked;
        internal static long VerifyMismatched;

        internal static int QueueDepth => _pending.Count;
        internal static int InFlight => _inFlight.Count;
        internal static long AsyncCompleted;
        internal static long AsyncDeclined;

        // Step-loop faults on a worker (the request still completes, as a decline) and anything else a worker
        // caught that is not a shutdown abort. Reset per mission.
        internal static long WorkerFaults;
        internal static long WorkerAborts;

        /// <summary>
        /// Real seconds a worker may spend on one request before <see cref="EvictIfStranded"/> takes its key back:
        /// orders of magnitude above a worst-case solve, so it fires only on a worker that is gone, never on one
        /// that is merely slow.
        /// </summary>
        private const float StrandedKeyTimeoutReal = 10f;

        /// <summary>
        /// Real seconds a request may wait in the queue before a worker takes it. Waiting is not a fault: a large
        /// strike on one slow worker queues for seconds, and evicting those keys would only re-queue them and
        /// move their solves onto the main thread. Long enough that only a pool with no live worker reaches it.
        /// </summary>
        private const float QueuedKeyTimeoutReal = 60f;

        /// <summary>Keys taken back from a worker that never returned. Reported by the census.</summary>
        internal static int AsyncStranded;

        // Sim time of the last synchronous cache write for a key that was in flight. A worker result captured
        // at an earlier sim time describes older geometry and must not overwrite the newer answer. Sim time,
        // not real time: within one frame or while paused the game has not moved, so the worker's answer
        // equals the synchronous one and may publish. Recorded only while the key is in flight. Main thread only.
        private static readonly Dictionary<TofKey, float> _syncWrittenAt = new Dictionary<TofKey, float>();

        /// <summary>Worker results dropped because a synchronous solve had already answered the key.</summary>
        internal static long AsyncSuperseded;

        /// <summary>Called by the synchronous path after it writes the cache. Main thread only.</summary>
        private static void NoteSyncWrite(TofKey key)
        {
            if (_inFlight.ContainsKey(key)) _syncWrittenAt[key] = GameClock.SimNow();
        }
        // Written by DrainCompleted, read by the census line. Real seconds, main thread only.
        internal static float LastPublishAgeReal;
        internal static float MaxPublishAgeReal;

        /// <summary>Worker count. 0 disables the pipeline and everything runs synchronously.</summary>
        internal static int WorkerCount { get; private set; }

        internal static void StartWorkers(int count)
        {
            if (_workers != null) return;
            if (count <= 0)
            {
                // Say so: silence would be indistinguishable from the pool having started.
                Bootstrap.Log.LogInfo(
                    "[AutoTOT] estimator: 0 solve workers, running SYNCHRONOUSLY on the main thread " +
                    "(EstimatorThreads = 0)");
                return;
            }
            WorkerCount = count;
            _workers = new Thread[count];
            for (int i = 0; i < count; i++)
            {
                var th = new Thread(WorkerLoop)
                {
                    // Background, so a stuck worker cannot keep the process alive; below normal, so the
                    // render and job threads win a contended core and the estimate arrives a frame later
                    // instead of stealing a frame.
                    IsBackground = true,
                    Priority = System.Threading.ThreadPriority.BelowNormal,
                    Name = "AutoTOT.Solve" + i,
                };
                _workers[i] = th;
                th.Start();
            }
            Bootstrap.Log.LogInfo(
                $"[AutoTOT] estimator: ASYNC, {count} solve worker(s) started " +
                $"({SystemInfo.processorCount} logical cores)");
        }

        // Runs until the process exits (the threads are background). Nothing stops the pool: stopping it
        // mid-mission would strand queued work (see EstimatorThreads in Bootstrap.LoadConfig).
        private static void WorkerLoop()
        {
            while (true)
            {
                try
                {
                    _signal.Wait();
                    while (_pending.TryDequeue(out SolveRequest r))
                    {
                        Interlocked.Exchange(ref r.StartedTicks, System.Diagnostics.Stopwatch.GetTimestamp());
                        IntegratedPhases ph = default;
                        // Setup was timed on the main thread; this stamps the loop start on this thread, so
                        // LoopDone measures the worker's own work.
                        ModelStats.LoopStarting();
                        try
                        {
                            r.Result = Solve(in r.Input, r.Ap, ref ph);
                        }
                        catch (Exception e) when (!(e is ThreadAbortException))
                        {
                            // Completed as a decline rather than dropped: a dropped request strands its key in
                            // _inFlight, so RequestRefresh reports "already queued" and the caller reuses an old
                            // estimate until EvictIfStranded. As a decline, DrainCompleted runs the lower tiers.
                            r.Result = -1f;
                            ReportWorkerFault(e);
                        }
                        _done.Enqueue(r);
                    }
                }
                catch (ThreadAbortException)
                {
                    // Process shutdown: counted, not logged, since a key poisoned at shutdown costs nothing.
                    // Kept apart from WorkerFaults so a real mid-mission fault cannot hide among them.
                    Interlocked.Increment(ref WorkerAborts);
                }
                catch (Exception e)
                {
                    // A worker must never die: the queue would fill and every refresh would stall.
                    // The one log call in the mod that runs off the main thread; see ModLog's note.
                    Interlocked.Increment(ref WorkerFaults);
                    ModLog.Warn("solve worker", e);
                }
            }
        }

        /// <summary>
        /// Count a step-loop fault on a worker; the first since the last reset is logged with its stack, the rest
        /// are counted on the estimator-census line. Worker thread.
        /// </summary>
        private static void ReportWorkerFault(Exception e)
        {
            if (Interlocked.Increment(ref WorkerFaults) != 1) return;
            Bootstrap.Log.LogWarning(
                "[AutoTOT] solve worker: the step loop threw, so that shot is answered by a less accurate tier. " +
                $"Later faults are counted on the estimator-census line:\n{e}");
        }

        /// <summary>
        /// Queue a refresh for a value that already has a usable previous estimate. Returns false if
        /// the work could not be queued, in which case the caller keeps using its cached value.
        /// Main thread only.
        /// </summary>
        internal static bool RequestRefresh(ObjectBase unit, string ammoId, ObjectBase target)
        {
            if (_workers == null) return false;
            if (!TryKey(unit, ammoId, target, out AmmunitionParameters ap, out TofKey key)) return false;
            // A stranded key is taken back and reported as a failed refresh, so the caller runs its
            // synchronous fallback this tick instead of waiting for the re-queued solve.
            if (_inFlight.ContainsKey(key))
            {
                if (!EvictIfStranded(key)) return true;   // already queued; not a failure
                return false;                            // evicted: solve it here, now
            }

            if (!TryBuildSolveInput(unit, ap, target, out SolveInput input, out _, emitDiag: false))
                return false;                            // integrator declines; caller goes synchronous
            ModelStats.SetupDone();                      // setup ran here, on the main thread

            var request = new SolveRequest
            {
                Key = key, Input = input, Ap = ap,
                Unit = unit, AmmoId = ammoId, Target = target,
                CapturedSim = GameClock.SimNow(),
                CapturedReal = Time.realtimeSinceStartup,
                CapturedUnscaled = Time.unscaledTime,
            };
            _inFlight[key] = request;
            _pending.Enqueue(request);
            _signal.Release();
            return true;
        }

        /// <summary>
        /// Takes back a key whose request never came back. Otherwise it stays in flight forever: every later
        /// <see cref="RequestRefresh"/> returns "already queued", and the caller's synchronous fallback runs
        /// only when a request fails. A step-loop fault completes its request as a decline, so the way in is a
        /// worker that is gone. A request a worker has taken is judged on its solve time
        /// (<see cref="StrandedKeyTimeoutReal"/>), one still queued on its wait (<see cref="QueuedKeyTimeoutReal"/>),
        /// so a backlog is not mistaken for a dead worker. The stranded request is not cancelled; if it
        /// finishes, <see cref="DrainCompleted"/> discards it because it is no longer the request its key waits
        /// on. Real time, not sim time: a paused or compressed game must not change how long a stopped thread
        /// takes to notice. Main thread only.
        /// </summary>
        private static bool EvictIfStranded(TofKey key)
        {
            if (!_inFlight.TryGetValue(key, out SolveRequest r)) return false;
            long started = Interlocked.Read(ref r.StartedTicks);
            bool solving = started != 0;
            float age = solving
                ? (float)((System.Diagnostics.Stopwatch.GetTimestamp() - started) /
                          (double)System.Diagnostics.Stopwatch.Frequency)
                : Time.realtimeSinceStartup - r.CapturedReal;
            if (age < (solving ? StrandedKeyTimeoutReal : QueuedKeyTimeoutReal)) return false;

            _inFlight.Remove(key);
            AsyncStranded++;
            Bootstrap.Log.LogWarning(
                $"[AutoTOT] estimator: a solve key had been {(solving ? "solving" : "queued")} {age:0.0}s " +
                $"with no result, so it was evicted and will be re-queued. " +
                $"workerFaults {WorkerFaults}, aborts {WorkerAborts}, stranded so far {AsyncStranded}.");
            return true;
        }

        /// <summary>
        /// True while a worker is already solving this key, so a caller that must not block can wait one more
        /// tick instead of solving on the main thread. Main thread only.
        /// </summary>
        internal static bool IsQueued(ObjectBase unit, string ammoId, ObjectBase target)
        {
            if (_workers == null) return false;
            if (!TryKey(unit, ammoId, target, out _, out TofKey key)) return false;
            if (!_inFlight.ContainsKey(key)) return false;
            // Same eviction as RequestRefresh: otherwise a deferred commit waiting on a stranded key would
            // wait for the whole mission.
            return !EvictIfStranded(key);
        }

        /// <summary>
        /// Publish finished results into the cache. Main thread only, called once at the top of the
        /// tick, so the cache keeps a single writer.
        /// </summary>
        internal static void DrainCompleted()
        {
            while (_done.TryDequeue(out SolveRequest r))
            {
                // Only the request its key waits on publishes. Anything else was dropped by a mission reset
                // (its ids belong to the previous mission) or evicted as stranded, possibly with a newer request
                // for the same key now in flight, whose answer must not be overwritten or lost.
                if (!_inFlight.TryGetValue(r.Key, out SolveRequest current) || !ReferenceEquals(current, r))
                    continue;
                _inFlight.Remove(r.Key);
                AsyncCompleted++;
                // How old the geometry behind this answer is by the time anything can read it.
                LastPublishAgeReal = Time.realtimeSinceStartup - r.CapturedReal;
                if (LastPublishAgeReal > MaxPublishAgeReal) MaxPublishAgeReal = LastPublishAgeReal;

                if (VerifySolve) VerifyAgainstMainThread(r);

                // A synchronous solve answered this key after this request was captured, so the cache holds
                // fresher geometry. Checked by stamp rather than by dropping the key from _inFlight at the
                // synchronous write, which would let the key queue again and the older result publish last.
                bool superseded = _syncWrittenAt.TryGetValue(r.Key, out float syncAt) &&
                                  syncAt > r.CapturedSim;
                _syncWrittenAt.Remove(r.Key);
                if (superseded) { AsyncSuperseded++; continue; }

                // The synchronous path's plausibility bound; otherwise a worker could publish an impossible
                // flight time that callers take as fact for half a second.
                bool implausible = r.Unit != null && !r.Unit.IsDestroyed &&
                                   r.Target != null && !r.Target.IsDestroyed &&
                                   IsImplausible(r.Result, r.Unit, r.Ap, r.Target);
                if (r.Result > MinValidSeconds && !implausible)
                {
                    // Dated to the capture, so a reader's age is that of the geometry, queue and solve included.
                    _cache.Set(r.Key, r.Result, r.CapturedUnscaled);
                    ModelStats.TierUsed(ModelStats.Tier.Integrator);
                    continue;
                }
                if (implausible)
                    Bootstrap.Log.LogWarning(
                        $"[AutoTOT] estimate-rejected {r.AmmoId} (worker): {r.Result:0.00}s against " +
                        $"a straight-line floor of " +
                        $"{StraightLineFloorSeconds(r.Unit, r.Ap, r.Target):0.0}s. Not cached.");

                // The loop declined, as it does for a shot it cannot close (a SAM planned beyond its reach).
                // The remaining tiers need ObjectBase, so they run here, and only they: re-running the integrator
                // would repeat on the main thread the most expensive solve there is, one that ran to maxFlight.
                AsyncDeclined++;
                ModelStats.Stalled();
                if (r.Unit == null || r.Unit.IsDestroyed || r.Target == null || r.Target.IsDestroyed)
                    continue;
                float v = LowerTiers(r.Unit, r.Ap, r.Target);
                _cache.Set(r.Key, v);
            }
        }

        /// <summary>
        /// Drop everything queued or in flight, on mission end: Unity reuses instance ids, so a late result
        /// could publish one mission's flight time under another's key. Results still in workers are
        /// discarded on arrival. Main thread only.
        /// </summary>
        internal static void ResetQueues()
        {
            while (_pending.TryDequeue(out _)) { }
            while (_done.TryDequeue(out _)) { }
            _inFlight.Clear();
            _syncWrittenAt.Clear();
            // Per mission, so one fault does not keep the census line printing for the rest of the session.
            Interlocked.Exchange(ref WorkerFaults, 0);
            Interlocked.Exchange(ref WorkerAborts, 0);
            AsyncSuperseded = 0;
            AsyncStranded = 0;
            LastPublishAgeReal = 0f;
            MaxPublishAgeReal = 0f;
            AsyncCompleted = 0;
            AsyncDeclined = 0;
            VerifyChecked = 0;
            VerifyMismatched = 0;
        }

        /// <summary>
        /// Worker pool health and result freshness, one line on a sim-time cadence: `oldestInFlight` should
        /// stay within a tick or two of the solve time (otherwise a key is stranded), and `publishAge` is how
        /// old the geometry behind a published result is, measured from the main-thread capture. Main thread
        /// only; silent when the pool is off or idle.
        /// </summary>
        internal static void LogAsyncCensus()
        {
            if (_workers == null) return;
            if (AsyncCompleted == 0 && _inFlight.Count == 0 && WorkerFaults == 0) return;

            float now = Time.realtimeSinceStartup;
            float oldest = 0f;
            foreach (KeyValuePair<TofKey, SolveRequest> kv in _inFlight)
            {
                float age = now - kv.Value.CapturedReal;
                if (age > oldest) oldest = age;
            }

            string faults = WorkerFaults > 0
                ? $", workerFaults {WorkerFaults} (NOT shutdown aborts: those shots fell back to a lower tier)"
                : ", workerFaults 0";
            Bootstrap.Log.LogInfo(
                $"[AutoTOT] estimator-census: inFlight {_inFlight.Count}, queued {_pending.Count}, " +
                $"completed {AsyncCompleted}, declined {AsyncDeclined}, superseded {AsyncSuperseded}, " +
                $"stranded {AsyncStranded}, " +
                $"oldestInFlight {oldest:0.00}s, " +
                $"publishAge last {LastPublishAgeReal:0.00}s max {MaxPublishAgeReal:0.00}s, " +
                $"aborts {WorkerAborts}{faults}.");
        }

        /// <summary>
        /// Re-solve a completed request here on the main thread and compare with what the worker
        /// returned. Exact equality is required: the same input through the same pure code must give
        /// the same float, so this is not a tolerance check.
        /// </summary>
        private static void VerifyAgainstMainThread(SolveRequest r)
        {
            IntegratedPhases ph = default;
            ModelStats.LoopStarting();   // see DumpInputFor: Solve times against this stamp
            float mine;
            try
            {
                mine = Solve(in r.Input, r.Ap, ref ph);
            }
            catch (Exception e)
            {
                // A verification run must not turn a fault the worker already absorbed into a tick error.
                ModLog.Warn("solve-verify main-thread solve", e);
                return;
            }
            VerifyChecked++;
            if (mine.Equals(r.Result)) return;   // bitwise, so NaN == NaN counts as agreement
            VerifyMismatched++;
            Bootstrap.Log.LogWarning(
                $"[AutoTOT] solve-verify MISMATCH {r.AmmoId}: worker {r.Result:F4}s, " +
                $"main {mine:F4}s, delta {(r.Result - mine):F4}s. The snapshot is not carrying " +
                $"every value the loop reads.");
        }

        /// <summary>
        /// Cache-only read, for callers that must not block. Main thread only.
        ///
        /// <para>A cached decline (-1) must look like a miss. <see cref="Kinematic"/> caches whatever the
        /// tier chain answers, and <see cref="Estimate"/> substitutes the straight-line tier for a decline,
        /// which this method cannot do because it must not run a tier. Raw, the -1 would reach callers as a
        /// flight time: the anchor scan would never let the item win, and the release gate would release the
        /// shot at once. A miss sends both to <see cref="Estimate"/>.</para>
        /// </summary>
        internal static bool TryCached(ObjectBase unit, string ammoId, ObjectBase target, out float value)
            => TryCached(unit, ammoId, target, out value, out _);

        /// <summary>
        /// As above, plus the value's age in real seconds: the cache is shared, so a hit may be up to the 0.5
        /// s TTL old, which is 10 sim seconds at the 20x compression cap. A worker's answer is dated to its
        /// input snapshot, so its age also counts the time it spent queued and solving.
        /// </summary>
        internal static bool TryCached(ObjectBase unit, string ammoId, ObjectBase target,
                                       out float value, out float ageReal)
        {
            value = 0f; ageReal = 0f;
            if (!TryKey(unit, ammoId, target, out _, out TofKey key)) return false;
            if (!_cache.TryGet(key, out float cached, out float age) || cached <= MinValidSeconds)
                return false;
            value = cached;
            ageReal = age;
            return true;
        }
    }
}
