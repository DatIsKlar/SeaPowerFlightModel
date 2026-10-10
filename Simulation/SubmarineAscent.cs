using System;
using System.Text;
using UnityEngine;

namespace AutoTOT
{
    /// <summary>
    /// How long a submarine needs to reach the depth its weapon can launch from, by stepping the
    /// game's own vertical model. Two mechanisms (ballast and dive planes) are mutually exclusive;
    /// the interlock between them changes the answer several-fold. Pure arithmetic, no Unity access.
    /// See docs/model/07-diagnostics.md §7.6 for equations, measurement evidence, and limits.
    /// </summary>
    internal static class SubmarineAscent
    {
        private const float StepSim = 0.1f;             // matches FlightTime's IntegrationStepSim
        private const int MaxSteps = 12000;             // 20 min of sim at StepSim; a real ascent is ~1 min

        // Interlock thresholds, from applyDepth. Named rather than inlined because the whole model
        // turns on them: above BOTH, ballast is zeroed and the boat is on its planes alone.
        private const float InterlockNormSpeed = 0.1f;
        private const float InterlockPitchDeg = 5f;

        // No-progress bail: a boat that cannot make its ordered depth (too slow to hold a bow-up angle, or
        // held down by the interlock) never arrives, and the answer is "never", not a large number.
        private const float MinProgressUnity = 1e-5f;
        private const int NoProgressSteps = 50;         // 5s of sim making no headway
        // A boat whose vertical rate is still falling (ballast ramping out of a descent, planes still pitching up)
        // is reversing, not stalled, however long the reversal takes: the tanks ramp at about 0.055 ft/s^2, so
        // undoing a 5 ft/s descent alone takes some 90 s.
        private const float MinRateImprovementU = 1e-7f;

        /// <summary>Seed for one ascent. Plain data: no Unity types, so the loop is testable.</summary>
        internal struct State
        {
            public float DepthU;          // current, unity units, positive down
            public float TargetDepthU;    // where the launcher is taking it, same convention
            // The launch gate, same convention: the weapon's own depth ceiling, which the launcher passes on its way
            // to TargetDepthU (WeaponSystemLauncher.cs:414-419 commands 0.115 u above the ceiling and stops
            // blocking at the ceiling). Arrival is the first of the two the boat reaches; 0, the default, leaves
            // it at TargetDepthU.
            public float GateDepthU;
            public float PitchDeg;        // current hull pitch, positive nose-down
            public float BallastRateU;    // _currentDepthChangeUsingTanks, unity/s, positive down
            public float SpeedKn;         // held constant for the ascent
            public float MaxSpeedKn;      // _maxForwardVelocitySubmergedInKnots
            public float MaxPitchDeg;     // _maxPitchAngle
            public float PitchRateDeg;    // _pitchChangeRate, deg/s
            public float BallastAccelU;   // _ballastTankChangeRate, unity/s^2
            public float MaxBallastU;     // _maxDepthChangeUsingTanks, unity/s
        }

        /// <summary>
        /// Seconds to reach the launch gate (<see cref="State.GateDepthU"/>, or <see cref="State.TargetDepthU"/>
        /// when that is deeper or no gate is given), steering toward TargetDepthU as the game does. False when
        /// the boat makes no headway (already the answer for a depth-deadlocked order) or the seed is unusable.
        /// <paramref name="trace"/> receives a sampled profile for the asc-sim diagnostic when a
        /// builder is supplied; pass null on the hot path.
        /// </summary>
        internal static bool TryEstimate(State s, out float seconds, StringBuilder trace)
        {
            seconds = 0f;
            if (!GameMath.IsFinite(s.DepthU) || !GameMath.IsFinite(s.TargetDepthU) || s.MaxSpeedKn <= 0f ||
                s.PitchRateDeg <= 0f || !GameMath.IsFinite(s.SpeedKn)) return false;

            // Depth is positive down, so the deeper of the two is the one reached first on the way up.
            float arriveU = Math.Max(s.TargetDepthU, GameMath.IsFinite(s.GateDepthU) ? s.GateDepthU : 0f);
            float toGo = s.DepthU - arriveU;
            if (toGo <= 0f) { seconds = 0f; return true; }   // already shallow enough

            float depth = s.DepthU, pitch = s.PitchDeg, ballast = s.BallastRateU;
            float normSpd = Math.Abs(s.SpeedKn) / s.MaxSpeedKn;
            float speedU = s.SpeedKn * GameUnits.KnotsToUnityPerSecond;
            float t = 0f, nextSample = 0f;
            float prevRate = float.PositiveInfinity;
            int stalled = 0;

            for (int i = 0; i < MaxSteps; i++)
            {
                float err = s.TargetDepthU - depth;      // negative while ascending

                // Ballast: target rate IS the depth error, clamped, then ramped toward.
                float ballTgt = Mathf.Clamp(err, -s.MaxBallastU, s.MaxBallastU);
                if (ballast < ballTgt) ballast = Math.Min(ballast + s.BallastAccelU * StepSim, ballTgt);
                else if (ballast > ballTgt) ballast = Math.Max(ballast - s.BallastAccelU * StepSim, ballTgt);

                // Dive planes: pitch authority scales with speed AND collapses near the surface.
                float depthFac = Mathf.Clamp01(2f * depth);
                float pitchLimit = s.MaxPitchDeg * depthFac;
                float tgtPitch = normSpd * Mathf.Clamp(s.MaxPitchDeg * err, -pitchLimit, pitchLimit);
                if (pitch < tgtPitch) pitch = Math.Min(pitch + s.PitchRateDeg * StepSim, tgtPitch);
                else if (pitch > tgtPitch) pitch = Math.Max(pitch - s.PitchRateDeg * StepSim, tgtPitch);

                // The interlock: under way and pitched, the boat flies its planes and blows no tanks. This
                // zeroes the state, not just this step's contribution, as the game does
                // (`_currentDepthChangeUsingTanks = 0f`): left charging, ballast would be released fully
                // wound up at its 5 ft/s cap the moment pitch drops below the threshold near the target, and
                // the boat would finish far too fast. It must ramp again from zero at _ballastTankChangeRate.
                if (normSpd > InterlockNormSpeed && Math.Abs(pitch) > InterlockPitchDeg) ballast = 0f;

                float rate = ballast + speedU * (float)Math.Sin(pitch * Math.PI / 180.0);
                float step = rate * StepSim;
                depth += step;
                t += StepSim;

                if (trace != null && t >= nextSample)
                {
                    nextSample += TelemetryCadence.SampleIntervalSim;
                    trace.Append($" t+{t:0}s {depth * GameUnits.UnityToFeet:0}ft p{pitch:0.0} " +
                                 $"r{-rate * GameUnits.UnityToFeet:0.00}ft/s{(ballast == 0f ? " planes" : " tanks")};");
                }

                if (depth <= arriveU) { seconds = t; return true; }

                // Ascending means depth decreasing, so progress is a negative step. No progress counts toward the
                // bail only while the rate is not improving either.
                bool improving = rate < prevRate - MinRateImprovementU;
                prevRate = rate;
                if (step > -MinProgressUnity && !improving) { if (++stalled >= NoProgressSteps) return false; }
                else stalled = 0;
            }
            return false;   // ran out of steps: treat as unreachable rather than guessing
        }

    }
}
