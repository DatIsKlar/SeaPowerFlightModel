using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using SeaPower;
using UnityEngine;

namespace AutoTOT.Lab
{
    /// <summary>
    /// <c>nodecheck [corpus-dir] [--fuzz N]</c>: the ported altitude-node function against the game's.
    /// Three parts, all on the game DLL the lab was built against:
    /// <list type="number">
    /// <item>recorded: every capture that stored the game's nodes, rebuilt from its own inputs;</item>
    /// <item>grid: each lofting round of the corpus over a grid of launch heights, target heights, legs and
    /// loft overrides, port and game side by side;</item>
    /// <item>fuzz: randomised ammunition that reaches every branch, port and game side by side.</item>
    /// </list>
    /// Parts 2 and 3 need the game's 6-parameter method, which exists up to 0.8.7; on 0.9.0 the game side runs
    /// only inside the game (LiveTestRunner NodeSweep).
    /// </summary>
    internal static partial class Program
    {
        private static int NodeCheck(string[] args)
        {
            string dir = args.FirstOrDefault(a => !a.StartsWith("--")) ?? DefaultCorpus();
            int fuzz = 100000;
            int at = Array.IndexOf(args, "--fuzz");
            if (at >= 0 && at + 1 < args.Length) fuzz = int.Parse(args[at + 1], Inv);

            Type ms = typeof(Missile).Assembly.GetType(FlightTime.MissileSimulatorType);
            MethodInfo game = ms?.GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static)
                .FirstOrDefault(m => m.Name == FlightTime.AltitudeNodesMethodName &&
                                     m.GetParameters().Length == FlightTime.AltitudeNodesParameterCount &&
                                     m.ReturnType == typeof(List<Vector2>));
            AltitudeNodeCheck.Reference reference = null;
            if (game != null)
                reference = (AmmunitionParameters ap, float la, float ta, float flat, float lo, out float end) =>
                {
                    object[] a = { ap, la, ta, flat, lo, 0f };
                    var nodes = (List<Vector2>)game.Invoke(null, a);
                    end = (float)a[5];
                    return nodes;
                };

            // 1. Recorded nodes.
            int recorded = 0, recordedExact = 0, recordedClose = 0, recordedDeclined = 0;
            double recordedWorst = 0;
            var recordedSamples = new List<string>();
            // The same rebuilt inputs through the game's own method, where the DLL has it: if port and game agree on
            // a capture, a difference to its stored nodes comes from rebuilding the inputs, not from the port.
            var sameInputs = reference == null ? null : new AltitudeNodeCheck(reference);
            var lofters = new Dictionary<string, AmmunitionParameters>();
            foreach (string file in Directory.GetFiles(dir, "*.solveinput").OrderBy(f => f, StringComparer.Ordinal))
            {
                Dictionary<string, string> kv = ReadDump(file);
                if (B(kv, "NonKin") || !B(kv, "Lofting")) continue;
                AmmunitionParameters ap;
                try { ap = BuildAmmo(kv); }
                catch (Exception) { continue; }
                string label = kv.GetValueOrDefault("AmmoLabel", Path.GetFileName(file));
                if (!lofters.ContainsKey(label)) lofters[label] = ap;

                Vector2[] stored = Nodes(kv, "AltNodes");
                if (stored == null || stored.Length == 0 || !kv.ContainsKey("ConfiguredLoftAlt")) continue;
                recorded++;
                float la = Mathf.Max(V3(kv, "Pos").y, 0f), ta = F(kv, "TargetAlt0"),
                      flat = F(kv, "FlatDistTotal"), lo = F(kv, "LoftAlt");
                sameInputs?.Compare(Path.GetFileNameWithoutExtension(file), ap, la, ta, flat, lo);
                List<Vector2> port = FlightTime.AltitudeNodes(ap, la, ta, flat, lo, out float end);
                if (port == null) { recordedDeclined++; continue; }
                double worst = Math.Abs(end - F(kv, "KappaEndFlat"));
                if (port.Count != stored.Length) worst = double.PositiveInfinity;
                else
                    for (int i = 0; i < stored.Length; i++)
                        worst = Math.Max(worst, Math.Max(Math.Abs(stored[i].x - port[i].x),
                                                         Math.Abs(stored[i].y - port[i].y)));
                if (worst == 0.0) recordedExact++;
                else if (worst < 0.001) recordedClose++;
                else if (recordedSamples.Count < 20)
                    recordedSamples.Add($"{Path.GetFileNameWithoutExtension(file)}: " +
                                        (double.IsInfinity(worst) ? $"{port.Count} nodes, capture {stored.Length}"
                                                                  : $"largest difference {worst:G6}u"));
                if (!double.IsInfinity(worst)) recordedWorst = Math.Max(recordedWorst, worst);
            }
            Console.WriteLine($"recorded: {recorded} captures with stored nodes, {recordedExact} bit-identical, " +
                              $"{recordedClose} within 0.001u, " +
                              $"{recorded - recordedExact - recordedClose - recordedDeclined} further off " +
                              $"(largest finite {recordedWorst:G6}u), {recordedDeclined} declined");
            foreach (string s in recordedSamples) Console.WriteLine("  " + s);
            if (sameInputs != null)
            {
                Console.WriteLine(sameInputs.Summary("recorded inputs, port vs game"));
                foreach (string s in sameInputs.Samples) Console.WriteLine("  " + s);
            }

            if (reference == null)
            {
                Console.WriteLine("grid and fuzz skipped: this game DLL has no 6-parameter BuildAltitudeNodes " +
                                  "(0.9.0 or later); run them in the game with the NodeSweep plan key.");
                return recorded == recordedExact ? 0 : 1;
            }

            // 2. Grid over every lofting round of the corpus.
            var grid = new AltitudeNodeCheck(reference);
            foreach (KeyValuePair<string, AmmunitionParameters> kv in lofters.OrderBy(k => k.Key, StringComparer.Ordinal))
                grid.GridFor(kv.Key, kv.Value);
            Console.WriteLine(grid.Summary($"grid ({lofters.Count} rounds)"));
            foreach (string s in grid.Samples) Console.WriteLine("  " + s);

            // 3. Randomised ammunition, a fixed seed so a rerun draws the same cases.
            var random = new AltitudeNodeCheck(reference);
            var rng = new System.Random(20261112);
            for (int i = 0; i < fuzz; i++)
            {
                var ap = new AmmunitionParameters();
                AltitudeNodeCheck.Randomise(ap, rng);
                float[] g = AltitudeNodeCheck.RandomGeometry(rng);
                random.Compare($"fuzz#{i}", ap, g[0], g[1], g[2], g[3]);
            }
            Console.WriteLine(random.Summary("fuzz"));
            foreach (string s in random.Samples) Console.WriteLine("  " + s);

            // Judged on port against game. Against the stored nodes the lab's rebuilt ammunition is only as exact as
            // the capture, which for older captures (yj-20 above all) it is not.
            bool clean = sameInputs.Exact == sameInputs.Cases && grid.Exact + grid.PortDeclined == grid.Cases &&
                         random.Exact + random.PortDeclined == random.Cases;
            return clean ? 0 : 1;
        }
    }
}
