using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using SeaPower;
using UnityEngine;

namespace AutoTOT
{
    /// <summary>
    /// Compares <see cref="FlightTime.AltitudeNodes"/> with the game's own <c>MissileSimulator.BuildAltitudeNodes</c>
    /// on the same inputs. The lab passes the 0.8.x method (6 parameters, returns a list) as the reference; the
    /// live test runner passes 0.9.0's (FlyoutBuffers), which only exists inside the running game. Both sides
    /// get the same cases: a geometry grid for each real round and randomised ammunition that reaches every
    /// branch of the function. Diagnostic only, never in the Workshop build.
    /// </summary>
    internal sealed class AltitudeNodeCheck
    {
        internal delegate List<Vector2> Reference(AmmunitionParameters ap, float launchAlt, float targetAlt,
                                                  float flatDistTotal, float loftAltOverride,
                                                  out float loftEndFlatOut);

        private const int MaxSamples = 20;

        internal int Cases;
        internal int Exact;
        internal int CountMismatches;
        internal int ValueMismatches;
        internal int PortDeclined;
        internal int ReferenceFailed;
        internal double MaxAbsDiff;
        internal readonly List<string> Samples = new List<string>();

        private readonly Reference _reference;

        internal AltitudeNodeCheck(Reference reference) { _reference = reference; }

        // Launch heights from a deck to a high aircraft, target heights from the sea to a high aircraft, legs
        // from 0.4 nm to 290 nm, and the loft override the game sees: none (-1), zero, and real heights.
        private static readonly float[] LaunchAlts = { 0f, 0.05f, 2f, 30f, 136f };
        private static readonly float[] TargetAlts = { 0f, 0.2f, 5f, 90f };
        private static readonly float[] Flats = { 10f, 40f, 150f, 500f, 1000f, 2000f, 4000f, 8000f };
        private static readonly float[] Overrides = { -1f, 0f, 20f, 100f, 400f };

        internal static IEnumerable<float[]> Grid()
        {
            foreach (float la in LaunchAlts)
                foreach (float ta in TargetAlts)
                    foreach (float flat in Flats)
                        foreach (float lo in Overrides)
                            yield return new[] { la, ta, flat, lo };
        }

        /// <summary>Runs every grid case on one round.</summary>
        internal void GridFor(string label, AmmunitionParameters ap)
        {
            foreach (float[] g in Grid())
                Compare(label, ap, g[0], g[1], g[2], g[3]);
        }

        internal void Compare(string label, AmmunitionParameters ap, float launchAlt, float targetAlt,
                              float flatDistTotal, float loftAltOverride)
        {
            Cases++;
            List<Vector2> port = FlightTime.AltitudeNodes(ap, launchAlt, targetAlt, flatDistTotal, loftAltOverride,
                                                          out float portEnd);
            if (port == null) { PortDeclined++; return; }
            List<Vector2> game;
            float gameEnd;
            try
            {
                game = _reference(ap, launchAlt, targetAlt, flatDistTotal, loftAltOverride, out gameEnd);
            }
            catch (Exception e)
            {
                ReferenceFailed++;
                Note($"{label}: reference threw {e.GetType().Name}: {e.Message}");
                return;
            }
            if (game == null) { ReferenceFailed++; return; }

            string where = $"{label} launch {F(launchAlt)} target {F(targetAlt)} flat {F(flatDistTotal)} " +
                           $"override {F(loftAltOverride)}";
            if (game.Count != port.Count)
            {
                CountMismatches++;
                Note($"{where}: {port.Count} nodes, game {game.Count}");
                return;
            }
            double worst = Math.Abs(gameEnd - portEnd);
            for (int i = 0; i < game.Count; i++)
            {
                worst = Math.Max(worst, Math.Abs(game[i].x - port[i].x));
                worst = Math.Max(worst, Math.Abs(game[i].y - port[i].y));
            }
            MaxAbsDiff = Math.Max(MaxAbsDiff, worst);
            if (worst == 0.0) Exact++;
            else
            {
                ValueMismatches++;
                Note($"{where}: largest difference {worst:G6}u over {game.Count} nodes");
            }
        }

        private void Note(string s) { if (Samples.Count < MaxSamples) Samples.Add(s); }

        internal string Summary(string name) =>
            $"{name}: {Cases} cases, {Exact} bit-identical, {CountMismatches} node-count mismatches, " +
            $"{ValueMismatches} value mismatches (largest {MaxAbsDiff:G6}u), {PortDeclined} declined by the port " +
            $"(non-kinematic), {ReferenceFailed} reference failures";

        private static string F(float v) => v.ToString("0.###", CultureInfo.InvariantCulture);

        private static readonly FieldInfo UseKinematics =
            typeof(AmmunitionParameters).GetField("_useKinematics", BindingFlags.Instance | BindingFlags.NonPublic);

        /// <summary>
        /// Randomises every field BuildAltitudeNodes and LoftCap read, so the cases reach each branch: AAW and
        /// surface rounds, ballistic and cubic lofts, terminal dive, pop-up, terminal loft, loft-to-skim and every
        /// LoftMode. A field is often set to exactly zero, since most branches test "> 0".
        /// </summary>
        internal static void Randomise(AmmunitionParameters ap, System.Random rng)
        {
            float Pick(float max) => rng.Next(3) == 0 ? 0f : (float)(rng.NextDouble() * max);
            bool Coin() => rng.Next(2) == 0;

            UseKinematics?.SetValue(ap, rng.Next(4) == 0 ? (bool?)null : (bool?)true);
            Array targets = Enum.GetValues(typeof(Ammunition.Target));
            ap._targetType = (Ammunition.Target)targets.GetValue(rng.Next(targets.Length));
            Array modes = Enum.GetValues(typeof(AmmunitionParameters.LoftMode));
            ap._loftAltMode = (AmmunitionParameters.LoftMode)modes.GetValue(rng.Next(modes.Length));
            ap._maxLoftAltUnity = Pick(500f);
            ap._maxLoftAngle = Pick(95f);
            ap._finalFlightPhaseMaxAngle = Pick(95f);
            ap._seaSkimmingMaxDescentAngle = Pick(95f);
            ap._loftToSkim = Coin();
            ap._terminalLoft = Coin();
            ap._terminalDiveDistance = Pick(600f);
            ap._terminalApproachDist = Pick(900f);
            ap._terminalPopUpDist = Pick(300f);
            ap._terminalAltUnity = rng.Next(4) == 0 ? -1f : Pick(60f);
            ap._finalFlightPhaseDistToTargetUnity = Pick(1500f);
            ap._finalFlightPhaseAltUnity = Pick(60f);
            ap._seaSkimmingStartDistToTargetUnity = Pick(3000f);
            ap._seaSkimmingAltUnity = Pick(5f);
            FlightTime.BallisticLoftField?.SetValue(ap, Coin());
        }

        /// <summary>One random geometry, drawn from the same ranges as the grid.</summary>
        internal static float[] RandomGeometry(System.Random rng) => new[]
        {
            rng.Next(3) == 0 ? 0f : (float)(rng.NextDouble() * 140.0),
            rng.Next(3) == 0 ? 0f : (float)(rng.NextDouble() * 100.0),
            (float)(rng.NextDouble() * 8000.0) + 0.1f,
            rng.Next(4) == 0 ? -1f : (float)(rng.NextDouble() * 500.0),
        };
    }
}
