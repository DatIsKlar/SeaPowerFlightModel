using System;
using System.Collections.Generic;
using SeaPower;
using UnityEngine;

namespace AutoTOT
{
    internal static partial class FlightTime
    {
        /// <summary>
        /// The game's altitude-node curve for a kinematic or AAW round: <c>MissileSimulator.BuildAltitudeNodes</c>,
        /// ported line for line from decompiled-beta-new (0.8.6) MissileSimulator.cs:577-714. Game 0.8.7 has the
        /// same body; 0.9.0 (MissileSimulator.cs:1332-1478) keeps the arithmetic but writes into Unity native
        /// buffers (<c>FlyoutBuffers</c>), which neither the offline lab nor the solve workers can allocate, so the
        /// model carries this copy on every build. Operations keep the game's order so the floats come out the same.
        /// Returns null where the game takes its non-kinematic branch (<c>AppendNonKinematicStages</c>), which the
        /// model never asks for, and when <c>LoftCap</c> is not bound.
        /// </summary>
        internal static List<Vector2> AltitudeNodes(AmmunitionParameters ap, float launchAlt, float targetAlt,
                                                    float flatDistTotal, float loftAltOverride,
                                                    out float loftEndFlatOut)
        {
            loftEndFlatOut = 0f;
            bool aaw = ap._targetType == Ammunition.Target.AAW;
            if (ap.Kinematics == AmmunitionParameters.KinematicsLevel.None && !aaw) return null;
            EnsureSimLookup();
            if (_loftCapMethod == null) return null;

            var nodes = new List<Vector2> { new Vector2(0f, launchAlt) };
            // (stage start x, stage altitude, stage angle in degrees, earliest x the transition may begin);
            // at most one terminal, one final-flight and one sea-skimming stage.
            var checkpoints = new Vector4[3];
            int checkpointCount = 0;

            float tailDist = 0f;                                                           // num
            float finalAngle = ap._finalFlightPhaseMaxAngle > 0.01f ? ap._finalFlightPhaseMaxAngle : 10f;
            float skimAngle = ap._seaSkimmingMaxDescentAngle > 0.01f ? ap._seaSkimmingMaxDescentAngle : 10f;
            float loftCap = (float)_loftCapMethod.Invoke(null, new object[] { ap, launchAlt, targetAlt });
            bool lofts = !ap._loftToSkim && loftCap > 0f;                                  // flag2
            if (ap._terminalDiveDistance > 0f && ap._terminalDiveDistance < ap._terminalApproachDist && !aaw &&
                !ap._terminalLoft)
                tailDist = ap._terminalDiveDistance;
            if (ap._terminalPopUpDist > 0f)
                tailDist = ap._terminalPopUpDist;
            if (ap._terminalApproachDist > 0f && !ap._terminalLoft)
            {
                float terminalAlt = aaw ? targetAlt
                    : ap._terminalAltUnity > 0f ? ap._terminalAltUnity
                    : ap._finalFlightPhaseDistToTargetUnity > 0f && ap._finalFlightPhaseAltUnity > 0f
                        ? ap._finalFlightPhaseAltUnity
                    : !(ap._seaSkimmingStartDistToTargetUnity > 0f) || !(ap._seaSkimmingAltUnity > 0f)
                        ? -1f
                    : ap._seaSkimmingAltUnity;
                if (terminalAlt >= 0f)
                {
                    checkpoints[checkpointCount++] = new Vector4(flatDistTotal - tailDist,
                        Mathf.Max(terminalAlt, 0f), skimAngle, flatDistTotal - ap._terminalApproachDist);
                    tailDist = ap._terminalApproachDist;
                }
            }
            if (ap._finalFlightPhaseDistToTargetUnity > 0f)
            {
                checkpoints[checkpointCount++] = new Vector4(flatDistTotal - tailDist,
                    Mathf.Max(ap._finalFlightPhaseAltUnity, 0f), finalAngle,
                    flatDistTotal - ap._finalFlightPhaseDistToTargetUnity);
                tailDist = ap._finalFlightPhaseDistToTargetUnity;
            }
            if (ap._seaSkimmingStartDistToTargetUnity > 0f && !lofts)
            {
                checkpoints[checkpointCount++] = new Vector4(flatDistTotal - tailDist,
                    aaw ? targetAlt : Mathf.Max(ap._seaSkimmingAltUnity, 0f), skimAngle,
                    flatDistTotal - ap._seaSkimmingStartDistToTargetUnity);
                tailDist = ap._seaSkimmingStartDistToTargetUnity;
            }

            float loftLeg = flatDistTotal - tailDist;                                       // num4
            loftEndFlatOut = Mathf.Max(loftLeg, 0f);
            float loftTop = loftCap > 0f                                                    // num5
                ? Mathf.Max(loftAltOverride >= 0f ? loftAltOverride : loftCap, launchAlt) : -1f;
            if (loftTop > 0f && loftLeg > 1f)
            {
                float targetFloor = Mathf.Max(targetAlt, 0f);                               // num6
                float climbTan = Mathf.Tan(Mathf.Clamp(ap._maxLoftAngle, 0.5f, 89f) * (MathF.PI / 180f));
                bool ballisticLoft = BallisticLoftOf(ap);
                float apex;                                                                 // num8
                float climbDist;                                                            // num11
                if (ballisticLoft)
                {
                    apex = Mathf.Max(Mathf.Max(loftTop, launchAlt), targetFloor);
                    float up = Mathf.Sqrt(Mathf.Max(apex - launchAlt, 0f));
                    float down = Mathf.Sqrt(Mathf.Max(apex - targetFloor, 0f));
                    climbDist = up + down > 0.0001f ? loftLeg * Mathf.Clamp(up / (up + down), 0f, 0.95f) : 0f;
                }
                else
                {
                    float descentTan = Mathf.Tan(Mathf.Clamp(
                        ap._finalFlightPhaseMaxAngle > 0.01f ? ap._finalFlightPhaseMaxAngle : 45f, 0.5f, 89f) *
                        (MathF.PI / 180f));
                    float climbRun = 1f / climbTan;                                         // num13
                    float descentRun = 1f / descentTan;                                     // num14
                    float reachable = (loftLeg + launchAlt * climbRun + targetFloor * descentRun) /
                                      (climbRun + descentRun);
                    apex = Mathf.Max(Mathf.Min(loftTop, reachable), Mathf.Max(launchAlt, targetFloor));
                    climbDist = Mathf.Clamp((apex - launchAlt) * climbRun, 0f, loftLeg);
                }
                if (climbDist > 0.5f && apex > launchAlt + 0.5f)
                {
                    int steps = Mathf.Clamp(Mathf.CeilToInt((apex - launchAlt) / 25f) + 1, 3, 12);
                    for (int i = 1; i <= steps; i++)
                    {
                        float t = (float)i / (float)steps;
                        float shape = ballisticLoft ? 1f - (1f - t) * (1f - t) : (0f - t) * t * t + t * t + t;
                        nodes.Add(new Vector2(climbDist * t, launchAlt + (apex - launchAlt) * shape));
                    }
                }
                float lastX = nodes[nodes.Count - 1].x;
                if (loftLeg > lastX + 0.5f)
                {
                    float lastY = nodes[nodes.Count - 1].y;
                    float run = loftLeg - lastX;
                    int steps = Mathf.Clamp(Mathf.CeilToInt(Mathf.Abs(lastY - targetFloor) / 25f) + 1, 3, 12);
                    for (int j = 1; j <= steps; j++)
                    {
                        float t = (float)j / (float)steps;
                        float shape = ballisticLoft ? t * t : t * t * t;
                        nodes.Add(new Vector2(lastX + run * t, lastY + (targetFloor - lastY) * shape));
                    }
                }
            }

            // Stable insertion sort on x, as 0.9.0 does; 0.8.x's List.Sort agrees on three or fewer entries.
            for (int k = 1; k < checkpointCount; k++)
            {
                Vector4 value = checkpoints[k];
                int m = k - 1;
                while (m >= 0 && checkpoints[m].x > value.x)
                {
                    checkpoints[m + 1] = checkpoints[m];
                    m--;
                }
                checkpoints[m + 1] = value;
            }
            for (int c = 0; c < checkpointCount; c++)
            {
                Vector4 stage = checkpoints[c];
                Vector2 last = nodes[nodes.Count - 1];
                if (stage.x <= last.x + 0.5f || stage.x > flatDistTotal) continue;
                float rise = stage.y - last.y;                                              // num22
                float span = Mathf.Abs(rise) > 0.01f                                       // num23
                    ? Mathf.Abs(rise) / Mathf.Tan(Mathf.Clamp(stage.z, 1f, 89f) * (MathF.PI / 180f)) : 0f;
                if (span > 0.5f)
                {
                    float start = Mathf.Clamp(stage.w, last.x, stage.x);                    // num24
                    if (start > last.x + 0.5f)
                        nodes.Add(new Vector2(start, last.y));
                    if (start + span >= stage.x)
                    {
                        if (aaw)
                            nodes.Add(new Vector2(stage.x, stage.y));
                        else
                            nodes.Add(new Vector2(stage.x, last.y + rise * Mathf.Clamp01((stage.x - start) / span)));
                        continue;
                    }
                    nodes.Add(new Vector2(start + span, stage.y));
                }
                if (stage.x > nodes[nodes.Count - 1].x + 0.5f)
                    nodes.Add(new Vector2(stage.x, stage.y));
            }
            if (flatDistTotal > nodes[nodes.Count - 1].x + 0.5f)
                nodes.Add(new Vector2(flatDistTotal, Mathf.Max(targetAlt, 0f)));
            return nodes;
        }
    }
}
