# Sea Power Flight Model

A step-by-step flight-time model for Sea Power missiles. For a shooter, a weapon and a target it predicts how long the
round will fly, by flying it in simulation with the game's own thrust and drag functions and the guidance state machine
the game's missile mover runs. It holds no per-missile constants: every input is a game constant, a field from the
ammunition's `.ini`, or a value a game method returns.

This repository is source only. It ships inside the mods that compile it:
[Auto Time-on-Target](https://github.com/DatIsKlar/Sea-Power-Auto-Time-on-Target) and Flight Time Display. Players
install those mods, never this repository.

How the model works, with its accuracy: [`docs/model/`](docs/model/00-index.md).

## Layout

| path | contents |
|---|---|
| `Simulation/` | the model: entry points, integrator, per-step solve, lead intercept, waypoints, async workers |
| `Support/` | game units, math helpers, the game clock, a TTL cache, logging shapes |
| `Diagnostics/TelemetryCadence.cs` | sampling cadence shared by the model's traces and the host's flight recorder |
| `tools/lab/` | offline replay lab and the corpus of recorded flights it scores against |
| `docs/model/` | model documentation |

## Using it in a mod

A mod adds the sources to its own build, so each mod's DLL carries its own copy and needs nothing else at runtime:

```xml
<ItemGroup>
  <Compile Remove="FlightModel/**" />
  <Compile Include="$(FlightModelDir)/Simulation/*.cs" />
  <Compile Include="$(FlightModelDir)/Support/*.cs" />
  <Compile Include="$(FlightModelDir)/Diagnostics/TelemetryCadence.cs" />
</ItemGroup>
```

The sources are in namespace `AutoTOT` and target netstandard2.1. They reference `Seapower-Scripts`, `UniRx`,
`BepInEx`, `UnityEngine.CoreModule` and `UnityEngine.PhysicsModule`.

The host supplies four members the model reads outside itself. None of them takes part in a computed flight time:

| member | what the model uses it for |
|---|---|
| `Bootstrap.Log` | a BepInEx `ManualLogSource` for warnings |
| `Coordinator.VerboseLog`, `Coordinator.TraceFlightModel` | diagnostic switches |
| `CoordinatorProfiler.MsPerTick` | converting `Stopwatch` ticks to milliseconds |
| `SolveInputDump.Write(...)` | writing a solver input for the lab; a host that does not record returns `null` |

`tools/lab/Stubs.cs` is a complete minimal set.

The host also calls, on the main thread:

- every frame of a mission: `FlightTime.ObservePhysicsStep(GameClock.SimNow())`, which feeds the physics-step
  prediction used to round a surface target's velocity, and `FlightTime.DrainCompleted()`, which publishes finished
  worker solves;
- on leaving a mission: `FlightTime.ClearCache()` and `FlightTime.ResetPhysicsStep()`;
- once at startup, to solve off the main thread: `FlightTime.StartWorkers(count)`.

The integrator binds to the game's stable and beta simulator. On the public branch it does not bind, and estimates fall
back to the game's own `MaxRangePrecise` (`FlightTime.SimIsBeta` reports which).

## Building the lab

Create `FlightModel.local.props` at the repository root (not tracked):

```xml
<Project>
  <PropertyGroup>
    <GameDir>/path/to/Sea Power</GameDir>
  </PropertyGroup>
</Project>
```

Then `dotnet build tools/lab/Lab.csproj -c Release` and see [`tools/lab/README.md`](tools/lab/README.md).
