# Design decisions and product direction

This document records the decisions recovered from the initial research and
prototype discussions. It is intentionally broader than the API reference: its
purpose is to preserve *why* the integration is being built and to prevent a
later implementation from quietly narrowing the intended scope.

## Product purpose

`godot-mujoco` exists to make the full practical power of MuJoCo available
inside the open-source Godot game engine. The principal use cases are:

1. generating and recording physically plausible animation;
2. running articulated characters and mechanisms in real time;
3. active ragdolls which blend animation or tracked targets with physics; and
4. eventually, physically interacting networked VR avatars.

The plug-in is not intended to replace the established C, Python, `dm_control`,
MJX or native `simulate` workflows. Those interfaces are useful coverage
references: feature parity with the parts of MuJoCo that people actually use is
how this project avoids becoming an attractive but incomplete viewer.

Replicating the inspection and interaction facilities of `simulate` is
therefore a coverage strategy and a useful flagship application, not the final
product boundary.

## Repository boundary

The GDExtension remains generic. It must not contain assumptions about the XR
Human skeleton, humanoid proportions, trackers or a particular controller.

Human-specific model extraction, target construction, optimization and pose
inference belong in the XR Human repository. The generic bridge supplies the
safe MuJoCo model, state, simulation, inspection and editing facilities those
features require.

## Relationship to Godot physics

MuJoCo is initially an additional articulated simulation, not a claimed
drop-in replacement for Godot's `PhysicsServer3D` or Jolt.

- Jolt remains suitable for ordinary game-world rigid bodies and broad scene
  interaction.
- MuJoCo is especially suitable for linked mechanisms, robots, inverse
  dynamics and active ragdolls.
- A Godot scene may display and drive MuJoCo bodies while separately
  interacting with Jolt-owned scenery through an explicit contact or proxy
  boundary.
- Implementing a complete Godot physics-server extension would be a distinct,
  substantially larger project and must not be implied by this bridge.

The comparison demo deliberately builds the same small mechanism under Jolt
and MuJoCo so that differences can be observed rather than assumed.

## Native ownership and safety

`mjModel`, `mjData` and future `mjSpec` objects are owned by native,
reference-counted wrapper classes. Native pointers never cross into GDScript.

Godot-facing methods use checked arrays, dictionaries, transforms and named or
range-checked identifiers. Independent `mjData` instances may share one
immutable compiled model. Units and the meaning of every reported force must be
explicit.

## Coordinates and units

The public Godot API uses SI units: metres, seconds, kilograms, radians,
newtons and newton-metres.

MuJoCo's Z-up right-handed coordinates are converted to Godot's Y-up
right-handed coordinates with a proper rotation, never a reflection:

```text
MuJoCo (x, y, z) -> Godot (x, z, -y)
Godot  (x, y, z) -> MuJoCo (x, -z, y)
```

Orientations are converted as bases/matrices rather than by guessing
quaternion component order. Convention mismatches must fail tests loudly.

## MJCF import and model authority

MJCF is XML, but an MJCF model is not necessarily a self-contained XML file:
it can use includes and external meshes, textures, height fields and plug-ins.
MuJoCo itself remains responsible for resolving and validating those inputs.

Two modes are required and must not be confused.

### Source-authoritative import

An existing MJCF is loaded by MuJoCo and compiled to `mjModel`. Godot builds a
scene representation from the compiled bodies, geoms and assets, steps an
associated `mjData`, and synchronizes transforms back to the scene.

The source MJCF remains authoritative. Moving a generated Godot visual node
does not silently mutate the simulation. Unsupported visual assets use an
explicit placeholder; their collision and dynamics may still be fully present
inside MuJoCo.

The current work-in-progress implements this mode for body hierarchies,
primitives, compiled triangle meshes, basic material properties and 2D texture
pixels.

### Editable Godot model

Godot will eventually provide typed representations of MuJoCo bodies, joints,
geoms, sites, actuators, sensors, tendons, assets, keyframes and global
settings. These components form an editable source model.

The preferred live compilation path is:

```text
typed Godot MuJoCo components/resources
                    -> mjSpec
                    -> mj_compile()
                    -> mjModel + mjData
```

Serializing the hierarchy to MJCF is not required merely to compile it.
Canonical MJCF export remains important for interchange, version control,
debugging and use with the rest of the MuJoCo ecosystem, but is an optional
output rather than the mandatory runtime bridge.

Only fields represented by typed MuJoCo components can round-trip. Arbitrary
Godot meshes, scripts, Jolt constraints and physics materials cannot be
reliably inferred as MuJoCo semantics.

Unknown or not-yet-supported MJCF content must not be silently discarded by an
edit/export cycle. Until preservation is proven, such models remain
source-authoritative.

## Scene and simulation synchronization

Godot owns application time, input, presentation, networking and the rendered
scene. MuJoCo owns the articulated dynamic state.

The runtime loop advances `mjData` on a controlled fixed schedule and copies
MuJoCo body transforms into generated Godot nodes. Presentation smoothing may
interpolate rendered transforms, but it must not alter the authoritative
physics state.

Structural model edits initially recreate the compiled model. Named joint
positions, velocities, activations and controls should be preserved where
compatible. Later `mjSpec` support may permit more direct editing, but no code
should claim arbitrary live mutation before it is tested.

## Active ragdoll and animation model

MuJoCo is being adopted primarily for *active* ragdolls rather than only limp
linked bodies. Animation, XR observations or network state provide desired
poses and velocities. Actuators or a controller apply bounded physical effort
to follow those targets:

```text
animation / HMD / hands / trackers / network state
                         -> desired pose and velocity
                         -> actuator targets or joint torques
                         -> MuJoCo dynamics and contacts
                         -> physical body transforms
                         -> Godot Skeleton3D presentation
```

Tracking confidence should be able to vary by target. Environmental contact
must be allowed to move the result away from the desired animation. The root
body is not unconditionally teleported; hand, head or foot contact may impart
forces to the rest of the avatar.

The first pose-control milestone is not a learned controller. It is inspectable
joint actuation, inverse dynamics and target residuals with explicit units.
Optimization or learned policies come only after mapping and force semantics
are validated.

## Skeleton and visual presentation

Physics bodies are not the rendered character. A `SkeletonModifier3D` or
equivalent mapping copies validated physical body poses into a skinned
`Skeleton3D` while preserving bind-pose offsets and neutral rotations.

The acceptance criterion is geometric correspondence, not an attractive
animation. A deterministic asymmetric crouched pose must compare named joint
positions, axes, body transforms, centres of mass and offset probes between
Godot and MuJoCo. Sign, handedness or quaternion mistakes are test failures.

## Blender and offline authoring

Blender remains the preferred environment for mesh/armature construction,
weight painting, authored keyframes, graph editing, retargeting and polishing
or baking an offline result.

Godot plus MuJoCo remains authoritative for runtime physical animation: live XR
targets, networked players and unplanned contacts cannot be baked in advance.
The desired pipeline is bidirectional rather than exclusive:

```text
Blender-authored character and animation targets
                    -> Godot + MuJoCo active simulation
                    -> recorded Godot animation
                    -> optional Blender cleanup and final bake
```

Ragdoll Dynamics is a useful workflow reference for pose-following physics,
pinning, per-joint compliance, live manipulation and baking. Its public Blender
add-on is a bridge to a proprietary compiled `Ragdoll Core`; there is no public
evidence that it uses MuJoCo internally.

## `simulate`-parity coverage checklist

The Godot integration should eventually cover these categories even if their
UI differs from native `simulate`:

- load, validate, reload and inspect models and assets;
- pause, fixed-step, reset, speed control and keyframes;
- state (`qpos`, `qvel`, activation, control, mocap and userdata);
- actuators, sensors, tendons and constraints;
- contacts, applied forces and mouse-spring perturbation;
- joint axes, limits, inertia, COM and contact-force visualization;
- cameras, sites, visibility groups and geom filtering;
- solver warnings, timing and performance statistics;
- forward/inverse dynamics, Jacobians and mass-matrix utilities;
- plug-ins, flex objects, height fields, SDFs and supported asset types;
- multiple independent data instances and headless/batched operation; and
- model editing through `mjSpec` plus optional canonical MJCF export.

A versioned feature matrix should eventually record, for each MuJoCo facility,
whether it is exposed through the native API, importable, editable, exportable,
visualized and covered by a deterministic test.

## Platform and packaging policy

Linux desktop and headless tests come first. Build and artifact naming should
allow Windows and macOS to follow, but they are not supported until actually
built and tested. Mobile support is not promised.

The host, GDExtension, C++ runtime and MuJoCo library form one ABI system. The
pinned Nix development environment builds Godot with dynamic `libstdc++` so
Godot and MuJoCo share one coherent runtime. This is a tested requirement, not
merely packaging preference.

## Testing and claims

Important claims require deterministic evidence:

- minimal native API and independent-data tests;
- coordinate and quaternion round trips;
- multi-body MJCF compilation inside Godot;
- Jolt and MuJoCo scene-construction tests;
- model/asset import fixtures;
- golden-pose geometry comparison;
- static inverse dynamics under gravity;
- target-site residual reporting; and
- performance measurements that state model size, timestep and hardware.

No attractive demo substitutes for these tests. No unsupported platform,
asset type, optimization method or real-time pose solver is to be advertised as
complete.

## AI disclosure

Development uses OpenAI Codex assistance for research synthesis, scaffolding,
implementation, testing and documentation. The human maintainer directs the
project, reviews and accepts changes, and remains responsible for published
code and claims. Test results and inspectable measurements—not AI output by
itself—are the basis for stating that a feature works.
