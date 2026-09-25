# godot-mujoco

Research-grade, generic Godot 4 GDExtension bindings for MuJoCo's C API. This
is an early mapping and inverse-dynamics bridge, not a finished pose solver and
not a replacement for Godot's active PhysicsServer.

The bridge owns `mjModel` and `mjData` internally. GDScript receives ordinary
Godot values and reference-counted objects, never native pointers. Each call to
`MujocoModel.create_data()` allocates independent simulation state while
retaining the compiled model for its lifetime.

Implemented API: MJCF path/string compilation; checked `qpos`, `qvel`, `qacc`;
forward and inverse dynamics; named body/site transforms; joint anchors/axes;
body and subtree COM; contacts and contact wrenches; constraint forces; and
labelled generalized-force components.

## Linux development

```sh
git submodule update --init --recursive
nix develop
scons platform=linux target=template_debug
cp -a addons/mujoco example/addons/
godot4 --headless --path example --script res://tests/api_smoke.gd
```

Linux/headless is the first supported target. The descriptor and build layout
reserve Windows and macOS artifacts, but neither is claimed tested. Mobile is
out of scope. See [docs/conventions.md](docs/conventions.md).

## Architectural boundary

This repository has no humanoid assumptions. XR Human owns extraction of its
generated articulated rig, MJCF generation, generalized-coordinate mapping,
golden pose comparison, target sites and reports. An engine-level drop-in
replacement for Jolt would require implementing Godot's physics-server
extension API and is deliberately not claimed here.
