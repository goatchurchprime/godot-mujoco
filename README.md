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
godot4 --headless --path example --script res://tests/comparison_smoke.gd
godot4 --headless --path example --script res://tests/demo_scene_smoke.gd
godot4 --headless --path example --script res://tests/mjcf_loader_smoke.gd
godot4 --headless --path example --script res://tests/mjcf_playground_smoke.gd
godot4 --headless --path example --script res://tests/mjcf_importer_smoke.gd
godot4 --path example
```

The example opens a small kickable “Scally” ragdoll. Press **B** to rebuild
the same mechanical definition under Godot/Jolt or MuJoCo, **Space** to apply
the same impulse-equivalent kick, and **R** to reset. It is a qualitative
integration instrument, not a performance benchmark: both backends use the
same authored dimensions, masses, joint limits, initial pose and fixed MuJoCo
step, while their constraint/contact solvers remain intentionally native.

The example also includes `mjcf_playground.tscn`, a first `simulate`-style
source-authoritative MJCF viewer. It loads the XML through MuJoCo, constructs
Godot visuals from the compiled bodies, geoms, meshes, materials and textures,
and keeps them synchronized while MuJoCo steps. Run it with:

```sh
godot4 --path example res://mjcf_playground.tscn
```

Use the right mouse button to orbit, middle mouse button to pan, the wheel to
zoom, and the left mouse button to grab a dynamic MuJoCo body with a damped
spring force. **Space** pauses, **.** advances one exact MuJoCo step group,
**R** resets, **K** perturbs the free body, and **[**/**]** changes speed.

With the editor plug-in enabled, files named `.mjcf` import as reimportable,
source-authoritative `PackedScene` resources. Instantiating one loads its source
through MuJoCo and runs it. The importer deliberately does not claim every
`.xml` file; XML-named MJCF remains supported through `MujocoScene3D.mjcf_path`.
Imported scenes are read-only in the same sense as other imported scenes and
can be used as the base of an inherited scene. This does not yet bake generated
visual children into the editor preview, and generated nodes are not editable
MuJoCo model components.

Linux/headless is the first supported target. The descriptor and build layout
reserve Windows and macOS artifacts, but neither is claimed tested. Mobile is
out of scope. See [docs/conventions.md](docs/conventions.md).

The recovered product scope and architectural rationale are recorded in
[docs/design-decisions.md](docs/design-decisions.md). This includes the MJCF vs
`mjSpec` authority model, active-ragdoll direction, `simulate` parity strategy,
Blender/Godot workflow boundary and test requirements.

The pinned Linux development environment currently resolves Godot 4.6.3 and
MuJoCo 3.14.0. Godot is built with a dynamic C++ runtime so the host and MuJoCo
share one `libstdc++`; this avoids C++ symbol interposition in MuJoCo's MJCF
parser. Use `$GODOT_BIN` inside `nix develop` to select that compatible host.

## Architectural boundary

This repository has no humanoid assumptions. XR Human owns extraction of its
generated articulated rig, MJCF generation, generalized-coordinate mapping,
golden pose comparison, target sites and reports. An engine-level drop-in
replacement for Jolt would require implementing Godot's physics-server
extension API and is deliberately not claimed here.

## AI assistance disclosure

This project has been developed with assistance from OpenAI Codex. AI-assisted
work includes research synthesis, API and test scaffolding, build-system work,
documentation, and portions of the C++ and GDScript implementation. The human
maintainer directs the project, reviews changes, decides what is accepted, and
remains responsible for published code and claims. Passing tests and recorded
measurements—not AI output on its own—are the basis for reporting a feature as
working.
