extends SceneTree

## Exercises the exact multi-body model used by the interactive comparison.
## This deliberately stays renderer-independent so it can run in CI.

const STEP_COUNT := 480


func _init() -> void:
    var demo_script := load("res://comparison_demo.gd")
    assert(demo_script != null)
    var demo = demo_script.new()

    var model := MujocoModel.new()
    assert(model.compile_mjcf(demo._make_mjcf()), model.get_last_error())
    var sizes := model.get_sizes()
    assert(sizes.nbody == 7, "expected world plus six ragdoll bodies")
    assert(sizes.njnt == 6, "expected one free joint and five hinges")

    var simulation := model.create_data()
    assert(simulation != null)
    assert(simulation.forward())
    assert(simulation.apply_body_wrench(
        "torso", Vector3(55, 8, 0), Vector3(0, 0, -5)))
    assert(simulation.step())
    simulation.clear_applied_forces()
    assert(simulation.step(STEP_COUNT - 1))

    assert(is_equal_approx(simulation.get_time(), STEP_COUNT / 120.0))
    for link in demo.LINKS:
        var pose: Transform3D = simulation.get_body_transform(link.name)
        assert(pose.origin.is_finite(), "%s position became non-finite" % link.name)
        assert(pose.basis.is_finite(), "%s basis became non-finite" % link.name)
    assert(not simulation.get_contacts().is_empty(), "ragdoll never contacted the floor")

    print("godot-mujoco comparison smoke: PASS")
    demo.free()
    quit(0)
