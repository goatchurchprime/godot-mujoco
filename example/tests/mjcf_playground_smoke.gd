extends SceneTree


func _init() -> void:
    call_deferred("_run")


func _run() -> void:
    var packed := load("res://mjcf_playground.tscn") as PackedScene
    assert(packed != null)
    var demo = packed.instantiate()
    root.add_child(demo)
    await process_frame

    var scene: MujocoScene3D = demo.mujoco_scene
    assert(scene.simulation != null, scene.last_error)
    assert(is_equal_approx(scene.model.get_timestep(), 1.0 / 120.0))
    scene.set_paused(true)
    var start_time := scene.simulation.get_time()
    assert(scene.single_step())
    assert(is_equal_approx(scene.simulation.get_time() - start_time,
        scene.model.get_timestep() * scene.substeps))
    assert(scene.reset_simulation())
    assert(is_zero_approx(scene.simulation.get_time()))
    var hit: Dictionary = scene.simulation.raycast(
        Vector3(0.0, 1.5, -3.0), Vector3.BACK, false)
    assert(not hit.is_empty(), "MuJoCo ray did not hit the dynamic model")
    assert(hit.body_id == 1)
    scene.simulation.clear_applied_forces()
    assert(scene.simulation.apply_body_force_at_point(hit.body_id,
        Vector3(24.0, 0.0, 0.0), hit.position_m))
    assert(scene.simulation.step())
    assert(scene.simulation.get_qvel()[0] > 0.0,
        "force at selected point did not accelerate the free body")
    assert(scene.reset_simulation())
    assert(scene.perturb_body("falling_shapes", Vector3(0.2, 0.0, 0.0)))
    assert(scene.simulation.get_qvel()[0] > 0.0)
    assert(scene.simulation.get_time() > 0.0)

    print("godot-mujoco MJCF playground smoke: PASS")
    quit(0)
