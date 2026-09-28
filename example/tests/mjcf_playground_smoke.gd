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
    assert(scene.perturb_body("falling_shapes", Vector3(0.2, 0.0, 0.0)))
    assert(scene.simulation.get_qvel()[0] > 0.0)
    assert(scene.simulation.get_time() > 0.0)

    print("godot-mujoco MJCF playground smoke: PASS")
    quit(0)
