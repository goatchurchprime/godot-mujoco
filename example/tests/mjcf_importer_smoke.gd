extends SceneTree


func _init() -> void:
    call_deferred("_run")


func _run() -> void:
    var packed := load("res://models/imported_showcase.mjcf") as PackedScene
    if packed == null:
        push_error(".mjcf was not imported as a PackedScene")
        quit(1)
        return
    var scene := packed.instantiate() as MujocoScene3D
    assert(scene != null)
    root.add_child(scene)
    await process_frame
    assert(scene.simulation != null, scene.last_error)
    assert(scene.body_nodes.size() == 2)
    assert(scene.geom_nodes.size() == 2)
    print("godot-mujoco MJCF editor importer smoke: PASS")
    quit(0)
