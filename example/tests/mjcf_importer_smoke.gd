extends SceneTree


func _init() -> void:
    call_deferred("_run")


func _run() -> void:
    var packed := load("res://models/imported_showcase.mjcf") as PackedScene
    if packed == null:
        push_error(".mjcf was not imported as a PackedScene")
        quit(1)
        return
    # The packed import itself contains an editor-visible generated hierarchy.
    var preview := packed.instantiate() as MujocoScene3D
    assert(preview.get_node_or_null("GeneratedMJCF") != null)
    assert(preview.get_node("GeneratedMJCF").get_child_count() > 0)
    preview.free()

    var scene := packed.instantiate() as MujocoScene3D
    assert(scene != null)
    root.add_child(scene)
    await process_frame
    assert(scene.simulation != null, scene.last_error)
    assert(scene.body_nodes.size() == 2)
    assert(scene.joint_nodes.size() == 1)
    assert(scene.body_nodes[1] is MujocoBody3D)
    assert(scene.joint_nodes[0] is MujocoJoint3D)
    assert(scene.geom_nodes.size() == 2)
    assert(scene.get_node("GeneratedMJCF").get_child_count() > 0)
    print("godot-mujoco MJCF editor importer smoke: PASS")
    quit(0)
