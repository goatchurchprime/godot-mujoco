extends SceneTree


func _init() -> void:
    call_deferred("_run")


func _run() -> void:
    var loader_script := load("res://addons/mujoco/mujoco_scene_3d.gd")
    assert(loader_script != null)
    var scene = loader_script.new()
    scene.auto_load = false
    scene.simulate = false
    root.add_child(scene)

    assert(scene.load_mjcf("res://models/primitive_showcase.xml"), scene.last_error)
    assert(scene.body_nodes.size() == 2)
    assert(scene.geom_nodes.size() == 4)
    assert(scene.model.get_geoms()[1].type == "box")
    assert(scene.geom_nodes[1].mesh is BoxMesh)
    assert(scene.geom_nodes[2].mesh is SphereMesh)
    assert(scene.geom_nodes[3].mesh is ArrayMesh)
    assert(scene.model.get_meshes().size() == 1)
    assert(scene.model.get_meshes()[0].vertices.size() == 12)
    assert(scene.model.get_textures().size() == 1)
    assert(scene.model.get_textures()[0].data.size() > 0)
    assert(scene.geom_nodes[3].material_override.albedo_texture != null)

    var before: Transform3D = scene.body_nodes[1].global_transform
    assert(scene.simulation.step(30))
    scene._sync_body_transforms()
    var after: Transform3D = scene.body_nodes[1].global_transform
    assert(after.origin.y < before.origin.y, "loaded body did not fall under gravity")

    print("godot-mujoco MJCF loader smoke: PASS")
    quit(0)
