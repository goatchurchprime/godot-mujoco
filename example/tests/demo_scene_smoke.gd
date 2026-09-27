extends SceneTree

## Constructs both interactive-demo backends inside a real SceneTree. The
## numerical MuJoCo assertions live in comparison_smoke.gd; this catches scene
## wiring and Jolt joint-construction regressions.


func _init() -> void:
    call_deferred("_run")


func _run() -> void:
    var packed := load("res://comparison_demo.tscn") as PackedScene
    assert(packed != null)
    var demo = packed.instantiate()
    root.add_child(demo)
    await process_frame

    assert(demo.simulation != null, "MuJoCo backend did not initialize")
    assert(demo.link_nodes.size() == demo.LINKS.size())
    await physics_frame

    demo._select_backend(demo.Backend.JOLT)
    await process_frame
    assert(demo.jolt_bodies.size() == demo.LINKS.size(),
        "Jolt backend did not create every body")
    assert(demo.link_nodes.size() == demo.LINKS.size())
    for link in demo.LINKS:
        if link.parent.is_empty():
            continue
        var joint := demo.world_root.get_node("%s_joint" % link.name) \
            as Generic6DOFJoint3D
        assert(not joint.node_a.is_empty() and not joint.node_b.is_empty(),
            "%s Jolt joint has an empty body path" % link.name)
    for unused in range(4):
        await physics_frame
    for link in demo.LINKS:
        var body := demo.jolt_bodies[link.name] as RigidBody3D
        assert(body.global_transform.origin.is_finite(),
            "%s Jolt position became non-finite" % link.name)

    print("godot-mujoco demo scene smoke: PASS")
    quit(0)
