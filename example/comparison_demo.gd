extends Node3D

## A deliberately small backend comparison, not a benchmark or pose solver.
## Both worlds are generated from this one mechanical definition.

enum Backend { JOLT, MUJOCO }

const FIXED_STEP := 1.0 / 120.0
const LINKS := [
    {"name": "torso", "parent": "", "at": Vector3(0, 1.35, 0),
        "shape": "box", "size": Vector3(.25, .34, .14), "mass": 7.0,
        "color": Color("57a6d9")},
    {"name": "head", "parent": "torso", "at": Vector3(0, .43, 0),
        "shape": "sphere", "size": Vector3(.19, .19, .19), "mass": 2.0,
        "axis": Vector3.RIGHT, "range": Vector2(-35, 35),
        "color": Color("f1c27d")},
    {"name": "left_arm", "parent": "torso", "at": Vector3(-.31, .22, 0),
        "shape": "capsule_x", "size": Vector3(.42, .075, .075), "mass": 1.2,
        "axis": Vector3.FORWARD, "range": Vector2(-120, 80),
        "color": Color("e66565")},
    {"name": "right_arm", "parent": "torso", "at": Vector3(.31, .22, 0),
        "shape": "capsule_x", "size": Vector3(.42, .075, .075), "mass": 1.2,
        "axis": Vector3.FORWARD, "range": Vector2(-80, 120),
        "color": Color("e66565")},
    {"name": "left_leg", "parent": "torso", "at": Vector3(-.14, -.37, 0),
        "shape": "capsule_y", "size": Vector3(.09, .58, .09), "mass": 2.4,
        "axis": Vector3.FORWARD, "range": Vector2(-80, 55),
        "color": Color("75c782")},
    {"name": "right_leg", "parent": "torso", "at": Vector3(.14, -.37, 0),
        "shape": "capsule_y", "size": Vector3(.09, .58, .09), "mass": 2.4,
        "axis": Vector3.FORWARD, "range": Vector2(-55, 80),
        "color": Color("75c782")},
]

var backend := Backend.MUJOCO
var simulation: MujocoData
var model: MujocoModel
var accumulator := 0.0
var link_nodes := {}
var jolt_bodies := {}
var world_root: Node3D
var status: Label


func _ready() -> void:
    _build_room()
    _select_backend(backend)


func _physics_process(delta: float) -> void:
    if backend == Backend.MUJOCO and simulation != null:
        accumulator += delta
        while accumulator >= FIXED_STEP:
            simulation.step()
            accumulator -= FIXED_STEP
        for link in LINKS:
            link_nodes[link.name].transform = simulation.get_body_transform(link.name)
    _update_status()


func _unhandled_input(event: InputEvent) -> void:
    if not event is InputEventKey or not event.pressed or event.echo:
        return
    if event.keycode == KEY_B:
        _select_backend(Backend.JOLT if backend == Backend.MUJOCO else Backend.MUJOCO)
    elif event.keycode == KEY_R:
        _select_backend(backend)
    elif event.keycode == KEY_SPACE:
        _kick()


func _select_backend(next: Backend) -> void:
    backend = next
    accumulator = 0.0
    if world_root != null:
        world_root.queue_free()
    world_root = Node3D.new()
    world_root.name = "Simulation"
    add_child(world_root)
    link_nodes.clear()
    jolt_bodies.clear()
    simulation = null
    model = null
    if backend == Backend.MUJOCO:
        _build_mujoco()
    else:
        _build_jolt()


func _build_mujoco() -> void:
    model = MujocoModel.new()
    var xml := _make_mjcf()
    if not model.compile_mjcf(xml):
        push_error("MuJoCo compile failed: %s\n%s" % [model.get_last_error(), xml])
        return
    simulation = model.create_data()
    simulation.forward()
    for link in LINKS:
        var holder := Node3D.new()
        holder.name = link.name
        holder.add_child(_make_mesh(link))
        world_root.add_child(holder)
        link_nodes[link.name] = holder


func _build_jolt() -> void:
    var global_frames := {}
    for link in LINKS:
        var frame := Transform3D(Basis.IDENTITY, link.at)
        if not link.parent.is_empty():
            frame = global_frames[link.parent] * frame
        global_frames[link.name] = frame
        var body := RigidBody3D.new()
        body.name = link.name
        body.mass = link.mass
        body.transform = frame
        var collision := CollisionShape3D.new()
        collision.shape = _make_shape(link)
        collision.transform = _shape_offset(link)
        body.add_child(collision)
        body.add_child(_make_mesh(link))
        world_root.add_child(body)
        jolt_bodies[link.name] = body
        link_nodes[link.name] = body
    for link in LINKS:
        if link.parent.is_empty():
            continue
        var joint := Generic6DOFJoint3D.new()
        joint.name = "%s_joint" % link.name
        joint.transform = global_frames[link.name]
        world_root.add_child(joint)
        joint.node_a = joint.get_path_to(jolt_bodies[link.parent])
        joint.node_b = joint.get_path_to(jolt_bodies[link.name])
        joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
        joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
        joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
        # The comparison creature is planar: all authored hinges use local Z.
        joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
        joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT,
            deg_to_rad(link.range.x))
        joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT,
            deg_to_rad(link.range.y))


func _kick() -> void:
    var force := Vector3(55, 8, 0)
    if backend == Backend.MUJOCO and simulation != null:
        simulation.clear_applied_forces()
        simulation.apply_body_wrench("torso", force, Vector3(0, 0, -5))
        # A one-step impulse approximation: F = impulse / dt.
        simulation.step()
        simulation.clear_applied_forces()
    elif jolt_bodies.has("torso"):
        jolt_bodies.torso.apply_central_impulse(force * FIXED_STEP)
        jolt_bodies.torso.apply_torque_impulse(Vector3(0, 0, -5) * FIXED_STEP)


func _make_mjcf() -> String:
    var children := {}
    for link in LINKS:
        children.get_or_add(link.parent, []).append(link)
    var lines := PackedStringArray([
        '<mujoco model="scally-backend-comparison">',
        '  <compiler angle="degree" autolimits="true"/>',
        '  <option timestep="%.12f" gravity="0 0 -9.81" integrator="implicitfast"/>' % FIXED_STEP,
        '  <default><joint damping="0.35"/><geom friction="0.9 0.03 0.003" condim="3"/></default>',
        '  <worldbody>',
        '    <geom name="floor" type="plane" size="4 4 .1"/>',
    ])
    _append_mj_body(lines, children, children[" "] if children.has(" ") else children[""], 2)
    lines.append('  </worldbody>')
    lines.append('</mujoco>')
    return "\n".join(lines)


func _append_mj_body(lines: PackedStringArray, children: Dictionary,
        links: Array, depth: int) -> void:
    for link in links:
        var pad := "  ".repeat(depth)
        lines.append('%s<body name="%s" pos="%s">' % [pad, link.name,
            _mj_vec(link.at)])
        if link.parent.is_empty():
            lines.append('%s  <freejoint name="root"/>' % pad)
        else:
            lines.append('%s  <joint name="%s_joint" type="hinge" axis="0 1 0" range="%.9f %.9f"/>' \
                % [pad, link.name, link.range.x, link.range.y])
        lines.append("%s  %s" % [pad, _mj_geom(link)])
        if children.has(link.name):
            _append_mj_body(lines, children, children[link.name], depth + 1)
        lines.append("%s</body>" % pad)


func _mj_geom(link: Dictionary) -> String:
    if link.shape == "box":
        return '<geom name="%s_geom" type="box" size="%s" mass="%.9f"/>' \
            % [link.name, _mj_size(link.size), link.mass]
    if link.shape == "sphere":
        return '<geom name="%s_geom" type="sphere" size="%.9f" mass="%.9f"/>' \
            % [link.name, link.size.x, link.mass]
    var endpoint := Vector3(link.size.x, 0, 0) if link.shape == "capsule_x" \
        else Vector3(0, -link.size.y, 0)
    return '<geom name="%s_geom" type="capsule" fromto="0 0 0 %s" size="%.9f" mass="%.9f"/>' \
        % [link.name, _mj_vec(endpoint), link.size.z, link.mass]


func _mj_vec(godot: Vector3) -> String:
    return "%.12f %.12f %.12f" % [godot.x, -godot.z, godot.y]


func _mj_size(godot: Vector3) -> String:
    return "%.12f %.12f %.12f" % [absf(godot.x), absf(godot.z), absf(godot.y)]


func _make_shape(link: Dictionary) -> Shape3D:
    if link.shape == "box":
        var box := BoxShape3D.new()
        box.size = link.size * 2.0
        return box
    if link.shape == "sphere":
        var sphere := SphereShape3D.new()
        sphere.radius = link.size.x
        return sphere
    var capsule := CapsuleShape3D.new()
    capsule.radius = link.size.z
    capsule.height = (link.size.x if link.shape == "capsule_x" else link.size.y) \
        + 2.0 * capsule.radius
    return capsule


func _shape_offset(link: Dictionary) -> Transform3D:
    if link.shape == "capsule_x":
        return Transform3D(Basis.from_euler(Vector3(0, 0, PI * .5)),
            Vector3(link.size.x * .5, 0, 0))
    if link.shape == "capsule_y":
        return Transform3D(Basis.IDENTITY, Vector3(0, -link.size.y * .5, 0))
    return Transform3D.IDENTITY


func _make_mesh(link: Dictionary) -> MeshInstance3D:
    var instance := MeshInstance3D.new()
    var material := StandardMaterial3D.new()
    material.albedo_color = link.color
    material.roughness = .72
    instance.material_override = material
    if link.shape == "box":
        var mesh := BoxMesh.new(); mesh.size = link.size * 2.0; instance.mesh = mesh
    elif link.shape == "sphere":
        var mesh := SphereMesh.new(); mesh.radius = link.size.x; mesh.height = link.size.x * 2.0; instance.mesh = mesh
    else:
        var mesh := CapsuleMesh.new()
        mesh.radius = link.size.z
        mesh.height = (link.size.x if link.shape == "capsule_x" else link.size.y) + 2.0 * mesh.radius
        instance.mesh = mesh
    instance.transform = _shape_offset(link)
    return instance


func _build_room() -> void:
    var floor := StaticBody3D.new()
    floor.position.y = -.05
    var collision := CollisionShape3D.new(); var shape := BoxShape3D.new()
    shape.size = Vector3(8, .1, 8); collision.shape = shape; floor.add_child(collision)
    var visual := MeshInstance3D.new(); var mesh := BoxMesh.new(); mesh.size = shape.size
    visual.mesh = mesh; floor.add_child(visual); add_child(floor)
    var camera := Camera3D.new(); camera.position = Vector3(2.5, 1.5, -4.2)
    add_child(camera); camera.look_at(Vector3(0, .9, 0))
    var light := DirectionalLight3D.new(); light.rotation_degrees = Vector3(-50, -25, 0)
    light.shadow_enabled = true; add_child(light)
    var canvas := CanvasLayer.new(); status = Label.new()
    status.position = Vector2(18, 18); status.add_theme_font_size_override("font_size", 18)
    canvas.add_child(status); add_child(canvas)


func _update_status() -> void:
    var name := "MuJoCo" if backend == Backend.MUJOCO else "Godot/Jolt"
    var torso := link_nodes.get("torso") as Node3D
    status.text = ("Scally ragdoll — %s\n" % name
        + "B: switch backend   Space: kick   R: reset\n"
        + "torso height: %.3f m" % (torso.position.y if torso else 0.0))
