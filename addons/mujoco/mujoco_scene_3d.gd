@tool
class_name MujocoScene3D
extends Node3D

const MujocoBodyNode = preload("res://addons/mujoco/mujoco_body_3d.gd")
const MujocoJointNode = preload("res://addons/mujoco/mujoco_joint_3d.gd")

## Runtime MJCF loader and visualizer. The source MJCF remains authoritative;
## this node does not yet serialize edits to its generated children back to XML.

@export_file("*.xml", "*.mjcf", "*.mjz") var mjcf_path := ""
@export var auto_load := true
@export var simulate := true
@export_range(1, 100, 1) var substeps := 1
@export_range(0.01, 8.0, 0.01) var time_scale := 1.0

var model: MujocoModel
var simulation: MujocoData
var body_nodes: Dictionary = {}
var joint_nodes: Dictionary = {}
var geom_nodes: Dictionary = {}
var last_error := ""

var _accumulator := 0.0
var _step_seconds := 1.0 / 60.0
var _generated_root: Node3D
var _meshes: Array = []
var _textures: Array = []
var _materials: Array = []


func _ready() -> void:
    if Engine.is_editor_hint():
        return
    if auto_load and not mjcf_path.is_empty():
        load_mjcf(mjcf_path)


func _physics_process(delta: float) -> void:
    if Engine.is_editor_hint():
        return
    if not simulate or simulation == null:
        return
    _accumulator += delta * time_scale
    while _accumulator >= _step_seconds:
        if not simulation.step(substeps):
            last_error = simulation.get_last_error()
            set_physics_process(false)
            return
        _accumulator -= _step_seconds
    _sync_body_transforms()


func load_mjcf(path: String) -> bool:
    clear_model()
    var filesystem_path := path
    if path.begins_with("res://") or path.begins_with("user://"):
        filesystem_path = ProjectSettings.globalize_path(path)

    model = MujocoModel.new()
    if not model.load_mjcf(filesystem_path):
        last_error = model.get_last_error()
        model = null
        return false
    simulation = model.create_data()
    if simulation == null or not simulation.forward():
        last_error = "MuJoCo could not allocate or initialize simulation data"
        model = null
        simulation = null
        return false

    _generated_root = Node3D.new()
    _generated_root.name = "GeneratedMJCF"
    add_child(_generated_root)
    _meshes = model.get_meshes()
    _textures = model.get_textures()
    _materials = model.get_materials()
    _build_bodies(model.get_bodies())
    _build_joints(model.get_joints())
    _build_geoms(model.get_geoms())
    _step_seconds = model.get_timestep() * substeps
    _accumulator = 0.0
    last_error = ""
    _sync_body_transforms()
    return true


func set_paused(paused: bool) -> void:
    simulate = not paused


func is_paused() -> bool:
    return not simulate


func single_step() -> bool:
    if simulation == null:
        last_error = "No MuJoCo simulation is loaded"
        return false
    if not simulation.step(substeps):
        last_error = simulation.get_last_error()
        return false
    _accumulator = 0.0
    _sync_body_transforms()
    last_error = ""
    return true


func reset_simulation() -> bool:
    if simulation == null:
        last_error = "No MuJoCo simulation is loaded"
        return false
    simulation.reset()
    if not simulation.forward():
        last_error = simulation.get_last_error()
        return false
    _accumulator = 0.0
    _sync_body_transforms()
    last_error = ""
    return true


func perturb_body(body_name: StringName, impulse_ns: Vector3,
        angular_impulse_nms := Vector3.ZERO) -> bool:
    if simulation == null:
        last_error = "No MuJoCo simulation is loaded"
        return false
    # MuJoCo exposes applied forces rather than an impulse call. Applying
    # impulse / dt for exactly one model step gives an inspectable impulse.
    var dt := model.get_timestep()
    if dt <= 0.0:
        last_error = "The MuJoCo model timestep is invalid"
        return false
    simulation.clear_applied_forces()
    if not simulation.apply_body_wrench(body_name, impulse_ns / dt,
            angular_impulse_nms / dt):
        last_error = simulation.get_last_error()
        return false
    var stepped := simulation.step()
    simulation.clear_applied_forces()
    if not stepped:
        last_error = simulation.get_last_error()
        return false
    _accumulator = 0.0
    _sync_body_transforms()
    last_error = ""
    return true


func clear_model() -> void:
    model = null
    simulation = null
    body_nodes.clear()
    joint_nodes.clear()
    geom_nodes.clear()
    _meshes.clear()
    _textures.clear()
    _materials.clear()
    if _generated_root != null:
        # Imported scenes contain a packed editor preview with this name. It
        # must be removed synchronously before rebuilding the live hierarchy,
        # otherwise one frame contains duplicate bodies and node names.
        if _generated_root.get_parent() == self:
            remove_child(_generated_root)
        _generated_root.free()
        _generated_root = null
    else:
        var imported_preview := get_node_or_null("GeneratedMJCF")
        if imported_preview != null:
            remove_child(imported_preview)
            imported_preview.free()


func _build_bodies(bodies: Array) -> void:
    for body in bodies:
        var node := MujocoBodyNode.new()
        node.name = _safe_name(body.name, "body", body.id)
        node.mujoco_id = body.id
        node.source_name = body.name
        node.mass_kg = body.mass_kg
        node.dynamic = body.id > 0
        node.set_meta("mujoco_body_id", body.id)
        node.set_meta("mujoco_name", body.name)
        body_nodes[body.id] = node
    for body in bodies:
        var node: Node3D = body_nodes[body.id]
        var parent_id: int = body.parent_id
        if body.id != 0 and body_nodes.has(parent_id):
            body_nodes[parent_id].add_child(node)
        else:
            _generated_root.add_child(node)
        node.transform = body.local_transform


func _build_joints(joints: Array) -> void:
    for joint in joints:
        var node := MujocoJointNode.new()
        node.name = _safe_name(joint.name, "joint", joint.id)
        node.position = joint.position_m
        node.mujoco_id = joint.id
        node.source_name = joint.name
        node.joint_type = joint.type
        node.axis_unit = joint.axis_unit
        node.limited = joint.limited
        node.range_rad_or_m = joint.range_rad_or_m
        node.damping = joint.damping
        node.set_meta("mujoco_joint_id", joint.id)
        body_nodes[joint.body_id].add_child(node)
        joint_nodes[joint.id] = node


func _build_geoms(geoms: Array) -> void:
    for geom in geoms:
        var instance := MeshInstance3D.new()
        instance.name = _safe_name(geom.name, "geom", geom.id)
        instance.set_meta("mujoco_geom_id", geom.id)
        instance.set_meta("mujoco_name", geom.name)
        instance.set_meta("mujoco_type", geom.type)
        instance.mesh = _make_mesh(geom)
        instance.transform = geom.local_transform
        if geom.type == "ellipsoid":
            var ellipsoid_size: Vector3 = geom.size_mj
            # MuJoCo local Z maps to Godot local Y.
            instance.scale = Vector3(
                ellipsoid_size.x * 2.0,
                ellipsoid_size.z * 2.0,
                ellipsoid_size.y * 2.0)
        instance.material_override = _make_material(geom)
        body_nodes[geom.body_id].add_child(instance)
        geom_nodes[geom.id] = instance


func _make_mesh(geom: Dictionary) -> Mesh:
    var size: Vector3 = geom.size_mj
    match geom.type:
        "plane":
            var mesh := PlaneMesh.new()
            mesh.size = Vector2(maxf(size.x * 2.0, 0.1), maxf(size.y * 2.0, 0.1))
            return mesh
        "sphere":
            var mesh := SphereMesh.new()
            mesh.radius = size.x
            mesh.height = size.x * 2.0
            return mesh
        "ellipsoid":
            var mesh := SphereMesh.new()
            mesh.radius = 0.5
            mesh.height = 1.0
            return mesh
        "capsule":
            var mesh := CapsuleMesh.new()
            mesh.radius = size.x
            mesh.height = 2.0 * (size.y + size.x)
            return mesh
        "cylinder":
            var mesh := CylinderMesh.new()
            mesh.top_radius = size.x
            mesh.bottom_radius = size.x
            mesh.height = size.y * 2.0
            return mesh
        "box":
            var mesh := BoxMesh.new()
            mesh.size = Vector3(size.x * 2.0, size.z * 2.0, size.y * 2.0)
            return mesh
        "mesh":
            if geom.data_id >= 0 and geom.data_id < _meshes.size():
                var source: Dictionary = _meshes[geom.data_id]
                if not source.vertices.is_empty():
                    return _make_array_mesh(source)
            return _make_placeholder_mesh()
        _:
            # Hfield and SDF rendering need additional compiled asset buffers.
            return _make_placeholder_mesh()


func _make_array_mesh(source: Dictionary) -> ArrayMesh:
    var arrays := []
    arrays.resize(Mesh.ARRAY_MAX)
    arrays[Mesh.ARRAY_VERTEX] = source.vertices
    if not source.normals.is_empty():
        arrays[Mesh.ARRAY_NORMAL] = source.normals
    if not source.texcoords.is_empty():
        arrays[Mesh.ARRAY_TEX_UV] = source.texcoords
    var mesh := ArrayMesh.new()
    mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
    return mesh


func _make_placeholder_mesh() -> BoxMesh:
    var mesh := BoxMesh.new()
    mesh.size = Vector3(0.08, 0.08, 0.08)
    return mesh


func _make_material(geom: Dictionary) -> StandardMaterial3D:
    var material := StandardMaterial3D.new()
    material.albedo_color = geom.color
    material.roughness = 0.72
    var material_id: int = geom.material_id
    if material_id >= 0 and material_id < _materials.size():
        var source: Dictionary = _materials[material_id]
        material.albedo_color = source.color
        material.roughness = source.roughness
        material.metallic = source.metallic
        material.uv1_scale = Vector3(source.texture_repeat.x,
            source.texture_repeat.y, 1.0)
        var texture_id: int = source.color_texture_id
        if texture_id >= 0 and texture_id < _textures.size():
            material.albedo_texture = _make_texture(_textures[texture_id])
    material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA \
        if material.albedo_color.a < 1.0 else BaseMaterial3D.TRANSPARENCY_DISABLED
    return material


func _make_texture(source: Dictionary) -> ImageTexture:
    var format := Image.FORMAT_RGBA8
    match source.channels:
        1: format = Image.FORMAT_L8
        2: format = Image.FORMAT_LA8
        3: format = Image.FORMAT_RGB8
        4: format = Image.FORMAT_RGBA8
    var image := Image.create_from_data(source.width, source.height, false,
        format, source.data)
    return ImageTexture.create_from_image(image)


func _sync_body_transforms() -> void:
    if simulation == null:
        return
    for id in body_nodes:
        body_nodes[id].global_transform = simulation.get_body_transform_by_id(id)


func _safe_name(source: String, fallback: String, id: int) -> String:
    return source if not source.is_empty() else "%s_%d" % [fallback, id]
