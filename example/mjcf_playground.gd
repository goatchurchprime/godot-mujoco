extends Node3D

## A deliberately small `simulate`-style playground for source-authoritative
## MJCF files. MuJoCo owns dynamics; generated Godot nodes are presentation.

@onready var mujoco_scene: MujocoScene3D = $MujocoScene3D
@onready var status: Label = $Interface/Status
@onready var camera: Camera3D = $Camera3D

var camera_focus := Vector3(0.0, 0.8, 0.0)
var camera_distance := 4.6
var camera_yaw := 0.55
var camera_pitch := -0.22
var orbiting := false
var panning := false
var grabbed_body_id := -1
var grabbed_body_name := ""
var grabbed_local_point := Vector3.ZERO
var grab_target := Vector3.ZERO
var grab_depth := 1.0
var grab_marker: MeshInstance3D

const GRAB_STIFFNESS_N_PER_M := 180.0
const GRAB_DAMPING_NS_PER_M := 18.0
const GRAB_MAX_FORCE_N := 350.0


func _ready() -> void:
	_update_camera()
	_make_grab_marker()
	if mujoco_scene.simulation == null:
		status.text = "MJCF load failed:\n%s" % mujoco_scene.last_error
		return
	_update_status()


func _process(_delta: float) -> void:
	_update_status()


func _physics_process(_delta: float) -> void:
	if grabbed_body_id <= 0 or mujoco_scene.simulation == null:
		return
	var body_transform := mujoco_scene.simulation.get_body_transform_by_id(
		grabbed_body_id)
	var point := body_transform * grabbed_local_point
	var velocity := mujoco_scene.simulation.get_body_point_velocity(
		grabbed_body_id, point)
	var force := (grab_target - point) * GRAB_STIFFNESS_N_PER_M \
		- velocity * GRAB_DAMPING_NS_PER_M
	if force.length() > GRAB_MAX_FORCE_N:
		force = force.normalized() * GRAB_MAX_FORCE_N
	mujoco_scene.simulation.clear_applied_forces()
	mujoco_scene.simulation.apply_body_force_at_point(grabbed_body_id,
		force, point)
	grab_marker.position = mujoco_scene.to_global(grab_target)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		_handle_mouse_button(event)
		return
	if event is InputEventMouseMotion:
		_handle_mouse_motion(event)
		return
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	match event.keycode:
		KEY_SPACE:
			mujoco_scene.set_paused(not mujoco_scene.is_paused())
		KEY_PERIOD:
			mujoco_scene.set_paused(true)
			mujoco_scene.single_step()
		KEY_R:
			mujoco_scene.reset_simulation()
		KEY_K:
			mujoco_scene.perturb_body("falling_shapes", Vector3(1.8, 1.1, 0.0),
				Vector3(0.0, 0.0, -0.35))
		KEY_BRACKETLEFT:
			mujoco_scene.time_scale = maxf(0.125, mujoco_scene.time_scale * 0.5)
		KEY_BRACKETRIGHT:
			mujoco_scene.time_scale = minf(8.0, mujoco_scene.time_scale * 2.0)


func _handle_mouse_button(event: InputEventMouseButton) -> void:
	if event.button_index == MOUSE_BUTTON_RIGHT:
		orbiting = event.pressed
	elif event.button_index == MOUSE_BUTTON_MIDDLE:
		panning = event.pressed
	elif event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			_begin_grab(event.position)
		else:
			_end_grab()
	elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
		camera_distance = maxf(0.25, camera_distance * 0.88)
		_update_camera()
	elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
		camera_distance = minf(100.0, camera_distance / 0.88)
		_update_camera()


func _handle_mouse_motion(event: InputEventMouseMotion) -> void:
	if orbiting:
		camera_yaw -= event.relative.x * 0.006
		camera_pitch = clampf(camera_pitch - event.relative.y * 0.006,
			-1.5, 1.5)
		_update_camera()
	elif panning:
		var scale := camera_distance * 0.0018
		camera_focus += camera.global_basis.x * -event.relative.x * scale
		camera_focus += camera.global_basis.y * event.relative.y * scale
		_update_camera()
	if grabbed_body_id > 0:
		var target_global := camera.project_position(event.position, grab_depth)
		grab_target = mujoco_scene.to_local(target_global)


func _begin_grab(screen_position: Vector2) -> void:
	if mujoco_scene.simulation == null:
		return
	var origin_global := camera.project_ray_origin(screen_position)
	var direction_global := camera.project_ray_normal(screen_position)
	var origin_local := mujoco_scene.to_local(origin_global)
	var direction_local := (mujoco_scene.global_basis.inverse()
		* direction_global).normalized()
	var hit: Dictionary = mujoco_scene.simulation.raycast(origin_local,
		direction_local, true)
	if hit.is_empty() or hit.body_id <= 0:
		return
	grabbed_body_id = hit.body_id
	grabbed_body_name = hit.body_name
	var body_transform := mujoco_scene.simulation.get_body_transform_by_id(
		grabbed_body_id)
	grabbed_local_point = body_transform.affine_inverse() * hit.position_m
	grab_target = hit.position_m
	grab_depth = origin_global.distance_to(
		mujoco_scene.to_global(hit.position_m))
	grab_marker.visible = true
	grab_marker.position = mujoco_scene.to_global(grab_target)


func _end_grab() -> void:
	grabbed_body_id = -1
	grabbed_body_name = ""
	grab_marker.visible = false
	if mujoco_scene.simulation != null:
		mujoco_scene.simulation.clear_applied_forces()


func _update_camera() -> void:
	var horizontal := cos(camera_pitch) * camera_distance
	camera.position = camera_focus + Vector3(
		sin(camera_yaw) * horizontal,
		-sin(camera_pitch) * camera_distance,
		cos(camera_yaw) * horizontal)
	camera.look_at(camera_focus, Vector3.UP)


func _make_grab_marker() -> void:
	grab_marker = MeshInstance3D.new()
	var mesh := SphereMesh.new()
	mesh.radius = 0.035
	mesh.height = 0.07
	grab_marker.mesh = mesh
	var material := StandardMaterial3D.new()
	material.albedo_color = Color(1.0, 0.85, 0.1)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	grab_marker.material_override = material
	grab_marker.visible = false
	add_child(grab_marker)


func _update_status() -> void:
	if mujoco_scene.simulation == null:
		return
	var contacts: Array = mujoco_scene.simulation.get_contacts()
	status.text = (
        "MuJoCo MJCF playground\n"
		+ "RMB: orbit   MMB: pan   wheel: zoom   LMB: grab\n"
		+ "Space: pause   .: step   R: reset   K: kick   [ ]: speed\n"
		+ "state: %s    speed: %.3fx    model dt: %.6f s\n"
		% ["paused" if mujoco_scene.is_paused() else "running",
			mujoco_scene.time_scale, mujoco_scene.model.get_timestep()]
		+ "simulation time: %.3f s    contacts: %d    selected: %s"
		% [mujoco_scene.simulation.get_time(), contacts.size(),
			grabbed_body_name if not grabbed_body_name.is_empty() else "none"])
