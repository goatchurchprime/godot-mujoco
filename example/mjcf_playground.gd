extends Node3D

## A deliberately small `simulate`-style playground for source-authoritative
## MJCF files. MuJoCo owns dynamics; generated Godot nodes are presentation.

@onready var mujoco_scene: MujocoScene3D = $MujocoScene3D
@onready var status: Label = $Interface/Status


func _ready() -> void:
    if mujoco_scene.simulation == null:
        status.text = "MJCF load failed:\n%s" % mujoco_scene.last_error
        return
    _update_status()


func _process(_delta: float) -> void:
    _update_status()


func _unhandled_input(event: InputEvent) -> void:
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


func _update_status() -> void:
    if mujoco_scene.simulation == null:
        return
    var contacts: Array = mujoco_scene.simulation.get_contacts()
    status.text = (
        "MuJoCo MJCF playground\n"
        + "Space: pause   .: step   R: reset   K: kick   [ ]: speed\n"
        + "state: %s    speed: %.3fx    model dt: %.6f s\n"
        % ["paused" if mujoco_scene.is_paused() else "running",
            mujoco_scene.time_scale, mujoco_scene.model.get_timestep()]
        + "simulation time: %.3f s    contacts: %d"
        % [mujoco_scene.simulation.get_time(), contacts.size()])
