@tool
class_name MujocoJoint3D
extends Node3D

## Inspectable joint frame owned and solved by MuJoCo.

@export var mujoco_id := -1
@export var source_name := ""
@export_enum("free", "ball", "slide", "hinge", "unknown") var joint_type := "unknown"
@export var axis_unit := Vector3.ZERO
@export var limited := false
@export var range_rad_or_m := Vector2.ZERO
@export var damping := 0.0
