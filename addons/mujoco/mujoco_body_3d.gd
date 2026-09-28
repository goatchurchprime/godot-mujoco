@tool
class_name MujocoBody3D
extends Node3D

## Editor/runtime representation of a body owned by MuJoCo. This deliberately
## does not inherit RigidBody3D: Jolt must not simulate the same body again.

@export var mujoco_id := -1
@export var source_name := ""
@export var mass_kg := 0.0
@export var dynamic := false
