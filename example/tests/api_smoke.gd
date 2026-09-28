extends SceneTree

const XML := """<mujoco model="api-smoke">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 .1"/>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom name="link" type="capsule" fromto="0 0 0 0 0 .5" size=".05" mass="1"/>
      <site name="tip" pos="0 0 .5"/>
    </body>
  </worldbody>
</mujoco>"""

const TWO_MESH_XML := """<mujoco model="mesh-offset-smoke">
  <asset>
    <mesh name="first" vertex="0 0 0  1 0 0  0 1 0" face="0 1 2"/>
    <mesh name="second" vertex="10 0 0  11 0 0  10 1 0" face="0 1 2"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="first"/>
    <geom type="mesh" mesh="second"/>
  </worldbody>
</mujoco>"""

func _init() -> void:
    var model := MujocoModel.new()
    assert(model.compile_mjcf(XML), model.get_last_error())
    assert(model.get_sizes().nq == 1)
    var first := model.create_data()
    var second := model.create_data()
    assert(first != null and second != null and first != second)
    assert(first.set_qpos(PackedFloat64Array([0.35])))
    assert(second.set_qpos(PackedFloat64Array([-0.2])))
    assert(first.forward() and second.forward())
    assert(abs(first.get_qpos()[0] - second.get_qpos()[0]) > 0.5)
    assert(first.get_body_transform("pendulum").origin.is_finite())
    assert(first.get_site_transform("tip").origin.is_finite())
    assert(first.get_joint_frame("hinge").axis_unit.is_normalized())
    assert(first.set_qvel(PackedFloat64Array([0.0])))
    assert(first.set_qacc(PackedFloat64Array([0.0])))
    var before := first.get_qpos()
    assert(first.inverse())
    assert(first.get_qpos() == before, "mj_inverse changed qpos")
    assert(first.get_generalized_forces().inverse_N_or_Nm.size() == 1)
    first.clear_applied_forces()
    assert(first.apply_body_wrench("pendulum", Vector3(3, 0, 0), Vector3.ZERO))
    var time_before := first.get_time()
    assert(first.step(2))
    assert(first.get_time() > time_before)
    assert(not first.set_qpos(PackedFloat64Array([1.0, 2.0])))
    assert("requires 1" in first.get_last_error())
    assert(model.get_joints().size() == 1)
    assert(model.get_joints()[0].type == "hinge")

    var mesh_model := MujocoModel.new()
    assert(mesh_model.compile_mjcf(TWO_MESH_XML), mesh_model.get_last_error())
    var meshes: Array = mesh_model.get_meshes()
    assert(meshes.size() == 2)
    assert(meshes[0].vertices.size() == 3)
    assert(meshes[1].vertices.size() == 3)
    var first_center: Vector3 = (meshes[0].vertices[0]
        + meshes[0].vertices[1] + meshes[0].vertices[2]) / 3.0
    var second_center: Vector3 = (meshes[1].vertices[0]
        + meshes[1].vertices[1] + meshes[1].vertices[2]) / 3.0
    assert(first_center.distance_to(second_center) > 5.0,
        "second mesh used the first mesh's shared-buffer vertex range")
    print("godot-mujoco API smoke: PASS")
    quit(0)
