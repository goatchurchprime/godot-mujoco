#include "mujoco_data.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>

#include <cmath>
#include <cstring>

namespace godot {
namespace {
Vector3 mj_to_g(const mjtNum *v) { return Vector3(v[0], v[2], -v[1]); }
Vector3 g_to_mj(const Vector3 &v) { return Vector3(v.x, -v.z, v.y); }

Transform3D pose_to_g(const mjtNum *position, const mjtNum *matrix) {
	Basis basis;
	for (int column = 0; column < 3; ++column) {
		Vector3 godot_axis;
		godot_axis[column] = 1.0;
		Vector3 mj_axis = g_to_mj(godot_axis);
		mjtNum rotated[3] = {
			matrix[0] * mj_axis.x + matrix[1] * mj_axis.y + matrix[2] * mj_axis.z,
			matrix[3] * mj_axis.x + matrix[4] * mj_axis.y + matrix[5] * mj_axis.z,
			matrix[6] * mj_axis.x + matrix[7] * mj_axis.y + matrix[8] * mj_axis.z,
		};
		basis.set_column(column, mj_to_g(rotated));
	}
	return Transform3D(basis, mj_to_g(position));
}

Dictionary force_bundle(const mjModel *m, const mjData *d) {
	Dictionary result;
	auto pack = [m](const mjtNum *source) {
		PackedFloat64Array values; values.resize(m->nv);
		for (int i = 0; i < m->nv; ++i) values.set(i, source[i]);
		return values;
	};
	result["inverse_N_or_Nm"] = pack(d->qfrc_inverse);
	result["bias_N_or_Nm"] = pack(d->qfrc_bias);
	result["passive_N_or_Nm"] = pack(d->qfrc_passive);
	result["actuator_N_or_Nm"] = pack(d->qfrc_actuator);
	result["constraint_N_or_Nm"] = pack(d->qfrc_constraint);
	result["applied_N_or_Nm"] = pack(d->qfrc_applied);
	return result;
}
} // namespace

MujocoData::~MujocoData() { if (data_) mj_deleteData(data_); }

void MujocoData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("reset"), &MujocoData::reset);
	ClassDB::bind_method(D_METHOD("forward"), &MujocoData::forward);
	ClassDB::bind_method(D_METHOD("inverse"), &MujocoData::inverse);
	ClassDB::bind_method(D_METHOD("step", "substeps"), &MujocoData::step, DEFVAL(1));
	ClassDB::bind_method(D_METHOD("get_time"), &MujocoData::get_time);
	ClassDB::bind_method(D_METHOD("clear_applied_forces"), &MujocoData::clear_applied_forces);
	ClassDB::bind_method(D_METHOD("apply_body_wrench", "name", "force_n", "torque_nm"), &MujocoData::apply_body_wrench);
	ClassDB::bind_method(D_METHOD("set_qpos", "values"), &MujocoData::set_qpos);
	ClassDB::bind_method(D_METHOD("set_qvel", "values"), &MujocoData::set_qvel);
	ClassDB::bind_method(D_METHOD("set_qacc", "values"), &MujocoData::set_qacc);
	ClassDB::bind_method(D_METHOD("get_qpos"), &MujocoData::get_qpos);
	ClassDB::bind_method(D_METHOD("get_qvel"), &MujocoData::get_qvel);
	ClassDB::bind_method(D_METHOD("get_qacc"), &MujocoData::get_qacc);
	ClassDB::bind_method(D_METHOD("get_body_transform", "name"), &MujocoData::get_body_transform);
	ClassDB::bind_method(D_METHOD("get_body_transform_by_id", "id"), &MujocoData::get_body_transform_by_id);
	ClassDB::bind_method(D_METHOD("get_site_transform", "name"), &MujocoData::get_site_transform);
	ClassDB::bind_method(D_METHOD("get_geom_transform_by_id", "id"), &MujocoData::get_geom_transform_by_id);
	ClassDB::bind_method(D_METHOD("get_joint_frame", "name"), &MujocoData::get_joint_frame);
	ClassDB::bind_method(D_METHOD("get_body_com", "name"), &MujocoData::get_body_com);
	ClassDB::bind_method(D_METHOD("get_subtree_com", "name"), &MujocoData::get_subtree_com);
	ClassDB::bind_method(D_METHOD("get_contacts"), &MujocoData::get_contacts);
	ClassDB::bind_method(D_METHOD("get_constraint_forces"), &MujocoData::get_constraint_forces);
	ClassDB::bind_method(D_METHOD("get_generalized_forces"), &MujocoData::get_generalized_forces);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MujocoData::get_last_error);
}

bool MujocoData::initialize(const Ref<MujocoModel> &model) {
	if (model.is_null() || !model->native_model()) return false;
	owner_ = model;
	data_ = mj_makeData(model->native_model());
	if (!data_) { last_error_ = "mj_makeData failed"; return false; }
	return true;
}

void MujocoData::reset() { if (data_) mj_resetData(owner_->native_model(), data_); }
bool MujocoData::forward() { if (!data_) return false; mj_forward(owner_->native_model(), data_); return true; }
bool MujocoData::inverse() { if (!data_) return false; mj_inverse(owner_->native_model(), data_); return true; }
bool MujocoData::step(int substeps) {
	if (!data_ || substeps < 1 || substeps > 10000) {
		last_error_ = "step substeps must be between 1 and 10000";
		return false;
	}
	for (int i = 0; i < substeps; ++i) mj_step(owner_->native_model(), data_);
	last_error_ = "";
	return true;
}
double MujocoData::get_time() const { return data_ ? data_->time : 0.0; }
void MujocoData::clear_applied_forces() {
	if (data_) mju_zero(data_->xfrc_applied, 6 * owner_->native_model()->nbody);
}
bool MujocoData::apply_body_wrench(const StringName &name, const Vector3 &force_n,
		const Vector3 &torque_nm) {
	if (!force_n.is_finite() || !torque_nm.is_finite()) {
		last_error_ = "body wrench contains a non-finite component";
		return false;
	}
	int id = named_id(mjOBJ_BODY, name, "body");
	if (id < 0) return false;
	Vector3 force = g_to_mj(force_n); Vector3 torque = g_to_mj(torque_nm);
	mjtNum *wrench = data_->xfrc_applied + 6 * id;
	wrench[0] += force.x; wrench[1] += force.y; wrench[2] += force.z;
	wrench[3] += torque.x; wrench[4] += torque.y; wrench[5] += torque.z;
	last_error_ = "";
	return true;
}

bool MujocoData::set_vector(mjtNum *target, int size, const PackedFloat64Array &values, const char *label) {
	if (!data_) { last_error_ = "Simulation data is not initialized"; return false; }
	if (values.size() != size) {
		last_error_ = String(label) + " requires " + String::num_int64(size) + " values; got " + String::num_int64(values.size());
		return false;
	}
	for (int i = 0; i < size; ++i) {
		if (!std::isfinite(values[i])) { last_error_ = String(label) + " contains a non-finite value"; return false; }
	}
	for (int i = 0; i < size; ++i) target[i] = values[i];
	last_error_ = "";
	return true;
}

PackedFloat64Array MujocoData::get_vector(const mjtNum *source, int size) const {
	PackedFloat64Array result; if (!source) return result; result.resize(size);
	for (int i = 0; i < size; ++i) result.set(i, source[i]);
	return result;
}

bool MujocoData::set_qpos(const PackedFloat64Array &v) { return set_vector(data_ ? data_->qpos : nullptr, owner_->native_model()->nq, v, "qpos"); }
bool MujocoData::set_qvel(const PackedFloat64Array &v) { return set_vector(data_ ? data_->qvel : nullptr, owner_->native_model()->nv, v, "qvel"); }
bool MujocoData::set_qacc(const PackedFloat64Array &v) { return set_vector(data_ ? data_->qacc : nullptr, owner_->native_model()->nv, v, "qacc"); }
PackedFloat64Array MujocoData::get_qpos() const { return get_vector(data_ ? data_->qpos : nullptr, owner_->native_model()->nq); }
PackedFloat64Array MujocoData::get_qvel() const { return get_vector(data_ ? data_->qvel : nullptr, owner_->native_model()->nv); }
PackedFloat64Array MujocoData::get_qacc() const { return get_vector(data_ ? data_->qacc : nullptr, owner_->native_model()->nv); }

int MujocoData::named_id(mjtObj type, const StringName &name, const char *label) {
	CharString utf8 = String(name).utf8(); int id = mj_name2id(owner_->native_model(), type, utf8.get_data());
	if (id < 0) last_error_ = String("Unknown ") + label + ": " + String(name);
	return id;
}

Transform3D MujocoData::get_body_transform(const StringName &name) {
	int id = named_id(mjOBJ_BODY, name, "body"); if (id < 0) return Transform3D();
	return get_body_transform_by_id(id);
}
Transform3D MujocoData::get_body_transform_by_id(int id) {
	if (!data_ || id < 0 || id >= owner_->native_model()->nbody) {
		last_error_ = "Body id is out of range";
		return Transform3D();
	}
	last_error_ = "";
	return pose_to_g(data_->xpos + 3 * id, data_->xmat + 9 * id);
}
Transform3D MujocoData::get_site_transform(const StringName &name) {
	int id = named_id(mjOBJ_SITE, name, "site"); if (id < 0) return Transform3D();
	return pose_to_g(data_->site_xpos + 3 * id, data_->site_xmat + 9 * id);
}
Transform3D MujocoData::get_geom_transform_by_id(int id) {
	if (!data_ || id < 0 || id >= owner_->native_model()->ngeom) {
		last_error_ = "Geom id is out of range";
		return Transform3D();
	}
	last_error_ = "";
	return pose_to_g(data_->geom_xpos + 3 * id, data_->geom_xmat + 9 * id);
}
Dictionary MujocoData::get_joint_frame(const StringName &name) {
	Dictionary result; int id = named_id(mjOBJ_JOINT, name, "joint"); if (id < 0) return result;
	result["position_m"] = mj_to_g(data_->xanchor + 3 * id);
	result["axis_unit"] = mj_to_g(data_->xaxis + 3 * id);
	result["type"] = owner_->native_model()->jnt_type[id];
	return result;
}
Vector3 MujocoData::get_body_com(const StringName &name) { int id = named_id(mjOBJ_BODY, name, "body"); return id < 0 ? Vector3() : mj_to_g(data_->xipos + 3 * id); }
Vector3 MujocoData::get_subtree_com(const StringName &name) { int id = named_id(mjOBJ_BODY, name, "body"); return id < 0 ? Vector3() : mj_to_g(data_->subtree_com + 3 * id); }

Array MujocoData::get_contacts() {
	Array result; const mjModel *m = owner_->native_model();
	for (int i = 0; i < data_->ncon; ++i) {
		const mjContact &c = data_->contact[i]; Dictionary item;
		item["distance_m"] = c.dist; item["position_m"] = mj_to_g(c.pos);
		item["geom1_id"] = c.geom[0]; item["geom2_id"] = c.geom[1];
		const char *g1 = mj_id2name(m, mjOBJ_GEOM, c.geom[0]); const char *g2 = mj_id2name(m, mjOBJ_GEOM, c.geom[1]);
		item["geom1"] = g1 ? String::utf8(g1) : String(); item["geom2"] = g2 ? String::utf8(g2) : String();
		mjtNum wrench[6] = {}; mj_contactForce(m, data_, i, wrench);
		PackedFloat64Array force; force.resize(6); for (int j = 0; j < 6; ++j) force.set(j, wrench[j]);
		item["contact_frame_force_N_and_torque_Nm"] = force; item["constraint_address"] = c.efc_address;
		result.push_back(item);
	}
	return result;
}
PackedFloat64Array MujocoData::get_constraint_forces() const { return get_vector(data_ ? data_->efc_force : nullptr, data_ ? data_->nefc : 0); }
Dictionary MujocoData::get_generalized_forces() const { return data_ ? force_bundle(owner_->native_model(), data_) : Dictionary(); }
String MujocoData::get_last_error() const { return last_error_; }

} // namespace godot
