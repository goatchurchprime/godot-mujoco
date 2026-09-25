#pragma once

#include "mujoco_model.hpp"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/transform3d.hpp>

namespace godot {

class MujocoData : public RefCounted {
	GDCLASS(MujocoData, RefCounted)

	Ref<MujocoModel> owner_;
	mjData *data_ = nullptr;
	String last_error_;

	bool set_vector(mjtNum *target, int size, const PackedFloat64Array &values,
			const char *label);
	PackedFloat64Array get_vector(const mjtNum *source, int size) const;
	int named_id(mjtObj type, const StringName &name, const char *label);

protected:
	static void _bind_methods();

public:
	~MujocoData() override;
	bool initialize(const Ref<MujocoModel> &model);
	void reset();
	bool forward();
	bool inverse();

	bool set_qpos(const PackedFloat64Array &values);
	bool set_qvel(const PackedFloat64Array &values);
	bool set_qacc(const PackedFloat64Array &values);
	PackedFloat64Array get_qpos() const;
	PackedFloat64Array get_qvel() const;
	PackedFloat64Array get_qacc() const;

	Transform3D get_body_transform(const StringName &name);
	Transform3D get_site_transform(const StringName &name);
	Dictionary get_joint_frame(const StringName &name);
	Vector3 get_body_com(const StringName &name);
	Vector3 get_subtree_com(const StringName &name);
	Array get_contacts();
	PackedFloat64Array get_constraint_forces() const;
	Dictionary get_generalized_forces() const;
	String get_last_error() const;
};

} // namespace godot
