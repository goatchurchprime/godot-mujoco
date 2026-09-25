#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <mujoco/mujoco.h>

namespace godot {

class MujocoData;

class MujocoModel : public RefCounted {
	GDCLASS(MujocoModel, RefCounted)

	mjModel *model_ = nullptr;
	String last_error_;

protected:
	static void _bind_methods();

public:
	~MujocoModel() override;
	bool load_mjcf(const String &path);
	bool compile_mjcf(const String &xml);
	Ref<MujocoData> create_data();
	String get_last_error() const;
	Dictionary get_sizes() const;
	PackedStringArray get_names(const StringName &object_type) const;

	const mjModel *native_model() const { return model_; }
	mjModel *native_model() { return model_; }
};

} // namespace godot
