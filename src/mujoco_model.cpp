#include "mujoco_model.hpp"
#include "mujoco_data.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include <cstring>

namespace godot {

MujocoModel::~MujocoModel() {
	if (model_) mj_deleteModel(model_);
}

void MujocoModel::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_mjcf", "path"), &MujocoModel::load_mjcf);
	ClassDB::bind_method(D_METHOD("compile_mjcf", "xml"), &MujocoModel::compile_mjcf);
	ClassDB::bind_method(D_METHOD("create_data"), &MujocoModel::create_data);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MujocoModel::get_last_error);
	ClassDB::bind_method(D_METHOD("get_sizes"), &MujocoModel::get_sizes);
	ClassDB::bind_method(D_METHOD("get_names", "object_type"), &MujocoModel::get_names);
}

bool MujocoModel::load_mjcf(const String &path) {
	CharString utf8 = path.utf8();
	char error[2048] = {};
	mjModel *candidate = mj_loadXML(utf8.get_data(), nullptr, error, sizeof(error));
	if (!candidate) {
		last_error_ = String::utf8(error);
		return false;
	}
	if (model_) mj_deleteModel(model_);
	model_ = candidate;
	last_error_ = "";
	return true;
}

bool MujocoModel::compile_mjcf(const String &xml) {
	CharString utf8 = xml.utf8();
	char error[2048] = {};
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	const char *virtual_name = "godot-memory-model.xml";
	if (mj_addBufferVFS(&vfs, virtual_name, utf8.get_data(), utf8.length()) != 0) {
		mj_deleteVFS(&vfs);
		last_error_ = "Could not create the in-memory MJCF file";
		return false;
	}
	mjModel *candidate = mj_loadXML(virtual_name, &vfs, error, sizeof(error));
	mj_deleteVFS(&vfs);
	if (!candidate) {
		last_error_ = String::utf8(error);
		return false;
	}
	if (model_) mj_deleteModel(model_);
	model_ = candidate;
	last_error_ = "";
	return true;
}

Ref<MujocoData> MujocoModel::create_data() {
	Ref<MujocoData> result;
	if (!model_) {
		last_error_ = "No MJCF model has been compiled";
		return result;
	}
	result.instantiate();
	Ref<MujocoModel> self(this);
	if (!result->initialize(self)) result.unref();
	return result;
}

String MujocoModel::get_last_error() const { return last_error_; }

Dictionary MujocoModel::get_sizes() const {
	Dictionary d;
	if (!model_) return d;
	d["nq"] = model_->nq; d["nv"] = model_->nv; d["na"] = model_->na;
	d["nu"] = model_->nu; d["nbody"] = model_->nbody; d["njnt"] = model_->njnt;
	d["nsite"] = model_->nsite; d["ngeom"] = model_->ngeom;
	return d;
}

PackedStringArray MujocoModel::get_names(const StringName &object_type) const {
	PackedStringArray out;
	if (!model_) return out;
	mjtObj type;
	int count;
	String key = object_type;
	if (key == "body") { type = mjOBJ_BODY; count = model_->nbody; }
	else if (key == "joint") { type = mjOBJ_JOINT; count = model_->njnt; }
	else if (key == "site") { type = mjOBJ_SITE; count = model_->nsite; }
	else if (key == "geom") { type = mjOBJ_GEOM; count = model_->ngeom; }
	else return out;
	for (int i = 0; i < count; ++i) {
		const char *name = mj_id2name(model_, type, i);
		out.push_back(name ? String::utf8(name) : String());
	}
	return out;
}

} // namespace godot
