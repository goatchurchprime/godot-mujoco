#include "mujoco_model.hpp"
#include "mujoco_data.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include <cstring>

namespace godot {
namespace {
Vector3 mj_to_g(const mjtNum *v) { return Vector3(v[0], v[2], -v[1]); }
Vector3 g_to_mj(const Vector3 &v) { return Vector3(v.x, -v.z, v.y); }

Transform3D pose_to_g(const mjtNum *position, const mjtNum *quaternion) {
	mjtNum matrix[9];
	mju_quat2Mat(matrix, quaternion);
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

const char *geom_type_name(int type) {
	switch (type) {
		case mjGEOM_PLANE: return "plane";
		case mjGEOM_HFIELD: return "hfield";
		case mjGEOM_SPHERE: return "sphere";
		case mjGEOM_CAPSULE: return "capsule";
		case mjGEOM_ELLIPSOID: return "ellipsoid";
		case mjGEOM_CYLINDER: return "cylinder";
		case mjGEOM_BOX: return "box";
		case mjGEOM_MESH: return "mesh";
		case mjGEOM_SDF: return "sdf";
		default: return "unknown";
	}
}
} // namespace

MujocoModel::~MujocoModel() {
	if (model_) mj_deleteModel(model_);
}

void MujocoModel::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_mjcf", "path"), &MujocoModel::load_mjcf);
	ClassDB::bind_method(D_METHOD("compile_mjcf", "xml"), &MujocoModel::compile_mjcf);
	ClassDB::bind_method(D_METHOD("create_data"), &MujocoModel::create_data);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MujocoModel::get_last_error);
	ClassDB::bind_method(D_METHOD("get_timestep"), &MujocoModel::get_timestep);
	ClassDB::bind_method(D_METHOD("get_sizes"), &MujocoModel::get_sizes);
	ClassDB::bind_method(D_METHOD("get_names", "object_type"), &MujocoModel::get_names);
	ClassDB::bind_method(D_METHOD("get_bodies"), &MujocoModel::get_bodies);
	ClassDB::bind_method(D_METHOD("get_geoms"), &MujocoModel::get_geoms);
	ClassDB::bind_method(D_METHOD("get_meshes"), &MujocoModel::get_meshes);
	ClassDB::bind_method(D_METHOD("get_textures"), &MujocoModel::get_textures);
	ClassDB::bind_method(D_METHOD("get_materials"), &MujocoModel::get_materials);
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

double MujocoModel::get_timestep() const {
	return model_ ? model_->opt.timestep : 0.0;
}

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

Array MujocoModel::get_bodies() const {
	Array result;
	if (!model_) return result;
	for (int id = 0; id < model_->nbody; ++id) {
		Dictionary item;
		const char *name = mj_id2name(model_, mjOBJ_BODY, id);
		item["id"] = id;
		item["name"] = name ? String::utf8(name) : String();
		item["parent_id"] = model_->body_parentid[id];
		item["local_transform"] = pose_to_g(model_->body_pos + 3 * id,
				model_->body_quat + 4 * id);
		item["mass_kg"] = model_->body_mass[id];
		result.push_back(item);
	}
	return result;
}

Array MujocoModel::get_geoms() const {
	Array result;
	if (!model_) return result;
	for (int id = 0; id < model_->ngeom; ++id) {
		Dictionary item;
		const char *name = mj_id2name(model_, mjOBJ_GEOM, id);
		const int body_id = model_->geom_bodyid[id];
		const char *body_name = mj_id2name(model_, mjOBJ_BODY, body_id);
		item["id"] = id;
		item["name"] = name ? String::utf8(name) : String();
		item["body_id"] = body_id;
		item["body_name"] = body_name ? String::utf8(body_name) : String();
		item["type"] = geom_type_name(model_->geom_type[id]);
		item["type_id"] = model_->geom_type[id];
		item["data_id"] = model_->geom_dataid[id];
		item["material_id"] = model_->geom_matid[id];
		item["size_mj"] = Vector3(model_->geom_size[3 * id],
				model_->geom_size[3 * id + 1], model_->geom_size[3 * id + 2]);
		item["size_godot_axes_m"] = Vector3(model_->geom_size[3 * id],
				model_->geom_size[3 * id + 2], model_->geom_size[3 * id + 1]);
		item["local_transform"] = pose_to_g(model_->geom_pos + 3 * id,
				model_->geom_quat + 4 * id);
		const float *rgba = model_->geom_rgba + 4 * id;
		if (model_->geom_matid[id] >= 0) {
			rgba = model_->mat_rgba + 4 * model_->geom_matid[id];
		}
		item["color"] = Color(rgba[0], rgba[1], rgba[2], rgba[3]);
		result.push_back(item);
	}
	return result;
}

Array MujocoModel::get_meshes() const {
	Array result;
	if (!model_) return result;
	for (int id = 0; id < model_->nmesh; ++id) {
		Dictionary item;
		const char *name = mj_id2name(model_, mjOBJ_MESH, id);
		item["id"] = id;
		item["name"] = name ? String::utf8(name) : String();
		PackedVector3Array vertices;
		PackedVector3Array normals;
		PackedVector2Array texcoords;
		const int face_address = model_->mesh_faceadr[id];
		const int normal_address = model_->mesh_normaladr[id];
		const int texcoord_address = model_->mesh_texcoordadr[id];
		const int corner_count = 3 * model_->mesh_facenum[id];
		vertices.resize(corner_count);
		if (normal_address >= 0) normals.resize(corner_count);
		if (texcoord_address >= 0) texcoords.resize(corner_count);
		for (int corner = 0; corner < corner_count; ++corner) {
			const int face_slot = 3 * face_address + corner;
			const int vertex_id = model_->mesh_face[face_slot];
			const float *vertex = model_->mesh_vert + 3 * vertex_id;
			mjtNum scaled[3] = {
				vertex[0] * model_->mesh_scale[3 * id],
				vertex[1] * model_->mesh_scale[3 * id + 1],
				vertex[2] * model_->mesh_scale[3 * id + 2],
			};
			mjtNum transformed[3];
			mju_rotVecQuat(transformed, scaled, model_->mesh_quat + 4 * id);
			mju_addTo3(transformed, model_->mesh_pos + 3 * id);
			vertices.set(corner, mj_to_g(transformed));
			if (normal_address >= 0) {
				const int normal_id = model_->mesh_facenormal[face_slot];
				const float *normal = model_->mesh_normal + 3 * normal_id;
				mjtNum rotated[3];
				mjtNum source[3] = {normal[0], normal[1], normal[2]};
				mju_rotVecQuat(rotated, source, model_->mesh_quat + 4 * id);
				normals.set(corner, mj_to_g(rotated).normalized());
			}
			if (texcoord_address >= 0) {
				const int texcoord_id = model_->mesh_facetexcoord[face_slot];
				const float *uv = model_->mesh_texcoord + 2 * texcoord_id;
				texcoords.set(corner, Vector2(uv[0], 1.0f - uv[1]));
			}
		}
		item["vertices"] = vertices;
		item["normals"] = normals;
		item["texcoords"] = texcoords;
		result.push_back(item);
	}
	return result;
}

Array MujocoModel::get_textures() const {
	Array result;
	if (!model_) return result;
	for (int id = 0; id < model_->ntex; ++id) {
		Dictionary item;
		const char *name = mj_id2name(model_, mjOBJ_TEXTURE, id);
		const int byte_count = model_->tex_width[id] * model_->tex_height[id] *
				model_->tex_nchannel[id];
		PackedByteArray bytes;
		bytes.resize(byte_count);
		memcpy(bytes.ptrw(), model_->tex_data + model_->tex_adr[id], byte_count);
		item["id"] = id;
		item["name"] = name ? String::utf8(name) : String();
		item["type_id"] = model_->tex_type[id];
		item["width"] = model_->tex_width[id];
		item["height"] = model_->tex_height[id];
		item["channels"] = model_->tex_nchannel[id];
		item["data"] = bytes;
		result.push_back(item);
	}
	return result;
}

Array MujocoModel::get_materials() const {
	Array result;
	if (!model_) return result;
	for (int id = 0; id < model_->nmat; ++id) {
		Dictionary item;
		const char *name = mj_id2name(model_, mjOBJ_MATERIAL, id);
		const float *rgba = model_->mat_rgba + 4 * id;
		item["id"] = id;
		item["name"] = name ? String::utf8(name) : String();
		item["color"] = Color(rgba[0], rgba[1], rgba[2], rgba[3]);
		item["roughness"] = model_->mat_roughness[id];
		item["metallic"] = model_->mat_metallic[id];
		item["texture_repeat"] = Vector2(model_->mat_texrepeat[2 * id],
				model_->mat_texrepeat[2 * id + 1]);
		int texture_id = model_->mat_texid[id * mjNTEXROLE + mjTEXROLE_RGB];
		if (texture_id < 0) texture_id = model_->mat_texid[id * mjNTEXROLE + mjTEXROLE_RGBA];
		if (texture_id < 0) texture_id = model_->mat_texid[id * mjNTEXROLE + mjTEXROLE_USER];
		item["color_texture_id"] = texture_id;
		result.push_back(item);
	}
	return result;
}

} // namespace godot
