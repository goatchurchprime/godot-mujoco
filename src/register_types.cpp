#include "mujoco_data.hpp"
#include "mujoco_model.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_mujoco(ModuleInitializationLevel level) {
	if (level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
	ClassDB::register_class<MujocoModel>();
	ClassDB::register_class<MujocoData>();
}
void uninitialize_mujoco(ModuleInitializationLevel level) { (void)level; }

extern "C" GDExtensionBool GDE_EXPORT mujoco_library_init(
		GDExtensionInterfaceGetProcAddress address,
		GDExtensionClassLibraryPtr library, GDExtensionInitialization *initialization) {
	GDExtensionBinding::InitObject init(address, library, initialization);
	init.register_initializer(initialize_mujoco);
	init.register_terminator(uninitialize_mujoco);
	init.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
	return init.init();
}
