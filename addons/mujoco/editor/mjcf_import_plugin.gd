@tool
extends EditorImportPlugin


func _get_importer_name() -> String:
    return "godot_mujoco.mjcf_scene"


func _get_visible_name() -> String:
    return "MuJoCo MJCF Scene"


func _get_recognized_extensions() -> PackedStringArray:
    # Claiming .xml globally would intercept unrelated XML assets. Existing
    # .xml MJCF files remain loadable through MujocoScene3D's path property.
    return PackedStringArray(["mjcf", "mjz"])


func _get_save_extension() -> String:
    return "scn"


func _get_resource_type() -> String:
    return "PackedScene"


func _get_preset_count() -> int:
    return 1


func _get_preset_name(_preset_index: int) -> String:
    return "Default"


func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
    return []


func _get_import_order() -> int:
    return ResourceImporter.IMPORT_ORDER_SCENE


func _import(source_file: String, save_path: String, _options: Dictionary,
        _platform_variants: Array[String], _gen_files: Array[String]) -> Error:
    # Validate through MuJoCo at import time so malformed source fails loudly.
    var model := MujocoModel.new()
    if not model.load_mjcf(ProjectSettings.globalize_path(source_file)):
        push_error("MJCF import failed for %s: %s" % [source_file,
            model.get_last_error()])
        return ERR_PARSE_ERROR

    var empty_meshes := PackedStringArray()
    for mesh in model.get_meshes():
        if mesh.vertices.is_empty():
            empty_meshes.append(mesh.name)
    if not empty_meshes.is_empty():
        push_error("MJCF import found meshes with no vertex data: %s. "
            % ", ".join(empty_meshes)
            + "Copy the model's companion assets beside the MJCF using its "
            + "original directory layout (check <compiler meshdir=...>).")
        return ERR_FILE_NOT_FOUND

    var root := Node3D.new()
    root.name = source_file.get_file().get_basename().to_pascal_case()
    root.set_script(load("res://addons/mujoco/mujoco_scene_3d.gd"))
    root.set("mjcf_path", source_file)
    root.set("auto_load", true)
    root.set("simulate", true)
    root.set_meta("mujoco_source", source_file)

    # Materialize a read-only visual hierarchy into the imported PackedScene.
    # The non-tool runtime script leaves it untouched in the editor, then
    # removes and regenerates it from an authoritative mjData when run.
    if not root.call("load_mjcf", source_file):
        push_error("MJCF preview generation failed for %s: %s" % [source_file,
            root.get("last_error")])
        root.free()
        return ERR_CANT_CREATE
    _assign_owner_recursive(root, root)

    var packed := PackedScene.new()
    var packed_error := packed.pack(root)
    root.free()
    if packed_error != OK:
        return packed_error
    return ResourceSaver.save(packed, "%s.%s" % [save_path,
        _get_save_extension()])


func _assign_owner_recursive(node: Node, scene_root: Node) -> void:
    for child in node.get_children():
        child.owner = scene_root
        _assign_owner_recursive(child, scene_root)
