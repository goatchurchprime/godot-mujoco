#!/usr/bin/env python
import os
import sys
from SCons.Script import ARGUMENTS, Default, Dir, SConscript

godot_cpp_dir = ARGUMENTS.get("godot_cpp_dir", "godot-cpp")
godot_cpp = Dir(godot_cpp_dir)
if not godot_cpp.exists():
    print("godot-cpp missing; initialize the submodule or pass godot_cpp_dir=...", file=sys.stderr)
    sys.exit(1)

env = SConscript(os.path.join(godot_cpp_dir, "SConstruct"))
for key in ("PATH", "CPATH", "LIBRARY_PATH", "LD_LIBRARY_PATH"):
    if os.environ.get(key):
        env["ENV"][key] = os.environ[key]

mujoco_root = ARGUMENTS.get("mujoco_root", os.environ.get("MUJOCO_ROOT", ""))
if not mujoco_root:
    print("MUJOCO_ROOT is required (the Nix dev shell sets it)", file=sys.stderr)
    sys.exit(1)

env.Append(CPPPATH=["src", os.path.join(mujoco_root, "include")])
env.Append(LIBPATH=[os.path.join(mujoco_root, "lib")], LIBS=["mujoco"])
if env["platform"] != "windows":
    env.Append(CXXFLAGS=["-Wall", "-Wextra", "-Werror",
                         "-Wno-unused-parameter", "-Wno-unused-variable"])
if env["platform"] == "linux":
    env.Append(LINKFLAGS=["-Wl,-rpath,$ORIGIN", "-Wl,-z,noexecstack"])

suffix = env["suffix"] + env["SHLIBSUFFIX"]
target = "addons/mujoco/bin/libmujoco_godot" + suffix
library = env.SharedLibrary(target, [
    "src/mujoco_model.cpp",
    "src/mujoco_data.cpp",
    "src/register_types.cpp",
])
Default(library)
