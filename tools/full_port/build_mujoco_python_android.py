"""Cross-build pinned MuJoCo CPU Python bindings for Chaquopy CPython 3.12.

Uses upstream binding targets/code generators; excludes only desktop simulate UI.
All downloads and generated files live under --output, never upstream sources.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import re
import subprocess
import urllib.request
import zipfile

TARGET_VERSION = "3.12.12-0"
TARGET_SHA256 = "7a4a278ec1fed0e0d0359fbec71d6d481d79a7efbc8e2090267a91046417027a"
MUJOCO_PIN = "10eeb8289598421cbca0b39459652ed57ec3d2e6"


def run(args, **kwargs):
    print(" ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "core-library", "ndk", "host-python", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--jobs", type=int, default=3)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    pin = subprocess.check_output(["git", "-C", args.source, "rev-parse", "HEAD"], text=True).strip()
    if pin != MUJOCO_PIN:
        raise ValueError(f"MuJoCo pin mismatch: {pin}")
    if subprocess.run(["git", "-C", str(args.source), "diff", "--quiet", "HEAD", "--"]).returncode:
        raise ValueError("MuJoCo source must be clean; apply portability edits only in the isolated build copy")
    archive = out / "target.zip"
    url = f"https://repo.maven.apache.org/maven2/com/chaquo/python/target/{TARGET_VERSION}/target-{TARGET_VERSION}-arm64-v8a.zip"
    if not archive.exists():
        urllib.request.urlretrieve(url, archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != TARGET_SHA256:
        raise ValueError("Chaquopy target archive hash mismatch")
    target = out / "target"
    with zipfile.ZipFile(archive) as stream:
        stream.extractall(target)
    (out / "target.sha256").write_text(hashlib.sha256(archive.read_bytes()).hexdigest() + "\n")
    src = out / "source"
    shutil.copytree(args.source / "python/mujoco", src / "mujoco", dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    env = dict(os.environ, PYTHONPATH=str(src / "mujoco"))
    for gen, dest in (("generate_enum_traits.py", "enum_traits.h"),
                      ("generate_function_traits.py", "function_traits.h"),
                      ("generate_spec_bindings.py", "specs.cc.inc")):
        with (src / "mujoco" / dest).open("w", encoding="utf-8") as stream:
            run([args.host_python, src / "mujoco/codegen" / gen], env=env, stdout=stream)
    original = (args.source / "python/mujoco/CMakeLists.txt").read_text()
    targets = original[original.index("add_subdirectory(util)"):original.index("mujoco_pybind11_module(_simulate")]
    targets = targets.replace("  pybind11_add_module(${name} ${ARGN})", """  add_library(${name} MODULE ${ARGN})
  target_include_directories(${name} PRIVATE ${Python3_INCLUDE_DIRS} ${pybind11_SOURCE_DIR}/include)
  target_link_libraries(${name} PRIVATE python_android)
  set_target_properties(${name} PROPERTIES PREFIX "" SUFFIX ".so")""")
    targets = targets.replace("include(CheckAvxSupport)\nget_avx_compile_options(AVX_COMPILE_OPTIONS)", "")
    cmake = '''cmake_minimum_required(VERSION 3.24)
project(mujoco_android_python LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(absl GIT_REPOSITORY https://github.com/abseil/abseil-cpp GIT_TAG 5650e9cf76d3be4318d5fa3af38ee483ddfd5e4a)
FetchContent_MakeAvailable(absl)
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen GIT_TAG ea13a98decd497a8c5588fb5de71b57bcf10d864)
FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11 GIT_TAG 97bf890db679505a14dfe547a5e77bb2bd05dc90)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
endif()
FetchContent_GetProperties(pybind11)
if(NOT pybind11_POPULATED)
  FetchContent_Populate(pybind11)
endif()
add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers INTERFACE ${eigen_SOURCE_DIR})
add_library(Eigen3::Eigen ALIAS eigen_headers)
add_library(pybind_headers INTERFACE)
target_include_directories(pybind_headers INTERFACE ${pybind11_SOURCE_DIR}/include)
add_library(pybind11::headers ALIAS pybind_headers)
add_library(mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION "@CORE@")
target_include_directories(mujoco INTERFACE "@MUJOCO@/include")
add_library(python_android SHARED IMPORTED GLOBAL)
set_target_properties(python_android PROPERTIES IMPORTED_LOCATION "@TARGET@/jniLibs/arm64-v8a/libpython3.12.so" IMPORTED_NO_SONAME TRUE)
set(Python3_INCLUDE_DIRS "@TARGET@/include/python3.12")
set(mujoco_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/mujoco" "${Python3_INCLUDE_DIRS}")
add_subdirectory(mujoco)
'''
    cmake = cmake.replace("@CORE@", args.core_library.resolve().as_posix()).replace("@MUJOCO@", args.source.resolve().as_posix()).replace("@TARGET@", target.as_posix())
    (src / "CMakeLists.txt").write_text(cmake)
    (src / "mujoco/CMakeLists.txt").write_text(targets)
    run(["cmake", "-S", src, "-B", out / "build", "-G", "Ninja",
         f"-DCMAKE_TOOLCHAIN_FILE={args.ndk.as_posix()}/build/cmake/android.toolchain.cmake",
         "-DANDROID_ABI=arm64-v8a", "-DANDROID_PLATFORM=android-29", "-DANDROID_STL=c++_shared",
         "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"])
    run(["cmake", "--build", out / "build", "--parallel", args.jobs])
    toolchain = args.ndk / "toolchains/llvm/prebuilt/windows-x86_64/bin"
    readelf = toolchain / "llvm-readelf.exe"
    records = {}
    for extension in sorted((out / "build/mujoco").glob("_*.so")):
        info = subprocess.check_output([str(readelf), "-h", "-d", "--dyn-syms", str(extension)], text=True)
        dependencies = re.findall(r"\(NEEDED\).*\[([^\]]+)\]", info)
        if "AArch64" not in info or f"PyInit_{extension.stem}" not in info:
            raise ValueError(f"invalid Android Python extension: {extension}")
        if any("/" in dep or "\\" in dep for dep in dependencies):
            raise ValueError(f"non-portable dependency path in {extension}: {dependencies}")
        records[extension.name] = dict(sha256=hashlib.sha256(extension.read_bytes()).hexdigest(), needed=dependencies)
    (out / "bindings_manifest.json").write_text(json.dumps(dict(
        python_target=TARGET_VERSION, target_sha256=TARGET_SHA256, mujoco_pin=MUJOCO_PIN,
        core_sha256=hashlib.sha256(args.core_library.read_bytes()).hexdigest(), extensions=records), indent=2) + "\n")


if __name__ == "__main__":
    main()
