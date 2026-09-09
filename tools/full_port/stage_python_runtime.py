"""Assemble an offline Android Newton Python/native/assets bundle.

Chaquopy supplies Python 3.12 and its NumPy 1.26.2 Android wheel. Other
runtime dependencies are staged here, with source/download/artifact hashes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import urllib.request
import zipfile

NEWTON_PIN = "d37f4d3d341ccce1e06a1dff21e9a054759b4855"
MUJOCO_PIN = "10eeb8289598421cbca0b39459652ed57ec3d2e6"
WARP_PIN = "d4de134b97b961f1a19bd76830e71ea7f9df2470"
WHEELS = {
    "mujoco_warp-3.12.0-py3-none-any.whl": (
        "47/24/d60f407b469be023566c8e149eb1598ff7d8f9b16332cdadc4fa45ebee3a",
        "a0d5d5df854dbf49727cbe2463f8b1c44117cc979cf17ae91d81f291ae0a3b62"),
    "trimesh-4.8.3-py3-none-any.whl": (
        "8e/bc/f36afd1ad03d09cd0d311be5951f2dd705a824f34dd871779d42c277d741",
        "f9f1622ecac5f3bed5b65d3a748bb536481f52cd4e4e120e0b637cabe6cf1542"),
    "typing_extensions-4.15.0-py3-none-any.whl": (
        "18/67/36e9267722cc04a6b9f15c7f3441c2363321a3ea07da7ae0c0707beb2a9c",
        "f0fa19c6845758ab08074a0cfa8b7aecb71c999ca73d62883bc25cc018c4e548"),
}
ANDROID_WHEELS = {
    "numpy-1.26.2-0-cp312-cp312-android_21_arm64_v8a.whl": (
        "numpy", "d144088be25177f118c4a91ee88c7b4c37e38209f7c05c9b496b21480d95cc75"),
    "chaquopy_openblas-0.2.20-5-py3-none-android_21_arm64_v8a.whl": (
        "chaquopy-openblas", "1e8e67a4f9e2fcde384890678bafbf7ffefe7b2cb4887e5c2996ba7607f4324c"),
    "chaquopy_libgfortran-4.9-0-py3-none-android_21_arm64_v8a.whl": (
        "chaquopy-libgfortran", "0b4caed1147f2a19707d4ba730afea57f7b8f2d8c046d8a48cf49377741c4604"),
}
EXTENSIONS = ("_callbacks", "_constants", "_enums", "_errors", "_functions",
              "_structs", "_specs", "_rollout", "_render")
SYSTEM_LIBS = {"libc.so", "libm.so", "libdl.so", "liblog.so", "libandroid.so", "libz.so", "libpython3.12.so"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_package(source, dest):
    shutil.copytree(source, dest, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyd", "*.dll", "tests", "examples", "*_test.py"))


def check_pin(path, expected):
    actual = subprocess.check_output(["git", "-C", path, "rev-parse", "HEAD"], text=True).strip()
    if actual != expected:
        raise ValueError(f"source pin mismatch: {path}: {actual}")
    if subprocess.run(["git", "-C", str(path), "diff", "--quiet", "HEAD", "--"]).returncode:
        raise ValueError(f"tracked source modifications must be recorded as a portability patch: {path}")


def verify_elf(root, readelf):
    libraries = list((root / "native/arm64-v8a").glob("*.so")) + list((root / "python/mujoco").glob("*.so"))
    supplied = {p.name for p in libraries} | SYSTEM_LIBS
    records = {}
    for path in libraries:
        info = subprocess.check_output([str(readelf), "-h", "-d", str(path)], text=True)
        if "AArch64" not in info or "ELF64" not in info:
            raise ValueError(f"not Android ARM64 ELF: {path}")
        needed = re.findall(r"\(NEEDED\).*\[([^\]]+)\]", info)
        missing = set(needed) - supplied
        if missing:
            raise ValueError(f"unresolved dependencies for {path.name}: {sorted(missing)}")
        records[path.relative_to(root).as_posix()] = needed
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("newton-source", "mujoco-source", "mujoco-build", "mujoco-library",
                "warp-bundle", "franka-description-root", "readelf", "output"):
        parser.add_argument("--" + arg, type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    if root.exists() and any(root.iterdir()):
        raise ValueError("use an empty output directory to prevent stale packaged files")
    check_pin(args.newton_source, NEWTON_PIN)
    check_pin(args.mujoco_source, MUJOCO_PIN)
    warp_manifest_file = args.warp_bundle / "warp_android_manifest.json"
    warp_manifest = json.loads(warp_manifest_file.read_text())
    if warp_manifest["warp_commit"] != WARP_PIN or warp_manifest["backend"] != "cpu-jit":
        raise ValueError("Warp compiler bundle source/backend mismatch")
    for relative, record in warp_manifest["files"].items():
        path = (args.warp_bundle / relative).resolve()
        if not path.is_relative_to(args.warp_bundle.resolve()) or sha(path) != record["sha256"]:
            raise ValueError(f"Warp bundle file/hash mismatch: {relative}")
    # Fail before producing a misleading partial bundle.
    for path in [args.warp_bundle / "native/arm64-v8a/libwarp.so",
                 args.warp_bundle / "native/arm64-v8a/libwarp-clang.so",
                 args.warp_bundle / "python/warp/native/clang-resource/include/stddef.h",
                 *[args.mujoco_build / "mujoco" / (name + ".so") for name in EXTENSIONS]]:
        if not path.is_file():
            raise FileNotFoundError(path)
    py = root / "python"
    native = root / "native/arm64-v8a"
    py.mkdir(parents=True)
    native.mkdir(parents=True)
    copy_package(args.newton_source / "newton", py / "newton")
    # Newton's _version.py reads installed distribution metadata. Source-only
    # staging otherwise reports "unknown" despite the verified source pin.
    dist = py / "newton-1.6.0.dev0.dist-info"
    dist.mkdir()
    (dist / "METADATA").write_text("Metadata-Version: 2.4\nName: newton\nVersion: 1.6.0.dev0\n"
                                  "Requires-Dist: warp-lang>=1.17.0\nRequires-Dist: mujoco~=3.12.0\n"
                                  "Requires-Dist: mujoco-warp~=3.12.0\n")
    (dist / "top_level.txt").write_text("newton\n")
    shutil.copy2(args.newton_source / "LICENSE.md", py / "newton/LICENSE.md")
    copy_package(args.warp_bundle / "python/warp", py / "warp")
    for path in (args.warp_bundle / "native/arm64-v8a").glob("*.so"):
        shutil.copy2(path, native / path.name)
    # Newton's public initializer eagerly imports optional desktop viewers.
    # Keep all engine APIs and optional source modules; defer the viewer import.
    init = py / "newton/__init__.py"
    text = init.read_text()
    original = "from . import actuators, controllers, geometry, ik, math, selection, sensors, solvers, usd, utils, viewer"
    if text.count(original) != 1:
        raise ValueError("Newton initializer changed: review headless import patch")
    init.write_text(text.replace(original, original.removesuffix(", viewer")).replace('    "viewer",\n', ""))
    mujoco = py / "mujoco"
    mujoco.mkdir()
    shutil.copy2(args.mujoco_source / "python/mujoco/__init__.py", mujoco / "__init__.py")
    shutil.copy2(args.mujoco_source / "LICENSE", mujoco / "LICENSE")
    for name in EXTENSIONS:
        shutil.copy2(args.mujoco_build / "mujoco" / (name + ".so"), mujoco / (name + ".so"))
        # Python package extensions are assets, so Android Gradle's jniLibs
        # stripping does not apply. Keep upstream build symbols externally.
        strip = args.readelf.with_name("llvm-strip" + args.readelf.suffix)
        subprocess.run([str(strip), "--strip-unneeded", str(mujoco / (name + ".so"))], check=True)
    shutil.copy2(args.mujoco_library, native / "libmujoco.so")
    downloads = root / "downloads"
    downloads.mkdir()
    for filename, (urlpart, digest) in WHEELS.items():
        wheel = downloads / filename
        urllib.request.urlretrieve(f"https://files.pythonhosted.org/packages/{urlpart}/{filename}", wheel)
        if sha(wheel) != digest:
            raise ValueError(f"download hash mismatch: {filename}")
        with zipfile.ZipFile(wheel) as archive:
            for member in archive.infolist():
                if not member.filename.endswith("_test.py") and "/tests/" not in member.filename:
                    archive.extract(member, py)
    android_wheels = root / "wheels"
    android_wheels.mkdir()
    for filename, (package, digest) in ANDROID_WHEELS.items():
        wheel = android_wheels / filename
        urllib.request.urlretrieve(f"https://chaquo.com/pypi-13.1/{package}/{filename}", wheel)
        if sha(wheel) != digest:
            raise ValueError(f"Android wheel hash mismatch: {filename}")
    assets = root / "assets/full_runtime/franka_description"
    (assets / "robots").mkdir(parents=True)
    shutil.copy2(args.franka_description_root / "robots/panda_arm_hand.urdf", assets / "robots/panda_arm_hand.urdf")
    shutil.copytree(args.franka_description_root / "meshes/collision", assets / "meshes/collision")
    # Source assets remain independently packaged by the existing native visual exporter.
    for license_name in ("LICENSE", "LICENSE.md", "LICENSE.txt"):
        source = args.franka_description_root / license_name
        if source.is_file():
            shutil.copy2(source, assets / license_name)
    licenses = root / "assets/full_runtime/licenses"
    shutil.copytree(args.warp_bundle / "licenses", licenses / "warp")
    shutil.copy2(args.mujoco_source / "LICENSE", licenses / "MuJoCo-LICENSE")
    shutil.copy2(args.newton_source / "LICENSE.md", licenses / "Newton-LICENSE.md")
    # Preserve the notices of native code statically linked into our libraries.
    dependency_roots = [args.mujoco_build / "_deps", args.mujoco_library.parent.parent / "_deps"]
    for dep_root in dependency_roots:
        if not dep_root.is_dir():
            raise FileNotFoundError(f"native dependency notices required: {dep_root}")
        for dep in dep_root.glob("*-src"):
            for notice in dep.iterdir():
                if notice.is_file() and any(word in notice.name.upper() for word in ("LICENSE", "COPYING", "NOTICE")):
                    dest = licenses / "mujoco-dependencies" / dep.name / notice.name
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(notice, dest)
    provenance = root / "provenance"
    provenance.mkdir()
    shutil.copy2(warp_manifest_file, provenance / warp_manifest_file.name)
    dependencies = verify_elf(root, args.readelf)
    manifest = dict(schema_version=1, python="3.12", chaquopy="17.0.0", numpy="1.26.2",
                    newton=NEWTON_PIN, warp=WARP_PIN, mujoco=MUJOCO_PIN, mujoco_version="3.12.1",
                    mujoco_warp="3.12.0 PyPI wheel", wheel_sha256={k: v[1] for k, v in WHEELS.items()},
                    android_wheel_sha256={k: v[1] for k, v in ANDROID_WHEELS.items()},
                    elf_dependencies=dependencies, files={})
    for path in sorted(root.rglob("*")):
        if path.is_file():
            manifest["files"][path.relative_to(root).as_posix()] = sha(path)
    (root / "runtime_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(dict(bundle=str(root), files=len(manifest["files"]), native_libraries=len(dependencies))))


if __name__ == "__main__":
    main()
