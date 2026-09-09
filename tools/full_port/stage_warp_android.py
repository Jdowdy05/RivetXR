"""Stage the Android LLVM SDK or the verified Warp CPU JIT payload.

Called by build_warp_android.ps1. Downloads and generated files stay outside git.
ELF checks establish packaging compatibility, not successful Android execution.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

WARP_COMMIT = "d4de134b97b961f1a19bd76830e71ea7f9df2470"
LLVM_COMMIT = "ca7933e47d3a3451d81e72ac174dcb5aa28b59d1"
SYSTEM_LIBRARIES = {"libc.so", "libm.so", "libdl.so", "liblog.so"}


def merge_tree(source: Path, destination: Path) -> None:
    shutil.copytree(source, destination, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".git"))


def stage_sdk(root: Path) -> None:
    sdk = root / "llvm-android-sdk"
    # Source declarations and generated target/config headers are both needed.
    for source in (root / "llvm/llvm/include", root / "llvm/clang/include",
                   root / "llvm-android/include", root / "llvm-android/tools/clang/include"):
        merge_tree(source, sdk / "include")
    (sdk / "lib").mkdir(parents=True, exist_ok=True)
    archives = sorted((root / "llvm-android/lib").glob("*.a"))
    if not archives:
        raise RuntimeError("Android LLVM build produced no static libraries")
    for archive in archives:
        shutil.copy2(archive, sdk / "lib" / archive.name)


def stage_bundle(root: Path, ndk: Path, destination: Path) -> None:
    # A failed restage must never leave a previous success manifest in place.
    (destination / "warp_android_manifest.json").unlink(missing_ok=True)
    libraries = destination / "native/arm64-v8a"
    libraries.mkdir(parents=True, exist_ok=True)
    for name in ("libwarp.so", "libwarp-clang.so"):
        shutil.copy2(root / "warp/warp/bin" / name, libraries / name)
    toolchain = ndk / "toolchains/llvm/prebuilt/windows-x86_64"
    # Keep original build outputs intact for debugging; only strip APK copies.
    for name in ("libwarp.so", "libwarp-clang.so"):
        subprocess.run([str(toolchain / "bin/llvm-strip.exe"), "--strip-unneeded", str(libraries / name)], check=True)
    shutil.copy2(toolchain / "sysroot/usr/lib/aarch64-linux-android/libc++_shared.so",
                 libraries / "libc++_shared.so")
    python_warp = destination / "python/warp"
    shutil.copytree(root / "warp/warp", python_warp, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("bin", "__pycache__", "*.pyc", "tests", "examples"))
    merge_tree(root / "llvm-android/lib/clang/22/include", python_warp / "native/clang-resource/include")
    licenses = destination / "licenses"
    licenses.mkdir(exist_ok=True)
    merge_tree(root / "warp/licenses", licenses / "warp-third-party")
    for source, name in ((root / "warp/LICENSE.md", "Warp-LICENSE.md"),
                         (root / "llvm/llvm/LICENSE.TXT", "LLVM-LICENSE.TXT"),
                         (ndk / "NOTICE", "Android-NDK-NOTICE.txt"),
                         (ndk / "NOTICE.toolchain", "Android-NDK-toolchain-NOTICE.txt")):
        shutil.copy2(source, licenses / name)

    readelf = toolchain / "bin/llvm-readelf.exe"
    evidence = {}
    for path in sorted(libraries.glob("*.so")):
        elf = subprocess.check_output([str(readelf), "-h", "-d", "--dyn-syms", "--wide", str(path)], text=True)
        if not re.search(r"Machine:\s+AArch64", elf):
            raise RuntimeError(f"Not Android AArch64: {path}")
        soname = re.search(r"\(SONAME\).*\[([^]]+)\]", elf)
        if not soname or soname.group(1) != path.name:
            raise RuntimeError(f"SONAME mismatch: {path}")
        needed = re.findall(r"\(NEEDED\).*\[([^]]+)\]", elf)
        missing = set(needed) - SYSTEM_LIBRARIES - {p.name for p in libraries.glob("*.so")}
        if missing:
            raise RuntimeError(f"Unpackaged dependencies for {path}: {sorted(missing)}")
        required = {"libwarp.so": ("wp_version",),
                    "libwarp-clang.so": ("wp_compile_cpp", "wp_load_obj", "wp_lookup", "wp_warp_clang_version")}.get(path.name, ())
        for symbol in required:
            if not re.search(rf"GLOBAL\s+DEFAULT\s+(?!UND\b)\S+\s+{symbol}(?:\s|$)", elf):
                raise RuntimeError(f"Missing exported {symbol} in {path}")
        evidence[path.name] = {"needed": needed, "soname": soname.group(1), "machine": "AArch64"}
        (root / "logs" / f"{path.name}.readelf.txt").write_text(elf)
    payload = {}
    for path in sorted(destination.rglob("*")):
        if path.is_file() and path.name != "warp_android_manifest.json":
            payload[path.relative_to(destination).as_posix()] = {
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "bytes": path.stat().st_size}
    manifest = {"schema_version": 1, "warp_commit": WARP_COMMIT, "llvm_commit": LLVM_COMMIT,
                "warp_patch_sha256": hashlib.sha256((Path(__file__).parent / "patches/warp-android-jit.patch").read_bytes()).hexdigest(),
                "llvm_version": "22.1.8", "android_api": 29, "ndk_version": "27.0.12077973",
                "backend": "cpu-jit", "elf": evidence, "files": payload,
                "android_execution_verified": False}
    (destination / "warp_android_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("sdk", "bundle"))
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--ndk", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.mode == "sdk":
        stage_sdk(args.root)
    else:
        if args.ndk is None or args.output is None:
            parser.error("bundle requires --ndk and --output")
        stage_bundle(args.root, args.ndk, args.output)


if __name__ == "__main__":
    main()
