"""Verify the pinned full-runtime bundle and stage its native APK inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def prepare(bundle, staging):
    bundle, staging = Path(bundle).resolve(), Path(staging).resolve()
    manifest_file = bundle / 'runtime_manifest.json'
    manifest = json.loads(manifest_file.read_bytes())
    expected = {'schema_version': 1, 'python': '3.12', 'chaquopy': '17.0.0', 'numpy': '1.26.2',
                'newton': 'd37f4d3d341ccce1e06a1dff21e9a054759b4855',
                'warp': 'd4de134b97b961f1a19bd76830e71ea7f9df2470', 'mujoco_version': '3.12.1'}
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise ValueError(f'Full runtime identity mismatch: {key}')
    files = manifest.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('Full runtime file manifest is missing')
    for name, digest in files.items():
        path = (bundle / name).resolve()
        if bundle not in path.parents or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Full runtime file mismatch: {name}')
    for required in ('native/arm64-v8a/libwarp.so', 'native/arm64-v8a/libwarp-clang.so',
                     'native/arm64-v8a/libmujoco.so', 'python/newton/__init__.py',
                     'python/warp/native/clang-resource/include/stddef.h'):
        if required not in files:
            raise ValueError(f'Missing full runtime payload: {required}')
    native = staging / 'jniLibs/arm64-v8a'
    native.mkdir(parents=True, exist_ok=True)
    # NDK/AGP is the single provider of libc++; do not bundle Chaquopy's older copy.
    for path in (bundle / 'native/arm64-v8a').glob('*.so'):
        if path.name != 'libc++_shared.so':
            shutil.copy2(path, native / path.name)
    provenance = staging / 'assets/full_runtime_provenance'
    provenance.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manifest_file, provenance / manifest_file.name)
    shutil.copy2(bundle / 'provenance/warp_android_manifest.json', provenance / 'warp_android_manifest.json')
    print(f'Full runtime verified and staged: {len(files)} hashed files')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True, type=Path)
    parser.add_argument('--staging', required=True, type=Path)
    args = parser.parse_args()
    prepare(args.bundle, args.staging)
