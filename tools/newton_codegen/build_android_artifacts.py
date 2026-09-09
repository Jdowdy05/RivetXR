"""Build a pinned Franka APIC bundle using explicit local source/tool paths.

Run as python -m tools.newton_codegen.build_android_artifacts from the repo.
The output must be a new, empty directory (normally under ignored out/ or a
shallow local build tree). All intermediate files stay inside that directory.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from tools.newton_codegen.artifact_manifest import (
    NEWTON_COMMIT, WARP_COMMIT, write_manifest,
)


def run(command, **kwargs):
    print('+', subprocess.list2cmdline([str(c) for c in command]), flush=True)
    subprocess.run([str(c) for c in command], check=True, **kwargs)


def require_commit(source, expected):
    source = Path(source)
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain',
                                    '--untracked-files=no'], text=True).strip()
    if actual != expected or dirty:
        raise ValueError(f'Required clean source commit {expected}: {source}')


def validate_output(output, sources):
    output = Path(output).resolve()
    for source in sources:
        source = Path(source).resolve()
        if output == source or output in source.parents or source in output.parents:
            raise ValueError('Output must not overlap an input source tree')


def build(args):
    output = args.output.resolve()
    require_commit(args.newton_source, NEWTON_COMMIT)
    require_commit(args.warp_source, WARP_COMMIT)
    validate_output(output, [args.newton_source, args.warp_source, args.franka_description_root, args.ndk])
    if output.exists() and any(output.iterdir()):
        raise ValueError('Output must be empty; choose a new directory for each build')
    ndk_properties = (args.ndk / 'source.properties').read_text()
    if 'Pkg.Revision = 27.0.12077973' not in ndk_properties:
        raise ValueError('NDK 27.0.12077973 required')
    output.mkdir(parents=True, exist_ok=True)
    scratch = output / '_build'
    scratch.mkdir()
    isolated = scratch / 'warp'
    env = dict(os.environ, GIT_LFS_SKIP_SMUDGE='1', PYTHONDONTWRITEBYTECODE='1')
    run(['git', 'clone', '--local', '--no-hardlinks', '--no-checkout', args.warp_source, isolated], env=env)
    run(['git', '-C', isolated, 'checkout', '--detach', WARP_COMMIT], env=env)
    patch = Path(__file__).resolve().parents[2] / 'cmake/patches/warp-android-runtime.patch'
    run(['git', '-C', isolated, 'apply', '--check', patch])
    run(['git', '-C', isolated, 'apply', patch])
    # Host binaries are read from the supplied pinned Warp source. Never build
    # or install into that source or change the user's Python environment.
    env['PYTHONPATH'] = os.pathsep.join([str(args.warp_source.resolve()),
                                       str(args.newton_source.resolve())])
    run([args.python, '-B', '-m', 'tools.newton_codegen.capture_franka',
         '--franka-description-root', args.franka_description_root,
         '--output', output, '--cache', scratch / 'cache'], env=env,
        cwd=Path(__file__).resolve().parents[2])
    build_dir = scratch / 'warp-build'
    run([args.cmake, '-S', isolated, '-B', build_dir, '-G', 'Ninja',
         f'-DCMAKE_MAKE_PROGRAM={args.ninja.resolve().as_posix()}',
         f'-DCMAKE_TOOLCHAIN_FILE={(args.ndk / "build/cmake/android.toolchain.cmake").resolve().as_posix()}',
         '-DANDROID_ABI=arm64-v8a', '-DANDROID_PLATFORM=android-29',
         '-DWARP_ENABLE_CUDA=OFF', '-DWARP_BUILD_CLANG=OFF',
         f'-DPython3_EXECUTABLE={args.python.resolve().as_posix()}'], env=env)
    run([args.cmake, '--build', build_dir, '--target', 'warp', '--parallel', str(args.jobs)], env=env)
    # Android requires a lib-prefixed package basename and the Warp patch sets
    # the matching DT_SONAME. A post-build rename would leave an unsafe ELF.
    shutil.copyfile(isolated / 'warp/bin/libwarp.so', output / 'libwarp.so')
    host_tag = 'windows-x86_64' if os.name == 'nt' else 'linux-x86_64'
    toolchain = args.ndk / 'toolchains/llvm/prebuilt' / host_tag
    suffix = '.exe' if os.name == 'nt' else ''
    clang = toolchain / 'bin' / f'clang++{suffix}'
    strip = toolchain / 'bin' / f'llvm-strip{suffix}'
    metadata = json.loads((output / 'capture_metadata.json').read_text())
    objects = []
    for index, source in enumerate(metadata.pop('generated_cpp')):
        obj = scratch / f'kernel-{index}.o'
        run([clang, '--target=aarch64-linux-android29', '-std=c++17', '-O3', '-fPIC',
             '-fno-fast-math', '-I', isolated / 'warp/native', '-c',
             output / 'generated_cpp' / source, '-o', obj])
        objects.append(obj)
    module = 'libquest_newton_kernels_franka.so'
    run([clang, '--target=aarch64-linux-android29', '-shared', *objects,
         '-L', output, '-l:libwarp.so', '-Wl,--no-undefined', '-Wl,-soname,' + module,
         '-o', output / module])
    shutil.copyfile(toolchain / 'sysroot/usr/lib/aarch64-linux-android/libc++_shared.so',
                    output / 'libc++_shared.so')
    binaries = ['franka_reset.wrp', 'franka_step.wrp', module, 'libwarp.so', 'libc++_shared.so']
    for name in binaries:
        if name.endswith('.so'):
            run([strip, '--strip-unneeded', output / name])
    metadata.update(android_abi='arm64-v8a', android_api=29, ndk_version='27.0.12077973',
                    kernel_modules=[module])
    write_manifest(metadata, output, binaries)
    run([args.python, '-B', '-m', 'tools.newton_codegen.verify_bundle',
         '--artifact-dir', output, '--franka-description-root', args.franka_description_root,
         '--readelf', toolchain / 'bin' / f'llvm-readelf{suffix}'], env=env,
        cwd=Path(__file__).resolve().parents[2])
    require_commit(args.newton_source, NEWTON_COMMIT)
    require_commit(args.warp_source, WARP_COMMIT)
    print(f'Bundle complete: {output / "artifact_manifest.json"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'newton-source', 'warp-source', 'ndk', 'ninja'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--franka-description-root', type=Path,
                        default=os.environ.get('FRANKA_DESCRIPTION_ROOT'))
    parser.add_argument('--python', type=Path, default=Path(sys.executable))
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--jobs', type=int, default=8)
    args = parser.parse_args()
    if not args.franka_description_root:
        parser.error('Set FRANKA_DESCRIPTION_ROOT or --franka-description-root')
    build(args)


if __name__ == '__main__':
    main()
