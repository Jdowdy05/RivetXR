"""Verify the full Newton APK, including native payloads in Chaquopy ZIPs.

This is a packaging contract, not proof of on-device imports, JIT or physics.
The old APIC-only contract intentionally remains separate.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tempfile
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.assets.convert_franka_meshes import ASSET_NAMES, BODY_NAMES, _contract, validate_qmsh
from tools.full_port.stage_python_runtime import ANDROID_WHEELS, NEWTON_PIN, MUJOCO_PIN, WARP_PIN

SYSTEM_LIBS = {'libc.so', 'libm.so', 'libdl.so', 'liblog.so', 'libandroid.so',
               'libGLESv3.so', 'libEGL.so', 'libz.so'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def run(*args):
    return subprocess.check_output([str(a) for a in args], text=True, stderr=subprocess.STDOUT)


def safe_name(name):
    require(name and '\\' not in name and ':' not in name and '!' not in name
            and not name.startswith('/') and all(p not in ('.', '..', '') for p in name.rstrip('/').split('/')),
            f'Unsafe archive/manifest path: {name}')


class Payloads:
    """Index actual archive names; .imy members remain individually inspectable."""
    def __init__(self, apk):
        self.archives = []
        self.entries = {}
        self.add(zipfile.ZipFile(apk))

    def add(self, archive, prefix=''):
        self.archives.append(archive)
        seen = set()
        for info in archive.infolist():
            safe_name(info.filename)
            require(info.filename not in seen, f'Duplicate ZIP member: {prefix}{info.filename}')
            seen.add(info.filename)
            if info.is_dir():
                continue
            name = prefix + info.filename
            self.entries[name] = (archive, info)
            if info.filename.endswith('.imy'):
                require(not prefix, f'Unexpected recursive .imy: {name}')
                self.add(zipfile.ZipFile(io.BytesIO(archive.read(info))), name + '!')

    def read(self, name):
        require(name in self.entries, f'Missing packaged payload: {name}')
        archive, info = self.entries[name]
        return archive.read(info)

    def python(self, relative):
        matches = [n for n in self.entries if '!' in n and n.split('!', 1)[1] == relative]
        require(len(matches) == 1, f'Expected one Python payload {relative}, found {matches}')
        return matches[0]

    def close(self):
        for archive in reversed(self.archives):
            archive.close()


def verify_manifest(xml):
    # Check decoded, merged manifest rather than source manifest declarations.
    for needle in ('package="com.questnewton"', 'com.oculus.supportedDevices',
                   'quest3|quest3s', 'android.app.lib_name', '"quest_newton"',
                   'com.questnewton.MainActivity', 'android.permission.INTERNET',
                   'com.oculus.permission.USE_SCENE', 'com.oculus.permission.USE_ANCHOR_API',
                   'android.intent.action.MAIN', 'android.intent.category.LAUNCHER',
                   'org.khronos.openxr.intent.category.IMMERSIVE_HMD', 'com.oculus.intent.category.VR'):
        require(needle in xml, f'Merged manifest missing {needle}')
    for key, value in {'compileSdkVersion': '34', 'minSdkVersion': '29', 'targetSdkVersion': '34',
                       'versionCode': '2', 'versionName': '"2.0-full-runtime"',
                       'extractNativeLibs': 'true', 'excludeFromRecents': 'true', 'exported': 'true'}.items():
        require(re.search(r'android:' + key + r'\([^\n]*?\)=' + re.escape(value) + r'(?:\s|$)', xml),
                f'Merged manifest value mismatch: {key}={value}')
    features = re.findall(r'E: uses-feature[^\n]*\n((?:\s+A:[^\n]*\n?)+)', xml)
    for feature in ('android.hardware.vr.headtracking', 'com.oculus.feature.PASSTHROUGH'):
        require(any(feature in block and re.search(r'android:required\([^\n]*?\)=true', block)
                    for block in features), f'Required VR feature missing: {feature}')
    require(any('android.hardware.vr.headtracking' in b and
                re.search(r'android:version\([^\n]*?\)=1(?:\s|$)', b) for b in features),
            'Headtracking feature version mismatch')
    require(any(re.search(r'android:glEsVersion\([^\n]*?\)=0x00030001', b) and
                re.search(r'android:required\([^\n]*?\)=true', b) for b in features),
            'OpenGL ES 3.1 feature missing')


def match_native(payload, source, strip, temp, label):
    if sha(payload) == sha(source.read_bytes()):
        return 'exact'
    # AGP strips jniLibs. Accept only the reproducible NDK transformation,
    # never an arbitrary ELF with matching SONAME or version strings.
    stripped = temp / 'identity.so'
    run(strip, '--strip-unneeded', '-o', stripped, source)
    require(sha(payload) == sha(stripped.read_bytes()), f'Packaged native hash mismatch: {label}')
    return 'ndk-strip-unneeded'


def verify_bundle(payloads, bundle, readelf, temp):
    manifest_bytes = (bundle / 'runtime_manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    expected = dict(schema_version=1, python='3.12', chaquopy='17.0.0', numpy='1.26.2',
                    newton=NEWTON_PIN, warp=WARP_PIN, mujoco=MUJOCO_PIN, mujoco_version='3.12.1')
    for key, value in expected.items():
        require(manifest.get(key) == value, f'Bundle identity mismatch: {key}')
    require(payloads.read('assets/full_runtime_provenance/runtime_manifest.json') == manifest_bytes,
            'Packaged bundle provenance mismatch')
    files = manifest.get('files')
    require(isinstance(files, dict) and files, 'Missing runtime file manifest')
    strip = readelf.with_name('llvm-strip' + readelf.suffix)
    counts = {'bundle_files': len(files), 'packaged_bundle_files': 0, 'native_identity': {}}
    for name, digest in files.items():
        safe_name(name)
        source = (bundle / name).resolve()
        require(source.is_relative_to(bundle) and source.is_file() and sha(source.read_bytes()) == digest,
                f'Bundle file hash mismatch: {name}')
        target = None
        if name.startswith('python/'):
            target = payloads.python(name.removeprefix('python/'))
        elif name.startswith('assets/'):
            target = name
        elif name.startswith('provenance/'):
            target = 'assets/full_runtime_provenance/' + PurePosixPath(name).name
        elif name.startswith('native/') and not name.endswith('/libc++_shared.so'):
            target = 'lib/arm64-v8a/' + source.name
        if target:
            data = payloads.read(target)
            if name.startswith('native/'):
                counts['native_identity'][target] = match_native(data, source, strip, temp, target)
            else:
                require(sha(data) == digest, f'Packaged bundle hash mismatch: {target}')
            counts['packaged_bundle_files'] += 1
    # libc++ comes solely from the selected NDK, never the older wheel copy.
    ndk_cxx = readelf.parent.parent / 'sysroot/usr/lib/aarch64-linux-android/libc++_shared.so'
    counts['native_identity']['libc++_shared.so'] = match_native(
        payloads.read('lib/arm64-v8a/libc++_shared.so'), ndk_cxx, strip, temp, 'NDK libc++')
    for wheel_name, (_, digest) in ANDROID_WHEELS.items():
        require(manifest.get('android_wheel_sha256', {}).get(wheel_name) == digest,
                f'Android wheel identity mismatch: {wheel_name}')
        wheel = bundle / 'wheels' / wheel_name
        require(sha(wheel.read_bytes()) == digest, f'Android wheel hash mismatch: {wheel_name}')
        checked = 0
        with zipfile.ZipFile(wheel) as archive:
            for name in archive.namelist():
                # Pip generates/rewrites metadata and compiles Python; native
                # wheel payloads must retain exact wheel bytes.
                if re.search(r'\.so(?:\.\d+)*$|\.a$', name):
                    target = payloads.python(name)
                    require(payloads.read(target) == archive.read(name), f'Wheel native hash mismatch: {target}')
                    checked += 1
                elif name.endswith('.dist-info/METADATA'):
                    target = payloads.python(name)
                    require(payloads.read(target) == archive.read(name), f'Wheel metadata mismatch: {target}')
        require(checked > 0, f'No native payload verified from wheel: {wheel_name}')
    counts['android_wheels'] = len(ANDROID_WHEELS)
    return counts


def verify_elf(payloads, readelf, temp):
    records = {}
    for name, (archive, info) in payloads.entries.items():
        if name.endswith('.imy'):
            continue
        with archive.open(info) as stream:
            magic = stream.read(4)
        require(not name.lower().endswith(('.dll', '.pyd', '.exe', '.dylib', '.lib')),
                f'Desktop/static native binary packaged: {name}')
        require(magic != b'MZ\x90\x00' and magic not in (b'\xcf\xfa\xed\xfe', b'\xfe\xed\xfa\xcf'),
                f'Desktop executable packaged: {name}')
        if name.endswith('.a'):
            path = temp / 'inspect.a'
            path.write_bytes(payloads.read(name))
            dump = run(readelf, '-h', '--wide', path)
            machines = re.findall(r'Machine:\s+([^\r\n]+)', dump)
            require(machines and all(m.strip() == 'AArch64' for m in machines),
                    f'Non-ARM64 static archive: {name}')
            continue
        native_name = re.search(r'\.so(?:\.\d+)*$', name)
        require(not native_name or magic == b'\x7fELF', f'Non-ELF shared library: {name}')
        if magic != b'\x7fELF':
            continue
        if name.startswith('lib/'):
            require(name.startswith('lib/arm64-v8a/'), f'Unsupported APK ABI: {name}')
        path = temp / 'inspect.so'
        path.write_bytes(payloads.read(name))
        dump = run(readelf, '-h', '-d', '--wide', path)
        require(re.search(r'Class:\s+ELF64', dump) and re.search(r'Machine:\s+AArch64', dump),
                f'Non-ARM64 ELF: {name}')
        sonames = re.findall(r'\(SONAME\).*\[([^\]]+)\]', dump)
        leaf = PurePosixPath(name.split('!')[-1]).name
        if name.startswith('lib/'):
            # Chaquopy's Gradle packaging removes the interpreter suffix from
            # this JNI library's filename; its pinned ELF SONAME retains it.
            expected_soname = 'libchaquopy_java-3.12.so' if leaf == 'libchaquopy_java.so' else leaf
            # The pinned Chaquopy CPython shared object has no DT_SONAME.
            expected = [] if leaf == 'libpython3.12.so' else [expected_soname]
            require(sonames == expected, f'APK native SONAME mismatch: {name}: {sonames}')
        if leaf == 'libquest_newton.so':
            symbols = run(readelf, '--dyn-syms', '--wide', path)
            require(re.search(r'FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+ANativeActivity_onCreate\s*$', symbols, re.M),
                    'NativeActivity entrypoint is not exported')
            require(b'com.questnewton.SimulationBridge' in path.read_bytes(), 'Native full-runtime JNI bridge missing')
        records[name] = dict(leaf=leaf, soname=sonames[0] if sonames else None,
                             needed=re.findall(r'\(NEEDED\).*\[([^\]]+)\]', dump))
    providers = {}
    for name, record in records.items():
        for identity in {record['leaf'], record['soname']} - {None}:
            require(identity not in providers or providers[identity] == name,
                    f'Ambiguous packaged ELF provider: {identity}')
            providers[identity] = name
    for name, record in records.items():
        missing = set(record['needed']) - SYSTEM_LIBS - providers.keys()
        require(not missing, f'Unpackaged ELF dependencies: {name}: {sorted(missing)}')
    return records


def verify_meshes(payloads):
    manifest = json.loads(payloads.read('assets/franka/franka_meshes.json'))
    require(manifest.get('schema_version') == 1 and manifest.get('body_count') == 12
            and manifest.get('visual_count') == 11, 'Franka visual schema mismatch')
    require([b['name'] for b in manifest['bodies']] == BODY_NAMES, 'Franka visual body order mismatch')
    require(manifest['bodies'] == _contract()['bodies'], 'Franka visual placement contract mismatch')
    require([a['name'] for a in manifest['assets']] == ASSET_NAMES, 'Franka visual asset order mismatch')
    urdf = payloads.read('assets/full_runtime/franka_description/robots/panda_arm_hand.urdf')
    require(sha(urdf) == manifest['urdf_sha256'], 'Visual/full runtime URDF mismatch')
    total = 0
    for asset in manifest['assets']:
        actual = validate_qmsh(payloads.read('assets/franka/' + asset['name']))
        require(all(asset.get(k) == v for k, v in actual.items()), f'QMSH metadata mismatch: {asset["name"]}')
        total += actual['bytes']
    require(total == manifest['total_geometry_bytes'] and total < 12 * 1024 * 1024, 'Visual geometry budget mismatch')
    return len(manifest['assets'])


def verify_remote_demos(payloads):
    from tools.remote_scene.demo import make_demo
    from tools.remote_scene.gimbal_demo import make_gimbal_demo
    from tools.remote_scene.protocol import pack_packet,unpack_packet
    report={}
    for name,mode in (('demo.rscn','points'),('demo-prepared.rscn','prepared'),('demo-unprepared.rscn','unprepared')):
        raw=payloads.read('assets/remote_scene/'+name)
        expected,_=make_demo(geometry=mode)
        require(raw==pack_packet(expected),f'Remote {mode} demo differs from current source')
        packet=unpack_packet(raw)
        report[mode]=dict(packet.geometry_summary,raw_bytes=len(raw),sha256=sha(raw))
    for mode in ('prepared','unprepared'):
        raw=payloads.read('assets/remote_scene/demo-rgb-'+mode+'.rscn')
        expected,_=make_gimbal_demo(geometry=mode)
        require(raw==pack_packet(expected),f'Remote RGB {mode} demo differs from current source')
        packet=unpack_packet(raw)
        require(packet.expected_mask==63,'RGB demo must include all six cameras')
        report['rgb-'+mode]=dict(packet.geometry_summary,raw_bytes=len(raw),sha256=sha(raw))
    return report


def verify(apk, bundle, aapt2, readelf, apksigner):
    signature = run(apksigner, 'verify', '--verbose', apk)
    require('Verified using v2 scheme (APK Signature Scheme v2): true' in signature, 'APK is not v2 signed')
    verify_manifest(run(aapt2, 'dump', 'xmltree', apk, '--file', 'AndroidManifest.xml'))
    payloads = Payloads(apk)
    try:
        for leaf in ('libquest_newton.so', 'libwarp.so', 'libwarp-clang.so', 'libmujoco.so',
                     'libpython3.12.so', 'libc++_shared.so', 'libopenxr_loader.so'):
            require('lib/arm64-v8a/' + leaf in payloads.entries, f'Missing required native library: {leaf}')
        for name in ('assets/panel.ktx', 'res/raw/efigs.fnt', 'res/raw/efigs_sdf.ktx'):
            require(len(payloads.read(name)) > 0, f'Empty TinyUI resource: {name}')
        for name in ('newton/__init__.py', 'warp/__init__.py', 'mujoco/__init__.py',
                     'mujoco_warp/__init__.py', 'warp/native/clang-resource/include/stddef.h'):
            payloads.python(name)
        repo = Path(__file__).resolve().parents[2]
        for source in (repo / 'quest/app/src/main/python/quest_sim').rglob('*.py'):
            name = source.relative_to(repo / 'quest/app/src/main/python').as_posix()
            require(payloads.read(payloads.python(name)) == source.read_bytes(), f'Application Python source mismatch: {name}')
        dex = b''.join(payloads.read(n) for n in payloads.entries if re.fullmatch(r'classes\d*\.dex', n))
        for descriptor in (b'Lcom/questnewton/MainActivity;', b'Lcom/questnewton/SimulationBridge;', b'Lcom/chaquo/python/Python;'):
            require(descriptor in dex, f'Missing Java class: {descriptor!r}')
        with tempfile.TemporaryDirectory(prefix='quest-full-apk-') as directory:
            temp = Path(directory)
            report = verify_bundle(payloads, bundle, readelf, temp)
            records = verify_elf(payloads, readelf, temp)
        report.update(apk=str(apk), apk_sha256=sha(apk.read_bytes()), native_libraries=len(records),
                      elf_dependencies=records, visual_meshes=verify_meshes(payloads),
                      python_archives=[n for n in payloads.entries if n.endswith('.imy')],
                      remote_demos=verify_remote_demos(payloads))
        return report
    finally:
        payloads.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ('apk', 'bundle', 'aapt2', 'readelf', 'apksigner'):
        parser.add_argument('--' + arg, required=True, type=Path)
    args = parser.parse_args()
    try:
        report = verify(**{k: v.resolve() for k, v in vars(args).items()})
    except (ValueError, OSError, KeyError, zipfile.BadZipFile, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Full APK contract failed: {error}\n')
    print('Full APK contract passed: ' + json.dumps(report, sort_keys=True))


if __name__ == '__main__':
    main()
