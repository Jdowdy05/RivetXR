"""Required verification of an actual Franka bundle; missing inputs are errors."""
import re
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest

from tools.newton_codegen.artifact_manifest import (
    GRAPH_OPERATIONS, NEWTON_VERSION, WARP_VERSION, canonicalize_graph, canonical_json, verify_artifacts,
)


def parse_elf(text):
    def field(pattern):
        matches = re.findall(pattern, text, re.MULTILINE)
        if len(matches) != 1:
            raise ValueError(f'Missing or ambiguous ELF field: {pattern}')
        return matches[0].strip()
    symbols = set()
    for line in text.splitlines():
        values = line.split()
        if (len(values) == 8 and values[0].rstrip(':').isdigit()
                and values[3] == 'FUNC' and values[4] in ('GLOBAL', 'WEAK')
                and values[5] in ('DEFAULT', 'PROTECTED') and values[6] != 'UND'):
            symbols.add(values[7].split('@')[0])
    return {'machine': field(r'^\s*Machine:\s*(.+)$'),
            'elf_class': field(r'^\s*Class:\s*(.+)$'),
            'soname': field(r'\(SONAME\).*\[([^\]]+)\]'),
            'needed': set(re.findall(r'\(NEEDED\).*\[([^\]]+)\]', text)),
            'symbols': symbols}


def validate_elf_closure(records, kernel_modules, required_symbols):
    if not kernel_modules or not required_symbols:
        raise ValueError('Missing required kernel modules or graph symbols')
    allowed_system = {'libc.so', 'libm.so', 'libdl.so'}
    for name, record in records.items():
        if record['machine'] != 'AArch64' or record['elf_class'] != 'ELF64' or record['soname'] != name:
            raise ValueError(f'Wrong ELF architecture or SONAME: {name}')
        missing = record['needed'] - records.keys() - allowed_system
        if missing:
            raise ValueError(f'Unpackaged ELF dependency: {name}: {sorted(missing)}')
    if not set(kernel_modules) <= records.keys():
        raise ValueError('Missing packaged kernel module')
    symbols = set().union(*(records[name]['symbols'] for name in kernel_modules))
    missing = set(required_symbols) - symbols
    if missing:
        raise ValueError(f'Missing graph kernel exports: {sorted(missing)}')


def verify_bundle(root, description_root, readelf):
    root, description_root, readelf = Path(root).resolve(), Path(description_root).resolve(), Path(readelf)
    for required in (root / 'artifact_manifest.json', description_root / 'robots/panda_arm_hand.urdf', readelf):
        if not required.is_file():
            raise ValueError(f'Required verification input missing: {required}')
    manifest_path = root / 'artifact_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    verify_artifacts(manifest, root)
    modules = manifest['kernel_modules']
    expected = {'franka_reset.wrp', 'franka_step.wrp', 'libwarp.so', 'libc++_shared.so', *modules}
    if {item['name'] for item in manifest['artifacts']} != expected:
        raise ValueError('Runtime binary allowlist differs from manifest')
    import newton
    import warp as wp
    if (newton.__version__, wp.__version__) != (NEWTON_VERSION, WARP_VERSION):
        raise ValueError('Required pinned host versions missing')
    wp.config.kernel_cache_dir = str(root / '_build/verification-cache')
    wp.init()
    from warp._src.context import runtime
    symbols = set()
    for kind in ('reset', 'step'):
        graph_file = root / f'franka_{kind}.wrp'
        data = graph_file.read_bytes()
        if canonicalize_graph(data, expected_operations=GRAPH_OPERATIONS[kind]) != data:
            raise ValueError('Graph is not canonical')
        graph = wp.capture_load(str(graph_file), device='cpu')
        for index in range(runtime.core.wp_apic_get_num_kernels(graph._native_graph)):
            for get_name in (runtime.core.wp_apic_get_kernel_forward_name,
                             runtime.core.wp_apic_get_kernel_backward_name):
                name = get_name(graph._native_graph, index)
                if name:
                    symbols.add(name.decode())
        del graph
    if symbols != set(manifest['required_kernel_symbols']):
        raise ValueError('Manifest kernel symbols do not match the actual graphs')
    records = {}
    for name in sorted(expected):
        if name.endswith('.so'):
            output = subprocess.check_output([str(readelf), '-h', '-d', '--dyn-syms', '--wide',
                                              str(root / name)], text=True)
            records[name] = parse_elf(output)
    validate_elf_closure(records, modules, symbols)
    # Set required inputs before test discovery so opt-in decorators cannot
    # skip real graph/model tests in a production build. Any skip is an error.
    os.environ['FRANKA_DESCRIPTION_ROOT'] = str(description_root)
    os.environ['FRANKA_ARTIFACT_DIR'] = str(root)
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).parent / 'tests'))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful() or result.skipped or result.testsRun == 0:
        raise ValueError('Required artifact verification failed or skipped tests')
    verify_artifacts(manifest, root)
    report = {'verified': True, 'tests_run': result.testsRun, 'tests_skipped': 0,
              'manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
              'graph_kernel_symbol_count': len(symbols),
              'elf_dependencies': {name: sorted(record['needed']) for name, record in records.items()}}
    (root / 'verification_report.json').write_text(canonical_json(report), encoding='utf-8')
    print('Required graph semantics, kernel exports, and AArch64 dependency closure verified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', required=True, type=Path)
    parser.add_argument('--franka-description-root', required=True, type=Path)
    parser.add_argument('--readelf', required=True, type=Path)
    args = parser.parse_args()
    verify_bundle(args.artifact_dir, args.franka_description_root, args.readelf)


if __name__ == '__main__':
    main()
