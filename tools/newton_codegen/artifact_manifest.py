"""Canonical, portable metadata for the pinned precompiled runtime bundle."""
import hashlib
import json
import struct
from collections import Counter
from pathlib import Path

NEWTON_COMMIT = 'd37f4d3d341ccce1e06a1dff21e9a054759b4855'
WARP_COMMIT = 'd4de134b97b961f1a19bd76830e71ea7f9df2470'
NEWTON_VERSION = '1.6.0.dev0'
WARP_VERSION = '1.18.0.dev2'


GRAPH_OPERATIONS = {'reset': {1: 1, 4: 2}, 'step': {1: 25, 4: 1, 5: 8}}


def canonicalize_graph(payload, *, expected_operations=None):
    """Erase stale host pointers that APIC overwrites through its relocations.

    Pinned warp/native/apic_types.h defines packed format 15. Only the CPU
    operations emitted by this capture are accepted. We never change region
    IDs, offsets, sizes, scalar arguments, or simulation state. In particular,
    handle relocations are rejected because those need original pointer IDs.
    """
    data = bytearray(payload)
    if len(data) < 64 or data[:4] != b'WRP1' or struct.unpack_from('<I', data, 4)[0] != 15:
        raise ValueError('Expected APIC format 15')
    if data[24] != 1 or struct.unpack_from('<I', data, 28)[0] != 0:
        raise ValueError('Expected CPU APIC graph')
    count = struct.unpack_from('<I', data, 12)[0]
    table = struct.unpack_from('<Q', data, 16)[0]
    if table + count * 32 > len(data):
        raise ValueError('Truncated section table')
    operations = []
    for index in range(count):
        kind, flags, offset, size, uncompressed = struct.unpack_from('<IIQQQ', data, table + index * 32)
        if flags or size != uncompressed or offset + size > len(data):
            raise ValueError('Invalid APIC section')
        if kind == 3:
            operations.append((offset, size))
    if len(operations) != 1:
        raise ValueError('Expected one operation section')
    offset, size = operations[0]
    end = offset + size
    if size < 4:
        raise ValueError('Missing operation count')
    op_count = struct.unpack_from('<I', data, offset)[0]
    position = offset + 4
    actual_operations = Counter()
    for _ in range(op_count):
        if position + 8 > end:
            raise ValueError('Truncated operation header')
        kind, length = struct.unpack_from('<II', data, position)
        if length < 8 or position + length > end:
            raise ValueError('Invalid operation length')
        actual_operations[kind] += 1
        if kind == 1:
            if length < 80 or data[position + 60] != 1:
                raise ValueError('Expected forward kernel launch')
            key_len, hash_len, nparams, _, nrelocs, value_size = struct.unpack_from('<HHHHII', data, position + 64)
            params = position + 80 + key_len + hash_len
            relocs = params + nparams * 16
            values = relocs + nrelocs * 24
            if values + value_size != position + length:
                raise ValueError('Invalid launch layout')
            reloc_index = 0
            for index in range(nparams):
                _, nrefs, value_offset, binding_size, _ = struct.unpack_from('<HHIII', data, params + index * 16)
                if value_offset + binding_size > value_size or reloc_index + nrefs > nrelocs:
                    raise ValueError('Invalid parameter layout')
                for _ in range(nrefs):
                    slot, _, _, reloc_kind = struct.unpack_from('<IiQB', data, relocs + reloc_index * 24)
                    if reloc_kind not in (1, 3) or slot + 8 > binding_size:
                        raise ValueError('Unsupported pointer relocation')
                    at = values + value_offset + slot
                    data[at:at + 8] = b'\0' * 8
                    reloc_index += 1
            if reloc_index != nrelocs:
                raise ValueError('Unclaimed relocation')
        elif kind == 4:
            # APICMemcpyD2DRecord: two region IDs and byte offsets, no raw
            # pointers. The exact packed size is 40 in pinned apic_types.h.
            if length != 40:
                raise ValueError('Invalid region-copy layout')
            dst, src, dst_offset, src_offset, size = struct.unpack_from('<iiQQQ', data, position + 8)
            if min(dst, src) < 0 or max(dst_offset, src_offset) + size > (1 << 64) - 1:
                raise ValueError('Invalid region-copy address')
        elif kind == 5:
            # APICMemsetRecord: region ID, fill value, byte offset and size.
            if length != 32:
                raise ValueError('Invalid region-fill layout')
            region, value, region_offset, size = struct.unpack_from('<iiQQ', data, position + 8)
            if region < 0 or region_offset + size > (1 << 64) - 1 or not 0 <= value <= 255:
                raise ValueError('Invalid region-fill address or value')
        else:
            raise ValueError(f'Unsupported operation for canonical capture: {kind}')
        position += length
    if position != end:
        raise ValueError('Trailing operation bytes')
    if expected_operations is not None and actual_operations != expected_operations:
        raise ValueError(f'Unexpected graph operation mix: {dict(actual_operations)}')
    return bytes(data)


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n'


def artifact_record(path):
    path = Path(path)
    return {'name': path.name, 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def verify_artifacts(manifest, root):
    root = Path(root).resolve()
    names = set()
    for item in manifest['artifacts']:
        name = item['name']
        if not name or '/' in name or '\\' in name or name in {'.', '..'} or name in names:
            raise ValueError('Invalid or duplicate artifact name')
        names.add(name)
        path = root / name
        if path.resolve().parent != root or not path.is_file() or artifact_record(path) != item:
            raise ValueError(f'Artifact integrity failure: {name}')


def write_manifest(metadata, root, binaries):
    root = Path(root)
    manifest = dict(metadata, schema_version=1,
                    artifacts=[artifact_record(root / name) for name in sorted(binaries)])
    verify_artifacts(manifest, root)
    payload = canonical_json(manifest)
    (root / 'artifact_manifest.json').write_text(payload, encoding='utf-8', newline='\n')
    # A raw JSON string keeps the header schema identical to the shipped manifest.
    header = ('#pragma once\n#include <cstddef>\nnamespace quest_newton::generated {\n'
              f'inline constexpr char kArtifactManifest[] = R"manifest({payload})manifest";\n'
              f'inline constexpr std::size_t kJointCount = {manifest["joint_count"]};\n'
              f'inline constexpr std::size_t kBodyCount = {manifest["body_count"]};\n'
              '}\n')
    (root / 'artifact_manifest.h').write_text(header, encoding='utf-8', newline='\n')
    return manifest
