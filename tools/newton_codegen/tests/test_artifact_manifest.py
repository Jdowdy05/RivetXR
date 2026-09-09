from pathlib import Path
import tempfile
import struct
import unittest

from tools.newton_codegen.artifact_manifest import canonical_json, artifact_record, verify_artifacts
from tools.newton_codegen.artifact_manifest import canonicalize_graph


class ManifestTests(unittest.TestCase):
    @staticmethod
    def graph_with_operation(operation):
        header = bytearray(64)
        struct.pack_into('<4sIIIQ', header, 0, b'WRP1', 15, 0, 1, 64)
        header[24] = 1
        ops = struct.pack('<I', 1) + operation
        return bytes(header) + struct.pack('<IIQQQ', 3, 0, 96, len(ops), len(ops)) + ops

    def test_opaque_operation_kinds_and_wrong_fixed_layouts_rejected(self):
        for kind in (2, 6, 10, 7, 99):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                canonicalize_graph(self.graph_with_operation(struct.pack('<II', kind, 8)))
        for kind, length in ((4, 8), (4, 41), (5, 8), (5, 33)):
            operation = struct.pack('<II', kind, length) + bytes(length - 8)
            with self.subTest(kind=kind, length=length), self.assertRaises(ValueError):
                canonicalize_graph(self.graph_with_operation(operation))

    def test_region_copy_and_fill_preserve_offsets_and_validate_operation_mix(self):
        for operation, kind in [(struct.pack('<IIiiQQQ', 4, 40, 1, 2, 8, 16, 36), 4),
                                (struct.pack('<IIiiQQ', 5, 32, 1, 0, 8, 36), 5)]:
            payload = self.graph_with_operation(operation)
            self.assertEqual(canonicalize_graph(payload, expected_operations={kind: 1}), payload)
            with self.assertRaises(ValueError):
                canonicalize_graph(payload, expected_operations={kind: 2})
        invalid = struct.pack('<IIiiQQQ', 4, 40, -1, 2, 0, 0, 36)
        with self.assertRaises(ValueError):
            canonicalize_graph(self.graph_with_operation(invalid))

    def test_canonical_graph_zeros_only_relocated_pointer_bytes(self):
        # Minimal format-15 CPU launch: one 16-byte value (pointer + scalar).
        header = bytearray(64)
        struct.pack_into('<4sIIIQ', header, 0, b'WRP1', 15, 0, 1, 64)
        header[24] = 1
        launch = bytearray(80)
        struct.pack_into('<II', launch, 0, 1, 80 + 16 + 24 + 16)
        launch[60] = 1
        struct.pack_into('<HHHHII', launch, 64, 0, 0, 1, 0, 1, 16)
        parameter = struct.pack('<HHIII', 1, 1, 0, 16, 8)
        relocation = struct.pack('<IiQB7x', 0, 1, 0, 1)
        values = struct.pack('<QQ', 0x1234567812345678, 42)
        ops = struct.pack('<I', 1) + launch + parameter + relocation + values
        fixture = bytes(header) + struct.pack('<IIQQQ', 3, 0, 96, len(ops), len(ops)) + ops
        actual = canonicalize_graph(fixture)
        self.assertEqual(actual[:-16], fixture[:-16])
        self.assertEqual(actual[-16:], struct.pack('<QQ', 0, 42))
        self.assertEqual(canonicalize_graph(actual), actual)
        with self.assertRaises(ValueError):
            canonicalize_graph(fixture[:-1])
        with self.assertRaises(ValueError):
            canonicalize_graph(b'not a graph')

    def test_canonical_json_orders_nested_keys_and_rejects_nan(self):
        self.assertEqual(canonical_json({'z': 1, 'a': {'y': 2, 'b': 3}}),
                         '{"a":{"b":3,"y":2},"z":1}\n')
        with self.assertRaises(ValueError):
            canonical_json({'value': float('nan')})

    def test_sha256_length_and_tamper_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / 'test.wrp'
            artifact.write_bytes(b'abc')
            record = artifact_record(artifact)
            self.assertEqual(record, {'name': 'test.wrp', 'bytes': 3, 'sha256':
                'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'})
            manifest = {'artifacts': [record]}
            verify_artifacts(manifest, root)
            artifact.write_bytes(b'abd')
            with self.assertRaises(ValueError):
                verify_artifacts(manifest, root)

    def test_rejects_artifact_path_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                verify_artifacts({'artifacts': [{'name': '../escape', 'bytes': 0,
                    'sha256': '0' * 64}]}, Path(directory))
