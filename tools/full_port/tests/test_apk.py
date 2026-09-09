"""Negative packaging fixtures; no SDK or device is needed."""
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import warnings
import zipfile

from tools.full_port.verify_apk import Payloads, match_native, verify_elf


def archive(files):
    out = io.BytesIO()
    with zipfile.ZipFile(out, 'w') as zipped:
        for name, data in files:
            zipped.writestr(name, data)
    out.seek(0)
    return out


class ApkPayloadTests(unittest.TestCase):
    def test_nested_versioned_dependency_is_inspected(self):
        nested = archive([('chaquopy/lib/libgfortran.so.3', b'\x7fELFfortran'),
                          ('numpy/core/_array.so', b'\x7fELFnumpy')])
        payloads = Payloads(archive([('assets/chaquopy/requirements.imy', nested.getvalue())]))
        def readelf(*args):
            data = Path(args[-1]).read_bytes()
            return 'Class: ELF64\nMachine: AArch64\n' + (
                '(NEEDED) Shared library: [libgfortran.so.3]\n' if data.endswith(b'numpy') else '')
        try:
            with tempfile.TemporaryDirectory() as temporary, patch('tools.full_port.verify_apk.run', readelf):
                records = verify_elf(payloads, Path('readelf'), Path(temporary))
            self.assertEqual(len(records), 2)
        finally:
            payloads.close()

    def test_missing_dependency_inside_imy_fails(self):
        payloads = Payloads(archive([('assets/chaquopy/app.imy',
                                     archive([('mujoco/_functions.so', b'\x7fELF')]).getvalue())]))
        try:
            with tempfile.TemporaryDirectory() as temporary, patch('tools.full_port.verify_apk.run',
                    return_value='Class: ELF64\nMachine: AArch64\n(NEEDED) [libmissing.so.3]\n'):
                with self.assertRaisesRegex(ValueError, 'Unpackaged ELF dependencies'):
                    verify_elf(payloads, Path('readelf'), Path(temporary))
        finally:
            payloads.close()

    def test_desktop_elf_inside_imy_fails(self):
        payloads = Payloads(archive([('assets/chaquopy/app.imy',
                                     archive([('warp/bin/hidden', b'\x7fELF')]).getvalue())]))
        try:
            with tempfile.TemporaryDirectory() as temporary, patch('tools.full_port.verify_apk.run',
                    return_value='Class: ELF64\nMachine: Advanced Micro Devices X86-64\n'):
                with self.assertRaisesRegex(ValueError, 'Non-ARM64 ELF'):
                    verify_elf(payloads, Path('readelf'), Path(temporary))
        finally:
            payloads.close()

    def test_duplicate_nested_member_fails(self):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            nested = archive([('newton/__init__.py', b'a'), ('newton/__init__.py', b'b')])
        with self.assertRaisesRegex(ValueError, 'Duplicate ZIP member'):
            Payloads(archive([('assets/chaquopy/app.imy', nested.getvalue())]))

    def test_nested_traversal_fails(self):
        nested = archive([('../outside.py', b'bad')])
        with self.assertRaisesRegex(ValueError, 'Unsafe archive'):
            Payloads(archive([('assets/chaquopy/app.imy', nested.getvalue())]))

    def test_hash_requires_exact_or_reproducible_strip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source.so'
            source.write_bytes(b'original with debug')
            def strip(*args):
                Path(args[-2]).write_bytes(b'reproducible stripped')
            with patch('tools.full_port.verify_apk.run', strip):
                self.assertEqual(match_native(b'original with debug', source, Path('strip'), root, 'test'), 'exact')
                self.assertEqual(match_native(b'reproducible stripped', source, Path('strip'), root, 'test'), 'ndk-strip-unneeded')
                with self.assertRaisesRegex(ValueError, 'native hash mismatch'):
                    match_native(b'tampered', source, Path('strip'), root, 'test')


if __name__ == '__main__':
    unittest.main()
