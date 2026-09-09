from pathlib import Path
import subprocess
import tempfile
import unittest
import sys

from tools.newton_codegen.build_android_artifacts import require_commit, validate_output
from tools.newton_codegen.capture_franka import collect_module_sources
from tools.newton_codegen.verify_bundle import parse_elf, validate_elf_closure


class BuildContractTests(unittest.TestCase):
    def test_required_verification_command_fails_without_artifact_inputs(self):
        result = subprocess.run([sys.executable, '-m', 'tools.newton_codegen.verify_bundle'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, 'Production verification must never silently skip')

    def test_same_basename_modules_are_kept_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            modules = []
            for name, content in [('first', b'first kernel'), ('second', b'second kernel')]:
                cache = root / name
                cache.mkdir()
                (cache / 'module.cpp').write_bytes(content)
                modules.append((name, {'module_name': 'same_module', 'binary_path': str(cache / 'module.o')}))
            sources = collect_module_sources(modules, root / 'collected')
            self.assertEqual(len(set(sources)), 2)
            self.assertEqual({(root / 'collected' / name).read_bytes() for name in sources},
                             {b'first kernel', b'second kernel'})
            (root / 'first' / 'another.cpp').write_bytes(b'ambiguous')
            with self.assertRaises(ValueError):
                collect_module_sources(modules, root / 'ambiguous')
            with self.assertRaises(ValueError):
                collect_module_sources([('missing', {'module_name': 'none',
                    'binary_path': str(root / 'missing' / 'module.o')})], root / 'missing_output')

    def test_conflicting_source_for_same_module_identity_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            modules = []
            for name in ('a', 'b'):
                cache = root / name
                cache.mkdir()
                (cache / 'module.cpp').write_text(name)
                modules.append(('same_hash', {'module_name': 'same', 'binary_path': str(cache / 'module.o')}))
            with self.assertRaises(ValueError):
                collect_module_sources(modules, root / 'collected')

    def test_missing_kernel_symbol_dependency_and_wrong_arch_are_rejected(self):
        def elf(soname, needed=(), symbols=()):
            return {'machine': 'AArch64', 'elf_class': 'ELF64', 'soname': soname,
                    'needed': set(needed), 'symbols': set(symbols)}
        records = {'libwarp.so': elf('libwarp.so', ['libc.so']),
                   'kernels.so': elf('kernels.so', ['libwarp.so', 'libc.so'], ['forward'])}
        validate_elf_closure(records, ['kernels.so'], ['forward'])
        with self.assertRaises(ValueError):
            validate_elf_closure(records, ['kernels.so'], ['missing_forward'])
        records['kernels.so']['needed'].add('libmissing.so')
        with self.assertRaises(ValueError):
            validate_elf_closure(records, ['kernels.so'], ['forward'])
        records['kernels.so']['needed'].remove('libmissing.so')
        records['libwarp.so']['machine'] = 'X86-64'
        with self.assertRaises(ValueError):
            validate_elf_closure(records, ['kernels.so'], ['forward'])

    def test_elf_parser_excludes_undefined_and_hidden_symbols(self):
        text = ''' Class: ELF64
 Machine: AArch64
 0x1 (NEEDED) Shared library: [libwarp.so]
 0xe (SONAME) Library soname: [kernels.so]
 1: 00000 0 FUNC GLOBAL DEFAULT UND absent
 2: 00100 8 FUNC GLOBAL DEFAULT 12 forward
 3: 00200 8 FUNC LOCAL HIDDEN 12 hidden
'''
        self.assertEqual(parse_elf(text), {'machine': 'AArch64', 'elf_class': 'ELF64',
                         'soname': 'kernels.so', 'needed': {'libwarp.so'}, 'symbols': {'forward'}})

    def test_reject_wrong_commit_and_dirty_tracked_source(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            subprocess.run(['git', 'init', '-q', str(repo)], check=True)
            (repo / 'source').write_text('clean')
            subprocess.run(['git', '-C', str(repo), 'add', 'source'], check=True)
            subprocess.run(['git', '-C', str(repo), '-c', 'user.name=Test', '-c',
                            'user.email=test@example.invalid', 'commit', '-qm', 'fixture'], check=True)
            commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
            require_commit(repo, commit)
            with self.assertRaises(ValueError):
                require_commit(repo, '0' * 40)
            (repo / 'source').write_text('changed')
            with self.assertRaises(ValueError):
                require_commit(repo, commit)

    def test_output_must_not_overlap_input_checkout(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'source'
            for output in (source, source / 'out', source.parent):
                with self.assertRaises(ValueError):
                    validate_output(output, [source])
            validate_output(Path(directory) / 'bundle', [source])
