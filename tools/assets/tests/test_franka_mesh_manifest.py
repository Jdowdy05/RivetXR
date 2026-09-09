"""Hand-derived geometry and packaging contracts for the Franka converter."""

import hashlib
import importlib
import json
from pathlib import Path
import struct
import tempfile
import unittest


BODY_NAMES = [f"panda_link{i}" for i in range(9)] + [
    "panda_hand", "panda_leftfinger", "panda_rightfinger"]
ASSETS = [f"link{i}" for i in range(8)] + ["hand", "finger"]
IDENTITY = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"


def dae(matrix=IDENTITY, parent=None):
    # Different normal and position index slots catch accidental interleaving.
    node = f'''<node id="node"><matrix>{matrix}</matrix>
      <instance_geometry url="#mesh"/></node>'''
    if parent:
        node = f'<node id="parent"><matrix>{parent}</matrix>{node}</node>'
    return f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
      <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
      <library_geometries><geometry id="mesh"><mesh>
        <source id="positions"><float_array id="positions-array" count="9">1 0 0 0 1 0 0 0 1</float_array>
          <technique_common><accessor source="#positions-array" count="3" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common></source>
        <source id="normals"><float_array id="normals-array" count="9">0 0 1 0 1 0 1 0 0</float_array>
          <technique_common><accessor source="#normals-array" count="3" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common></source>
        <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="1"><input semantic="VERTEX" source="#vertices" offset="0"/>
          <input semantic="NORMAL" source="#normals" offset="1"/><p>2 0 0 1 1 2</p></triangles>
      </mesh></geometry></library_geometries>
      <library_visual_scenes><visual_scene id="Scene">{node}</visual_scene></library_visual_scenes>
      <scene><instance_visual_scene url="#Scene"/></scene></COLLADA>'''


class FrankaMeshTests(unittest.TestCase):
    def setUp(self):
        try:
            self.converter = importlib.import_module("tools.assets.convert_franka_meshes")
        except ModuleNotFoundError:
            self.fail("Franka mesh converter has not been implemented")
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / "source"
        (self.root / "robots").mkdir(parents=True)
        (self.root / "meshes/visual").mkdir(parents=True)
        self.urdf = self.root / "robots/panda_arm_hand.urdf"
        links = []
        for index, name in enumerate(BODY_NAMES):
            if index == 8:
                links.append(f'<link name="{name}"/>')
                continue
            asset = f"link{index}" if index < 8 else "hand" if index == 9 else "finger"
            origin = '<origin xyz="0 0 0" rpy="0 0 3.14159265359"/>' if index == 11 else ""
            links.append(f'<link name="{name}"><visual>{origin}<geometry><mesh '
                         f'filename="package://franka_description/meshes/visual/{asset}.dae"/>'
                         '</geometry></visual></link>')
        self.urdf.write_text('<robot name="panda">' + ''.join(links) + '</robot>', encoding="utf-8")
        for asset in ASSETS:
            (self.root / f"meshes/visual/{asset}.dae").write_text(dae(), encoding="utf-8")
        self.physics = self.base / "artifact_manifest.json"
        self.sync_physics()
        self.output = self.base / "output"

    def sync_physics(self):
        self.physics.write_text(json.dumps({"schema_version": 1, "body_count": 12,
            "body_names": BODY_NAMES, "urdf_sha256": hashlib.sha256(self.urdf.read_bytes()).hexdigest()}))

    def convert(self):
        return self.converter.convert(self.root, self.physics, self.output)

    def mutate_dae(self, old, new):
        path = self.root / "meshes/visual/link0.dae"
        path.write_text(dae().replace(old, new), encoding="utf-8")

    def test_visible_mapping_origins_and_deterministic_bytes(self):
        manifest = self.convert()
        self.assertEqual([x["name"] for x in manifest["bodies"]], BODY_NAMES)
        self.assertEqual([x["name"] for x in manifest["assets"]], [x + ".qmsh" for x in ASSETS])
        self.assertEqual(manifest["body_count"], 12)
        self.assertEqual(manifest["visual_count"], 11)
        self.assertEqual(manifest["total_geometry_bytes"], 620)
        self.assertIsNone(manifest["bodies"][8]["asset"])
        self.assertEqual(manifest["bodies"][10]["asset"], "finger.qmsh")
        self.assertEqual(manifest["bodies"][11]["rotation"], [0, 0, 1, 0])
        for body in manifest["bodies"]:
            self.assertEqual(body["position"], [0, 0, 0])
            self.assertEqual(body["scale"], [1, 1, 1])
        before = {p.name: p.read_bytes() for p in self.output.iterdir()}
        self.convert()
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.output.iterdir()})
        self.assertEqual(self.converter.verify_output(self.output, self.physics), manifest)
        self.assertNotIn(str(self.root), before["franka_meshes.json"].decode())

    def test_authored_matrix_rotation_translation_and_position_slots(self):
        # Rz(+90) followed by translation (2,3,4): (1,0,0) -> (2,4,4).
        path = self.root / "meshes/visual/link0.dae"
        path.write_text(dae("0 -1 0 2 1 0 0 3 0 0 1 4 0 0 0 1"))
        self.convert()
        payload = (self.output / "link0.qmsh").read_bytes()
        self.assertEqual(struct.unpack_from("<4sIIII", payload), (b"QMSH", 1, 0, 3, 3))
        self.assertEqual(struct.unpack_from("<9f", payload, 20), (2, 4, 4, 1, 3, 4, 2, 3, 5))
        self.assertEqual(struct.unpack_from("<3H", payload, 56), (2, 0, 1))

    def test_nested_scene_composes_parent_before_child(self):
        path = self.root / "meshes/visual/link0.dae"
        path.write_text(dae("1 0 0 2 0 1 0 0 0 0 1 0 0 0 0 1",
                            "0 -1 0 0 1 0 0 0 0 0 1 0 0 0 0 1"))
        self.convert()
        self.assertEqual(struct.unpack_from("<3f", (self.output / "link0.qmsh").read_bytes(), 20), (0, 3, 0))

    def test_loose_source_lines_are_validated_but_not_triangle_surfaces(self):
        path = self.root / "meshes/visual/link0.dae"
        lines = '<lines count="1"><input semantic="VERTEX" source="#vertices" offset="0"/><p>0 2</p></lines>'
        path.write_text(dae().replace('</mesh>', lines + '</mesh>'))
        manifest = self.convert()
        self.assertEqual(manifest["assets"][0]["index_count"], 3)
        self.assertEqual(manifest["assets"][0]["discarded_line_segment_count"], 1)
        self.assertEqual(struct.unpack_from("<3H", (self.output / "link0.qmsh").read_bytes(), 56), (2, 0, 1))
        path.write_text(dae().replace('</mesh>', lines.replace('<p>0 2</p>', '<p>0 3</p>') + '</mesh>'))
        with self.assertRaises(ValueError):
            self.convert()

    def test_float32_error_is_measured_and_one_millimetre_gate_is_enforced(self):
        path = self.root / "meshes/visual/link0.dae"
        path.write_text(dae().replace('>1 0 0 0 1 0 0 0 1</float_array>', '>1.1 0 0 0 1 0 0 0 1</float_array>'))
        manifest = self.convert()
        self.assertAlmostEqual(manifest["assets"][0]["max_vertex_error_m"], 2.3841857821338408e-8, places=14)
        path.write_text(dae().replace('>1 0 0 0 1 0 0 0 1</float_array>', '>65536.003 0 0 0 1 0 0 0 1</float_array>'))
        with self.assertRaisesRegex(ValueError, 'one millimetre'):
            self.convert()

    def test_uninstantiated_and_duplicate_geometry_are_rejected(self):
        for replacement in ['', '<instance_geometry url="#mesh"/><instance_geometry url="#mesh"/>']:
            with self.subTest(replacement=replacement):
                self.mutate_dae('<instance_geometry url="#mesh"/>', replacement)
                with self.assertRaises(ValueError):
                    self.convert()

    def test_dae_accessor_and_scene_require_nonempty_ids(self):
        mutations = [
            ('id="positions-array"', 'id=""'),
            ('id="positions-array"', ''),
            ('id="Scene"', ''),
        ]
        for old, new in mutations:
            with self.subTest(old=old, new=new):
                source = dae().replace(old, new)
                if 'positions-array' in old:
                    source = source.replace('source="#positions-array"', 'source="#"')
                else:
                    source = source.replace('url="#Scene"', 'url="#"')
                (self.root / "meshes/visual/link0.dae").write_text(source)
                with self.assertRaises(ValueError):
                    self.convert()

    def test_rejects_unsupported_or_malformed_dae(self):
        mutations = [
            ('meter="1"', 'meter="0.01"'), ('Z_UP', 'Y_UP'),
            ('COLLADASchema', 'wrong-schema'), ('stride="3"', 'stride="4"'),
            ('name="X"', 'name="Q"'), ('source="#positions-array"', 'source="#normals-array"'),
            ('1 0 0 0 1 0 0 0 1</float_array>', 'nan 0 0 0 1 0 0 0 1</float_array>'),
            ('<p>2 0 0 1 1 2</p>', '<p>3 0 0 1 1 2</p>'),
            ('<p>2 0 0 1 1 2</p>', '<p>2 3 0 1 1 2</p>'),
            ('<p>2 0 0 1 1 2</p>', '<p>-1 0 0 1 1 2</p>'),
            ('<p>2 0 0 1 1 2</p>', '<p>2 0 0 1</p>'),
            ('triangles', 'polylist'), ('count="1"', 'count="999999999999"'),
            ('id="normals"', 'id="positions"'), ('url="#mesh"', 'url="#missing"'),
            ('<instance_geometry url="#mesh"/>', '<instance_node url="#node"/>'),
            ('<matrix>' + IDENTITY, '<matrix>1 0 0 0 0 1 0 0 0 0 1 0 1 0 0 1'),
            ('<matrix>' + IDENTITY, '<matrix>-1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1'),
        ]
        for old, new in mutations:
            with self.subTest(old=old, new=new):
                self.mutate_dae(old, new)
                with self.assertRaises(ValueError):
                    self.convert()

    def test_rejects_wrong_urdf_body_mapping_and_transforms(self):
        original = self.urdf.read_text()
        mutations = [
            ('<link name="panda_link8"/>', ''),
            ('<link name="panda_link8"/>', '<link name="panda_link0"/>'),
            ('meshes/visual/link0.dae', 'meshes/visual/../link0.dae'),
            ('meshes/visual/link0.dae', 'meshes/visual/link1.dae'),
            ('3.14159265359', '0'), ('xyz="0 0 0"', 'xyz="nan 0 0"'),
            ('filename="package://franka_description/meshes/visual/link0.dae"',
             'filename="package://franka_description/meshes/visual/link0.dae" scale="2 1 1"'),
        ]
        for old, new in mutations:
            with self.subTest(new=new):
                self.urdf.write_text(original.replace(old, new))
                self.sync_physics()
                with self.assertRaises(ValueError):
                    self.convert()

    def test_rejects_urdf_hash_and_physics_body_order_mismatch(self):
        self.urdf.write_text(self.urdf.read_text() + '\n')
        with self.assertRaises(ValueError):
            self.convert()
        self.sync_physics()
        physics = json.loads(self.physics.read_text())
        physics["body_names"][0:2] = reversed(physics["body_names"][0:2])
        self.physics.write_text(json.dumps(physics))
        with self.assertRaises(ValueError):
            self.convert()

    def test_rejects_output_overlap_and_undeclared_files(self):
        for output in [self.root, self.root / "output", self.base]:
            with self.subTest(output=output), self.assertRaises(ValueError):
                self.converter.convert(self.root, self.physics, output)
        self.convert()
        (self.output / "stray.txt").write_text("unexpected")
        with self.assertRaises(ValueError):
            self.converter.verify_output(self.output, self.physics)
        with self.assertRaises(ValueError):
            self.convert()

    def test_qmsh_rejects_bad_sizes_nonfinite_and_indices(self):
        valid = struct.pack("<4sIIII9f3H", b"QMSH", 1, 0, 3, 3, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 2)
        bad = [valid + b'\x00', valid[:-1], b'BAD!' + valid[4:]]
        for offset, value in [(4, 2), (8, 1), (12, 65536), (16, 0), (16, 4), (16, 3145731)]:
            changed = bytearray(valid)
            struct.pack_into("<I", changed, offset, value)
            bad.append(bytes(changed))
        changed = bytearray(valid)
        struct.pack_into("<f", changed, 20, float("inf"))
        bad.append(bytes(changed))
        bad.append(valid[:-2] + struct.pack("<H", 3))
        for payload in bad:
            with self.subTest(payload=payload[:20]), self.assertRaises(ValueError):
                self.converter.validate_qmsh(payload)

    def test_verifier_rejects_hash_header_and_manifest_tampering(self):
        self.convert()
        mesh = self.output / "link0.qmsh"
        mesh.write_bytes(mesh.read_bytes()[:-2] + b'\x00\x00')
        with self.assertRaises(ValueError):
            self.converter.verify_output(self.output, self.physics)
        self.convert()
        header = self.output / "franka_meshes.h"
        header.write_text("tampered")
        with self.assertRaises(ValueError):
            self.converter.verify_output(self.output, self.physics)
        mutations = [
            lambda m: m["bodies"].__setitem__(1, m["bodies"][0]),
            lambda m: m["bodies"][0].__setitem__("asset", "../link0.qmsh"),
            lambda m: m["bodies"][11].__setitem__("rotation", [0, 0, 0, 1]),
            lambda m: m["bodies"][0].__setitem__("position", [float("nan"), 0, 0]),
            lambda m: m["assets"][0].__setitem__("bounds_min", [-9, -9, -9]),
            lambda m: m["assets"][0].__setitem__("bytes", 0),
            lambda m: m["assets"][0].__setitem__("source", "C:/private/file.dae"),
            lambda m: m["assets"].append(m["assets"][0]),
            lambda m: m.__setitem__("total_geometry_bytes", 12 * 1024 * 1024),
        ]
        for change in mutations:
            with self.subTest(change=change):
                manifest = self.convert()
                change(manifest)
                (self.output / "franka_meshes.json").write_text(json.dumps(manifest))
                with self.assertRaises(ValueError):
                    self.converter.verify_output(self.output, self.physics)


if __name__ == "__main__":
    unittest.main()
