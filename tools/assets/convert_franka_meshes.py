"""Deterministically bake the selected Franka DAE visuals into QMSH v1.

The intentionally narrow COLLADA reader supports the audited 1.4.1, metre/Z_UP,
rigid matrix scene, indexed-triangle source profile. Validated loose line segments
are counted and omitted from triangle surfaces. It refuses other profiles instead
of silently dropping geometry. Materials and normals are not emitted.
The physics bundle must already have passed its separate real-bundle verifier.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


NS = "{http://www.collada.org/2005/11/COLLADASchema}"
MAX_XML_BYTES = 32 * 1024 * 1024
MAX_VERTICES = 65535
MAX_INDICES = 3145728
MAX_GEOMETRY_BYTES = 12 * 1024 * 1024
MAX_VERTEX_ERROR_M = 0.001
IDENTITY = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)
BODY_NAMES = [f"panda_link{i}" for i in range(9)] + [
    "panda_hand", "panda_leftfinger", "panda_rightfinger"]
ASSET_NAMES = [f"link{i}.qmsh" for i in range(8)] + ["hand.qmsh", "finger.qmsh"]


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _hash(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _integer(value, low, high, label):
    _require(type(value) is int and low <= value <= high, f"invalid {label}")
    return value


def _xml_integer(value, low, high, label):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9]+", value) is not None,
             f"invalid {label}")
    _require(len(value) <= 12, f"impossible {label}")
    return _integer(int(value), low, high, label)


def _vector(value, count, label):
    _require(isinstance(value, (list, tuple)) and len(value) == count, f"invalid {label}")
    _require(all(type(x) in (int, float) and math.isfinite(x) for x in value), f"nonfinite {label}")
    return value


def _floats(text, count, label):
    values = (text or "").split()
    _require(len(values) == count, f"invalid {label} length")
    try:
        return _vector([float(x) for x in values], count, label)
    except (OverflowError, TypeError) as exc:
        raise ValueError(f"invalid {label}") from exc


def _read(path, maximum):
    _require(path.is_file() and not path.is_symlink(), "expected a regular input file")
    _require(path.stat().st_size <= maximum, "input exceeds size limit")
    with path.open("rb") as stream:
        data = stream.read(maximum + 1)
    _require(len(data) <= maximum, "input exceeds size limit")
    return data


def _json(data):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            _require(key not in result, "duplicate JSON key")
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f"nonfinite JSON constant {value}")

    try:
        return json.loads(data, object_pairs_hook=unique, parse_constant=reject_constant)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid JSON") from exc


def _contract():
    return _json(_read(Path(__file__).resolve().parents[2] / "config/franka_meshes.json", 65536))


def _physics(path):
    manifest = _json(_read(Path(path), 1024 * 1024))
    _require(isinstance(manifest, dict), "invalid physics manifest")
    _require(type(manifest.get("schema_version")) is int and manifest["schema_version"] == 1,
             "unsupported physics manifest")
    _require(type(manifest.get("body_count")) is int and manifest["body_count"] == 12
             and manifest.get("body_names") == BODY_NAMES, "physics body order mismatch")
    _require(_hash(manifest.get("urdf_sha256")), "invalid physics URDF hash")
    return manifest


def _xml(data):
    _require(b"<!DOCTYPE" not in data.upper() and b"<!ENTITY" not in data.upper(),
             "XML declarations/entities are unsupported")
    try:
        return ET.fromstring(data)
    except ET.ParseError as exc:
        raise ValueError("malformed XML") from exc


def _one(element, tag):
    children = element.findall(tag)
    _require(len(children) == 1, f"expected one {tag}")
    return children[0]


def _children(element, allowed):
    _require(all(child.tag in allowed for child in element), "unsupported XML child")


def _local_ref(value, mapping):
    _require(isinstance(value, str) and len(value) > 1 and value.startswith("#") and value[1:] in mapping,
             "unresolved or external DAE reference")
    return mapping[value[1:]]


def _matrix_product(a, b):
    return tuple(sum(a[row * 4 + k] * b[k * 4 + col] for k in range(4))
                 for row in range(4) for col in range(4))


def _rigid_matrix(element):
    matrix = _floats(element.text, 16, "DAE matrix")
    _require(matrix[12:] == [0., 0., 0., 1.], "nonaffine DAE matrix")
    rows = [matrix[i:i + 3] for i in (0, 4, 8)]
    for i in range(3):
        for j in range(3):
            dot = sum(rows[i][k] * rows[j][k] for k in range(3))
            _require(abs(dot - (1 if i == j else 0)) <= 1e-5, "nonrigid DAE matrix")
    a, b, c = rows
    determinant = (a[0] * (b[1]*c[2] - b[2]*c[1]) - a[1] * (b[0]*c[2] - b[2]*c[0])
                   + a[2] * (b[0]*c[1] - b[1]*c[0]))
    _require(abs(determinant - 1) <= 1e-5, "reflected DAE matrix")
    return matrix


def _source(source):
    _children(source, {NS + "float_array", NS + "technique_common"})
    array = _one(source, NS + "float_array")
    accessor = _one(_one(source, NS + "technique_common"), NS + "accessor")
    _require(accessor.get("source") == "#" + array.get("id", ""), "source accessor array mismatch")
    _require(accessor.get("stride") == "3" and accessor.get("offset", "0") == "0",
             "unsupported source accessor layout")
    _children(accessor, {NS + "param"})
    _require([(p.get("name"), p.get("type")) for p in accessor] ==
             [(axis, "float") for axis in ("X", "Y", "Z")], "unsupported accessor parameters")
    count = _xml_integer(accessor.get("count"), 1, MAX_INDICES, "accessor count")
    _require(_xml_integer(array.get("count"), 3, MAX_INDICES * 3, "array count") == count * 3,
             "source count mismatch")
    data = _floats(array.text, count * 3, "source values")
    return [tuple(data[i:i + 3]) for i in range(0, len(data), 3)]


def _geometry(geometry):
    _children(geometry, {NS + "mesh"})
    mesh = _one(geometry, NS + "mesh")
    _children(mesh, {NS + "source", NS + "vertices", NS + "triangles", NS + "lines"})
    sources = {source.get("id"): _source(source) for source in mesh.findall(NS + "source")}
    vertices = _one(mesh, NS + "vertices")
    _children(vertices, {NS + "input"})
    position_input = _one(vertices, NS + "input")
    _require(position_input.get("semantic") == "POSITION", "unsupported vertices input")
    positions = _local_ref(position_input.get("source"), sources)
    _require(3 <= len(positions) <= MAX_VERTICES, "position count exceeds vertex limit")
    indices = []
    triangles = mesh.findall(NS + "triangles")
    _require(bool(triangles), "missing triangle geometry")
    for triangle in triangles:
        _children(triangle, {NS + "input", NS + "p"})
        inputs = triangle.findall(NS + "input")
        _require([(x.get("semantic"), x.get("offset")) for x in inputs] ==
                 [("VERTEX", "0"), ("NORMAL", "1")], "unsupported triangle inputs")
        _require(inputs[0].get("source") == "#" + vertices.get("id", ""), "vertex source mismatch")
        normals = _local_ref(inputs[1].get("source"), sources)
        count = _xml_integer(triangle.get("count"), 1, MAX_INDICES // 3, "triangle count")
        _require(len(indices) + 3 * count <= MAX_INDICES, "triangle index limit exceeded")
        tokens = (_one(triangle, NS + "p").text or "").split()
        _require(len(tokens) == count * 6, "triangle index count mismatch")
        for i in range(0, len(tokens), 2):
            indices.append(_xml_integer(tokens[i], 0, len(positions) - 1, "position index"))
            _xml_integer(tokens[i + 1], 0, len(normals) - 1, "normal index")
    line_count = 0
    for lines in mesh.findall(NS + "lines"):
        _children(lines, {NS + "input", NS + "p"})
        line_input = _one(lines, NS + "input")
        _require(line_input.get("semantic") == "VERTEX" and line_input.get("offset") == "0"
                 and line_input.get("source") == "#" + vertices.get("id", ""), "unsupported line inputs")
        count = _xml_integer(lines.get("count"), 1, MAX_INDICES // 2, "line count")
        tokens = (_one(lines, NS + "p").text or "").split()
        _require(len(tokens) == count * 2, "line index count mismatch")
        for token in tokens:
            _xml_integer(token, 0, len(positions) - 1, "line position index")
        line_count += count
        _require(line_count <= MAX_INDICES // 2, "line count exceeds limit")
    return positions, indices, line_count


def convert_dae(data):
    """Return QMSH bytes and geometry metadata from audited-profile DAE bytes."""
    _require(len(data) <= MAX_XML_BYTES, "DAE exceeds size limit")
    root = _xml(data)
    _require(root.tag == NS + "COLLADA" and root.get("version") == "1.4.1", "unsupported DAE schema")
    _require(all(isinstance(e.tag, str) and e.tag.startswith(NS) for e in root.iter()),
             "mixed DAE namespaces")
    ids = {}
    for element in root.iter():
        if element.tag in {NS + tag for tag in ("source", "float_array", "vertices", "geometry", "visual_scene")}:
            _require(bool(element.get("id")), "missing required DAE id")
        if "id" in element.attrib:
            key = element.get("id")
            _require(bool(key) and key not in ids, "duplicate or empty DAE id")
            ids[key] = element
    asset = _one(root, NS + "asset")
    unit = _one(asset, NS + "unit")
    _require(unit.get("meter") == "1" and (_one(asset, NS + "up_axis").text or "").strip() == "Z_UP",
             "unsupported DAE units or up axis")
    _children(root, {NS + tag for tag in ("asset", "library_images", "library_effects", "library_materials",
                                         "library_geometries", "library_controllers", "library_visual_scenes", "scene")})
    for tag in ("library_images", "library_controllers"):
        _require(all(len(library) == 0 for library in root.findall(NS + tag)), "unsupported images/controllers")
    library = _one(root, NS + "library_geometries")
    _children(library, {NS + "geometry"})
    geometries = {g.get("id"): _geometry(g) for g in library}
    _require(bool(geometries) and None not in geometries, "missing geometry id")
    scene_library = _one(root, NS + "library_visual_scenes")
    _children(scene_library, {NS + "visual_scene"})
    scene = _one(scene_library, NS + "visual_scene")
    _children(scene, {NS + "node"})
    scene_binding = _one(root, NS + "scene")
    _children(scene_binding, {NS + "instance_visual_scene"})
    _require(_one(scene_binding, NS + "instance_visual_scene").get("url") == "#" + scene.get("id", ""),
             "visual scene binding mismatch")
    positions, indices, seen = [], [], set()
    discarded_lines = 0

    def visit(node, parent, depth):
        nonlocal discarded_lines
        _require(depth <= 64 and node.get("type", "NODE") == "NODE", "unsupported scene hierarchy")
        _children(node, {NS + "matrix", NS + "node", NS + "instance_geometry"})
        matrix = _matrix_product(parent, _rigid_matrix(_one(node, NS + "matrix")))
        # Only matrix-first node layout is supported; transformed instances retain document order.
        _require(len(node) > 1 and node[0].tag == NS + "matrix", "unsupported node transform ordering")
        for child in node:
            if child.tag == NS + "node":
                visit(child, matrix, depth + 1)
            elif child.tag == NS + "instance_geometry":
                geometry = _local_ref(child.get("url"), geometries)
                key = child.get("url")[1:]
                _require(key not in seen, "duplicate geometry instance")
                seen.add(key)
                _children(child, {NS + "bind_material"})
                vertices, triangles, line_count = geometry
                discarded_lines += line_count
                _require(discarded_lines <= MAX_INDICES // 2, "combined line count exceeds limit")
                base = len(positions)
                _require(base + len(vertices) <= MAX_VERTICES and len(indices) + len(triangles) <= MAX_INDICES,
                         "combined geometry exceeds size limits")
                for point in vertices:
                    positions.append(tuple(sum(matrix[row * 4 + k] * point[k] for k in range(3))
                                           + matrix[row * 4 + 3] for row in range(3)))
                indices.extend(base + index for index in triangles)

    for node in scene:
        visit(node, IDENTITY, 0)
    _require(seen == set(geometries), "uninstantiated geometry")
    payload = bytearray(struct.pack("<4sIIII", b"QMSH", 1, 0, len(positions), len(indices)))
    max_error = 0.
    for point in positions:
        _vector(point, 3, "transformed position")
        try:
            packed = struct.pack("<3f", *point)
        except (OverflowError, struct.error) as exc:
            raise ValueError("position outside float32 range") from exc
        rounded = struct.unpack("<3f", packed)
        _vector(rounded, 3, "float32 position")
        max_error = max(max_error, math.dist(point, rounded))
        payload.extend(packed)
    _require(max_error <= MAX_VERTEX_ERROR_M, "float32 vertex error exceeds one millimetre")
    payload.extend(struct.pack(f"<{len(indices)}H", *indices))
    metadata = validate_qmsh(payload)
    metadata.update(max_vertex_error_m=max_error, dae_sha256=_sha(data),
                    discarded_line_segment_count=discarded_lines)
    return bytes(payload), metadata


def validate_qmsh(payload):
    """Validate an entire QMSH file before exposing its counts and actual bounds."""
    _require(20 <= len(payload) < MAX_GEOMETRY_BYTES, "invalid QMSH size")
    magic, version, flags, vertices, indices = struct.unpack_from("<4sIIII", payload)
    _require((magic, version, flags) == (b"QMSH", 1, 0), "unsupported QMSH header")
    _require(3 <= vertices <= MAX_VERTICES and 3 <= indices <= MAX_INDICES and indices % 3 == 0,
             "invalid QMSH counts")
    index_offset = 20 + 12 * vertices
    _require(len(payload) == index_offset + 2 * indices, "QMSH byte count mismatch")
    bounds_min, bounds_max = [math.inf] * 3, [-math.inf] * 3
    for position in struct.iter_unpack("<3f", memoryview(payload)[20:index_offset]):
        _vector(position, 3, "QMSH position")
        for axis in range(3):
            bounds_min[axis] = min(bounds_min[axis], position[axis])
            bounds_max[axis] = max(bounds_max[axis], position[axis])
    _require(all(index[0] < vertices for index in struct.iter_unpack("<H", memoryview(payload)[index_offset:])),
             "QMSH index out of range")
    return {"bytes": len(payload), "sha256": _sha(payload), "vertex_count": vertices,
            "index_count": indices, "bounds_min": bounds_min, "bounds_max": bounds_max}


def _urdf_bodies(data, contract):
    robot = _xml(data)
    _require(robot.tag == "robot" and robot.get("name") == "panda", "unexpected URDF robot")
    links = robot.findall("link")
    _require([link.get("name") for link in links] == BODY_NAMES, "URDF body order mismatch")
    bodies = []
    for link, expected in zip(links, contract["bodies"]):
        visuals = link.findall("visual")
        if expected["asset"] is None:
            _require(len(link) == 0, "link8 must be empty")
        else:
            _require(len(visuals) == 1, "expected one visual per visible body")
            visual = visuals[0]
            _children(visual, {"origin", "geometry", "material"})
            geometry = _one(visual, "geometry")
            _children(geometry, {"mesh"})
            mesh = _one(geometry, "mesh")
            source = f'meshes/visual/{Path(expected["asset"]).stem}.dae'
            _require(mesh.get("filename") == "package://franka_description/" + source, "unexpected visual asset path")
            _require(_floats(mesh.get("scale", "1 1 1"), 3, "URDF scale") == expected["scale"],
                     "unsupported URDF visual scale")
            origins = visual.findall("origin")
            _require(len(origins) <= 1, "duplicate visual origin")
            origin = origins[0].attrib if origins else {}
            position = _floats(origin.get("xyz", "0 0 0"), 3, "URDF position")
            rpy = _floats(origin.get("rpy", "0 0 0"), 3, "URDF rotation")
            expected_rpy = [0, 0, math.pi if link.get("name") == "panda_rightfinger" else 0]
            _require(position == expected["position"] and all(abs(a-b) < 1e-10 for a, b in zip(rpy, expected_rpy)),
                     "unexpected URDF visual origin")
        # Exact canonical Rz(pi) avoids residual roundoff from authored pi text.
        bodies.append(dict(expected))
    return bodies


def _header(manifest, manifest_bytes):
    def vector(values):
        literals = []
        for value in values:
            literal = format(float(value), ".9g")
            if "." not in literal and "e" not in literal.lower():
                literal += ".0"
            literals.append(literal + "F")
        return "{" + ", ".join(literals) + "}"

    lines = ["// Generated by tools.assets.convert_franka_meshes; do not edit.", "#pragma once",
             "#include <array>", "#include <cstdint>", "#include <cstddef>",
             "namespace quest_newton::generated_meshes {",
             "struct MeshAsset { const char* name; std::uint32_t vertex_count, index_count;",
             "  std::array<float, 3> bounds_min, bounds_max; std::uint64_t byte_count; };",
             "struct MeshBody { const char* name; int asset_index;",
             "  std::array<float, 3> position; std::array<float, 4> rotation; std::array<float, 3> scale; };",
             f'inline constexpr std::array<MeshAsset, {len(manifest["assets"])}> kMeshAssets{{{{']
    for asset in manifest["assets"]:
        lines.append(f'  {{"{asset["name"]}", {asset["vertex_count"]}, {asset["index_count"]}, '
                     f'{vector(asset["bounds_min"])}, {vector(asset["bounds_max"])}, {asset["bytes"]}}},')
    lines.extend(["}};", f'inline constexpr std::array<MeshBody, {len(manifest["bodies"])}> kMeshBodies{{{{'])
    names = [asset["name"] for asset in manifest["assets"]]
    for body in manifest["bodies"]:
        index = names.index(body["asset"]) if body["asset"] is not None else -1
        lines.append(f'  {{"{body["name"]}", {index}, {vector(body["position"])}, '
                     f'{vector(body["rotation"])}, {vector(body["scale"])}}},')
    lines.extend(["}};", f'inline constexpr std::size_t kVisualBodyCount = {manifest["visual_count"]};',
                  f'inline constexpr char kManifestSha256[] = "{_sha(manifest_bytes)}";', "}", ""])
    return "\n".join(lines).encode("utf-8")


def _file_allowlist(output, names, *, complete):
    if not output.exists():
        _require(not complete, "mesh output directory missing")
        return
    _require(output.is_dir() and not output.is_symlink(), "invalid output directory")
    found = set()
    for path in output.iterdir():
        _require(path.name in names and path.is_file() and not path.is_symlink()
                 and path.resolve().parent == output.resolve(), "unexpected or unsafe output file")
        found.add(path.name)
    _require(not complete or found == names, "missing mesh output file")


def verify_output(output_path, artifact_manifest_path):
    """Validate exact package files, metadata, body mappings, and physics identity.

    Keep the generated header here during verification; packaging should copy
    only the JSON and QMSH files after this function succeeds.
    """
    output = Path(output_path)
    physics = _physics(artifact_manifest_path)
    contract = _contract()
    names = set(ASSET_NAMES) | {"franka_meshes.json", "franka_meshes.h"}
    _file_allowlist(output, names, complete=True)
    manifest_bytes = _read(output / "franka_meshes.json", 1024 * 1024)
    manifest = _json(manifest_bytes)
    _require(isinstance(manifest, dict) and set(manifest) == {
        "schema_version", "body_count", "visual_count", "urdf_sha256", "total_geometry_bytes", "assets", "bodies"},
        "invalid mesh manifest schema")
    for field, expected in [("schema_version", 1), ("body_count", 12), ("visual_count", 11)]:
        _require(type(manifest[field]) is int and manifest[field] == expected, f"invalid {field}")
    _require(manifest["urdf_sha256"] == physics["urdf_sha256"], "mesh/physics URDF hash mismatch")
    _require(isinstance(manifest["bodies"], list) and len(manifest["bodies"]) == 12, "invalid body list")
    for body, expected in zip(manifest["bodies"], contract["bodies"]):
        _require(isinstance(body, dict) and set(body) == set(expected), "invalid mesh body schema")
        for key, length in [("position", 3), ("rotation", 4), ("scale", 3)]:
            _vector(body[key], length, key)
        _require(body == expected, "body visual mapping mismatch")
    assets = manifest["assets"]
    _require(isinstance(assets, list) and len(assets) == len(ASSET_NAMES), "invalid assets list")
    total = 0
    for asset, name in zip(assets, ASSET_NAMES):
        keys = {"name", "bytes", "sha256", "vertex_count", "index_count", "bounds_min", "bounds_max",
                "max_vertex_error_m", "dae_sha256", "source", "discarded_line_segment_count"}
        _require(isinstance(asset, dict) and set(asset) in (keys, keys - {"source"}), "invalid asset schema")
        _require(asset["name"] == name and _hash(asset["sha256"]) and _hash(asset["dae_sha256"]), "invalid asset identity")
        if "source" in asset:
            _require(asset["source"] == f"meshes/visual/{Path(name).stem}.dae", "invalid relative asset source")
        _integer(asset["bytes"], 20, MAX_GEOMETRY_BYTES - 1, "asset bytes")
        _integer(asset["vertex_count"], 3, MAX_VERTICES, "vertex count")
        _integer(asset["index_count"], 3, MAX_INDICES, "index count")
        _integer(asset["discarded_line_segment_count"], 0, MAX_INDICES // 2, "discarded line count")
        _vector(asset["bounds_min"], 3, "bounds min")
        _vector(asset["bounds_max"], 3, "bounds max")
        _vector([asset["max_vertex_error_m"]], 1, "vertex error")
        _require(0 <= asset["max_vertex_error_m"] <= MAX_VERTEX_ERROR_M, "vertex error exceeds limit")
        actual = validate_qmsh(_read(output / name, MAX_GEOMETRY_BYTES - 1))
        _require(all(asset[key] == value for key, value in actual.items()), "asset metadata/hash mismatch")
        total += actual["bytes"]
    _integer(manifest["total_geometry_bytes"], 1, MAX_GEOMETRY_BYTES - 1, "total geometry bytes")
    _require(total == manifest["total_geometry_bytes"] and total < MAX_GEOMETRY_BYTES, "total geometry mismatch")
    _require(_read(output / "franka_meshes.h", 1024 * 1024) == _header(manifest, manifest_bytes),
             "generated header does not match manifest")
    return manifest


def convert(franka_description_root, artifact_manifest_path, output_path):
    """Convert without mutating any source; return the verified generated manifest."""
    root = Path(franka_description_root).resolve(strict=True)
    output = Path(output_path).resolve()
    physics_path = Path(artifact_manifest_path).resolve(strict=True)
    _require(root.is_dir() and not (output == root or output in root.parents or root in output.parents),
             "output must not overlap Franka source")
    _require(physics_path != output and output not in physics_path.parents, "output overlaps physics input")
    _file_allowlist(output, set(ASSET_NAMES) | {"franka_meshes.json", "franka_meshes.h"}, complete=False)
    contract, physics = _contract(), _physics(physics_path)

    def source(relative):
        path = root / relative
        _require(root in path.resolve(strict=True).parents, "source path escapes description root")
        return _read(path, MAX_XML_BYTES)

    urdf = source(contract["urdf_relative_path"])
    _require(_sha(urdf) == physics["urdf_sha256"], "URDF hash does not match physics artifact")
    bodies = _urdf_bodies(urdf, contract)
    files, assets, total = {}, [], 0
    for name in ASSET_NAMES:
        relative = f"meshes/visual/{Path(name).stem}.dae"
        payload, metadata = convert_dae(source(relative))
        total += len(payload)
        _require(total < MAX_GEOMETRY_BYTES, "total geometry exceeds limit")
        assets.append(dict(name=name, **metadata, source=relative))
        files[name] = payload
    manifest = {"schema_version": 1, "body_count": 12, "visual_count": 11,
                "urdf_sha256": physics["urdf_sha256"], "total_geometry_bytes": total,
                "assets": assets, "bodies": bodies}
    manifest_bytes = (json.dumps(manifest, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")
    files["franka_meshes.json"] = manifest_bytes
    files["franka_meshes.h"] = _header(manifest, manifest_bytes)
    output.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (output / name).write_bytes(data)
    return verify_output(output, physics_path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--franka-description-root", type=Path, required=True)
    parser.add_argument("--artifact-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = convert(args.franka_description_root, args.artifact_manifest, args.output)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Franka conversion failed: {exc}\n")
    print(json.dumps({"asset_count": len(result["assets"]), "body_count": result["body_count"],
                      "visual_count": result["visual_count"], "total_geometry_bytes": result["total_geometry_bytes"],
                      "max_vertex_error_m": max(a["max_vertex_error_m"] for a in result["assets"]),
                      "manifest_sha256": _sha((args.output / "franka_meshes.json").read_bytes())}, sort_keys=True))


if __name__ == "__main__":
    main()
