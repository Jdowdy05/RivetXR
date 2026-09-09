"""Measure real captured Newton body poses against the production mesh renderer.

Run with the pinned Newton host Python and the operator's pinned-source
PYTHONPATH. No dependency installation, device connection, or actuation occurs.
--output is an ignored directory receiving visual_parity.bin and a JSON report.

Fixture QVPF v1 is little endian with no padding/trailing data: <4sIII64s>
holds magic QVPF, version 1, body_count 12, visual_count 11, and the 64 ASCII
hex bytes of the generated mesh-manifest SHA256. This is followed by <84f>
raw px,py,pz,qx,qy,qz,qw body values and <192d> independent expected row-major
robot_base-from-visual matrices, with column-vector transform convention.
The empty panda_link8 uses an identity body-from-visual transform.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

from tools.assets.convert_franka_meshes import verify_output
from tools.newton_codegen.artifact_manifest import (
    NEWTON_COMMIT, NEWTON_VERSION, WARP_COMMIT, WARP_VERSION, verify_artifacts,
)
from tools.newton_codegen.capture_franka import build_model, load_config


BODY_NAMES = [f"panda_link{i}" for i in range(9)] + [
    "panda_hand", "panda_leftfinger", "panda_rightfinger"]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def vector(text, count, label):
    values = [float(value) for value in text.split()]
    require(len(values) == count and all(math.isfinite(value) for value in values),
            f"invalid {label}")
    return values


def independent_visuals(description_root, mesh_manifest):
    """Parse URDF visual matrices without any converter transform helpers."""
    import numpy as np

    tree = ET.parse(description_root / "robots/panda_arm_hand.urdf").getroot()
    links = tree.findall("link")
    names = [link.get("name") for link in links]
    require(len(names) == len(set(names)) and set(names) == set(BODY_NAMES),
            "URDF body names do not match the physics contract")
    by_name = {link.get("name"): link for link in links}
    source_hashes = {}
    matrices = []
    visuals = 0
    for body_name, mesh_body in zip(BODY_NAMES, mesh_manifest["bodies"]):
        elements = by_name[body_name].findall("visual")
        transform = np.eye(4, dtype=np.float64)
        if body_name == "panda_link8":
            require(not elements and mesh_body["asset"] is None, "link8 must have no visual")
        else:
            require(len(elements) == 1, f"expected exactly one visual for {body_name}")
            visual = elements[0]
            origin = visual.find("origin")
            xyz = vector(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3, "visual translation")
            roll, pitch, yaw = vector(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3, "visual rotation")
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
            ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
            rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
            geometry = visual.find("geometry")
            require(geometry is not None and len(geometry) == 1 and geometry[0].tag == "mesh",
                    f"expected mesh geometry for {body_name}")
            mesh = geometry[0]
            scale = vector(mesh.get("scale", "1 1 1"), 3, "visual scale")
            require(all(value > 0 for value in scale), "visual scales must be positive")
            prefix = "package://franka_description/"
            filename = mesh.get("filename", "")
            require(filename.startswith(prefix), "unexpected mesh package")
            relative = filename[len(prefix):]
            source = (description_root / relative).resolve(strict=True)
            require(source.is_relative_to(description_root), "mesh source escapes description root")
            matching_assets = [asset for asset in mesh_manifest["assets"] if asset["name"] == mesh_body["asset"]]
            require(len(matching_assets) == 1, "visual mesh binding is missing or ambiguous")
            asset = matching_assets[0]
            require(relative == f'meshes/visual/{Path(asset["name"]).stem}.dae', "visual mesh source differs from generated binding")
            digest = sha256(source)
            require(digest == asset["dae_sha256"], "visual DAE source hash mismatch")
            source_hashes[relative] = digest
            transform[:3, :3] = rz @ ry @ rx @ np.diag(scale)
            transform[:3, 3] = xyz
            visuals += 1
        matrices.append(transform)
    require(visuals == 11, "expected eleven URDF visuals")
    return matrices, source_hashes


def independent_body_matrix(values):
    """Double precision quaternion algebra independent of renderer PoseMatrix."""
    import numpy as np

    values = np.asarray(values, dtype=np.float64)
    require(values.shape == (7,) and np.isfinite(values).all(), "invalid Newton transform")
    quaternion = values[3:].copy()
    norm = np.linalg.norm(quaternion)
    require(norm >= 1e-6, "degenerate Newton quaternion")
    quaternion /= norm
    v, w = quaternion[:3], quaternion[3]
    x, y, z = v
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = (w * w - np.dot(v, v)) * np.eye(3) + 2 * np.outer(v, v) + 2 * w * skew
    matrix[:3, 3] = values[:3]
    return matrix


def source_commit(module, expected):
    result = subprocess.run(
        ["git", "-C", str(Path(module.__file__).resolve().parent), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True, timeout=30,
    )
    require(result.stdout.strip() == expected, f"{module.__name__} source is not the pinned commit")


def replay_home(artifact_dir, description_root, physics, output):
    import newton
    import numpy as np
    import warp as wp

    require((newton.__version__, wp.__version__) == (NEWTON_VERSION, WARP_VERSION),
            "use the pinned Newton/Warp source environment")
    source_commit(newton, NEWTON_COMMIT)
    source_commit(wp, WARP_COMMIT)
    config = load_config()
    metadata = read_json(artifact_dir / "capture_metadata.json")
    for key in ("initial_q", "initial_qd", "joint_names", "lower_limits", "upper_limits"):
        require(physics[key] == config[key] == metadata[key], f"capture/config {key} mismatch")
    require(physics["initial_qd"] == [0] * 9, "home replay requires zero velocity")
    require(metadata["body_names"] == BODY_NAMES and metadata["urdf_sha256"] == physics["urdf_sha256"],
            "capture metadata source/body mismatch")
    wp.config.kernel_cache_dir = str(output / "warp-cache")
    wp.init()
    model = build_model(description_root, config)
    require(list(model.body_label) == BODY_NAMES, "live Newton body order differs from physics manifest")
    graph = wp.capture_load(str(artifact_dir / "franka_reset.wrp"), device="cpu")
    buffers = {name: value["size"] for name, value in graph._params.items()}
    require(buffers == {"joint_q_in": 36, "joint_qd_in": 36, "joint_q_out": 36,
                        "joint_qd_out": 36, "body_q_out": 336}, "reset graph buffer contract mismatch")
    home = wp.array(physics["initial_q"], dtype=wp.float32, device="cpu")
    zero = wp.zeros(9, dtype=wp.float32, device="cpu")
    graph.set_param("joint_q_in", home)
    graph.set_param("joint_qd_in", zero)
    wp.capture_launch(graph)
    actual_q = wp.zeros(9, dtype=wp.float32, device="cpu")
    actual_qd = wp.zeros(9, dtype=wp.float32, device="cpu")
    bodies = wp.zeros(12, dtype=wp.transform, device="cpu")
    graph.get_param("joint_q_out", actual_q)
    graph.get_param("joint_qd_out", actual_qd)
    graph.get_param("body_q_out", bodies)
    require(np.array_equal(actual_q.numpy(), home.numpy()) and np.array_equal(actual_qd.numpy(), zero.numpy()),
            "captured reset changed the home state")
    result = bodies.numpy().copy()
    require(result.shape == (12, 7) and np.isfinite(result).all(), "invalid captured body_q_out")
    # Independent live Newton FK checks that the saved graph's outputs really
    # retain the pinned model's body ordering and home transform semantics.
    state = model.state()
    newton.eval_fk(model, home, zero, state)
    require(np.allclose(result, state.body_q.numpy(), rtol=0, atol=1e-6),
            "captured reset differs from live pinned Newton FK")
    return result


def verify_negative_controls(verifier, output, fixture, bodies):
    """Prove the comparator detects representative placement regressions.

    Only ignored fixture copies change; production sources and the baseline
    real capture fixture remain untouched. A translated body simulates the old
    cuboid center offset, an omitted visual rotation tests the right finger,
    and exchanged raw bodies test accidental body-index binding.
    """
    cases = {}
    translated = bytearray(fixture)
    offset = 80 + 7 * 4  # panda_link1 px in raw body data
    struct.pack_into("<f", translated, offset, struct.unpack_from("<f", translated, offset)[0] + 0.002)
    cases["two_mm_body_offset"] = translated
    missing_rotation = bytearray(fixture)
    struct.pack_into("<16d", missing_rotation, 80 + 84 * 4 + 11 * 16 * 8,
                     *independent_body_matrix(bodies[11]).reshape(-1))
    cases["missing_rightfinger_visual_rotation"] = missing_rotation
    swapped = bytearray(fixture)
    first, second = 80 + 2 * 7 * 4, 80 + 3 * 7 * 4
    swapped[first:first + 28], swapped[second:second + 28] = swapped[second:second + 28], swapped[first:first + 28]
    cases["exchanged_link2_link3_body_binding"] = swapped
    results = {}
    for name, payload in cases.items():
        path = output / f"negative_{name}.bin"
        path.write_bytes(payload)
        rejected = subprocess.run([str(verifier), str(path)], capture_output=True, text=True, timeout=60)
        require(rejected.returncode != 0, f"parity verifier accepted negative control: {name}")
        measurement = json.loads(rejected.stdout)
        require(measurement.get("passed") is False, f"negative control did not fail a parity measurement: {name}")
        results[name] = {"rejected": True, "max_origin_error_m": measurement["max_origin_error_m"],
                         "max_orientation_error_degrees": measurement["max_orientation_error_degrees"]}
    return results


def verify(artifact_dir, description_root, mesh_dir, verifier, output):
    import numpy as np

    artifact_dir = Path(artifact_dir).resolve(strict=True)
    description_root = Path(description_root).resolve(strict=True)
    mesh_dir = Path(mesh_dir).resolve(strict=True)
    verifier = Path(verifier).resolve(strict=True)
    output = Path(output).resolve()
    require(verifier.is_file(), "compiled C++ parity verifier is required")
    for source in (artifact_dir, description_root, mesh_dir):
        require(not output.is_relative_to(source) and not source.is_relative_to(output),
                "parity output must not overlap source/artifact/mesh directories")
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "visual_parity_report.json"
    report = {"schema_version": 1, "passed": False, "status": "running"}
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    try:
        manifest_path = artifact_dir / "artifact_manifest.json"
        physics = read_json(manifest_path)
        verify_artifacts(physics, artifact_dir)
        require(physics["body_count"] == 12 and physics["body_names"] == BODY_NAMES,
                "unexpected physics body order")
        require(physics["body_frame"] == "robot_base" and physics["body_transform_layout"] == "px,py,pz,qx,qy,qz,qw",
                "unexpected physics body transform convention")
        for key, expected in (("newton_commit", NEWTON_COMMIT), ("newton_version", NEWTON_VERSION),
                              ("warp_commit", WARP_COMMIT), ("warp_version", WARP_VERSION)):
            require(physics[key] == expected, f"physics {key} is not pinned")
        urdf_digest = sha256(description_root / "robots/panda_arm_hand.urdf")
        require(urdf_digest == physics["urdf_sha256"], "URDF differs from captured physics source")
        mesh_manifest = verify_output(mesh_dir, manifest_path)
        local_visuals, dae_hashes = independent_visuals(description_root, mesh_manifest)
        bodies = replay_home(artifact_dir, description_root, physics, output)
        expected = np.array([independent_body_matrix(body) @ visual
                             for body, visual in zip(bodies, local_visuals)], dtype=np.float64)
        require(expected.shape == (12, 4, 4) and np.isfinite(expected).all(), "invalid expected visual transforms")
        mesh_digest = sha256(mesh_dir / "franka_meshes.json")
        fixture_path = output / "visual_parity.bin"
        fixture = struct.pack("<4sIII64s", b"QVPF", 1, 12, 11, mesh_digest.encode("ascii"))
        fixture += struct.pack("<84f", *bodies.reshape(-1))
        fixture += struct.pack("<192d", *expected.reshape(-1))
        fixture_path.write_bytes(fixture)
        result = subprocess.run([str(verifier), str(fixture_path)], capture_output=True, text=True, timeout=60)
        require(result.returncode == 0, "C++ parity verifier rejected fixture: " + (result.stderr or result.stdout).strip())
        measured = json.loads(result.stdout)
        require(measured.get("passed") is True and measured.get("body_count") == 12 and measured.get("visual_count") == 11,
                "invalid C++ parity result")
        require(measured["mesh_manifest_sha256"] == mesh_digest, "C++ verifier uses stale generated meshes")
        require(math.isfinite(measured["max_origin_error_m"]) and measured["max_origin_error_m"] < 0.001
                and math.isfinite(measured["max_orientation_error_degrees"]) and measured["max_orientation_error_degrees"] < 0.1,
                "measured visual parity exceeds thresholds")
        negative_controls = verify_negative_controls(verifier, output, fixture, bodies)
        report = dict(measured, schema_version=1, status="verified", pose="captured_reset_home",
                      fixture="visual_parity.bin", fixture_sha256=hashlib.sha256(fixture).hexdigest(),
                      physics_manifest_sha256=sha256(manifest_path), urdf_sha256=urdf_digest,
                      reset_graph_sha256=sha256(artifact_dir / "franka_reset.wrp"), source_dae_sha256=dae_hashes,
                      newton_commit=NEWTON_COMMIT, warp_commit=WARP_COMMIT,
                      negative_controls=negative_controls,
                      expected_transform_method="float64 normalized Newton quaternion times independently parsed URDF Rz Ry Rx and mesh scale")
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        return report
    except Exception as error:
        report.update(passed=False, status="failed", error=str(error))
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--franka-description-root", type=Path, required=True)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--verifier", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = verify(args.artifact_dir, args.franka_description_root, args.mesh_dir, args.verifier, args.output)
    except Exception as error:
        print(f"Visual parity verification failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps({key: report[key] for key in ("passed", "body_count", "visual_count",
          "max_origin_error_m", "max_orientation_error_degrees")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
