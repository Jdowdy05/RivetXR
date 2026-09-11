# Device validation

The current full-runtime application was installed and checked on **Meta Quest
3S on September 11, 2026**. Normal startup, simulation progression and readable
RIVET XR menus were observed. Existing application data and saved settings were
preserved. The factory physics default remains 200 Hz.

The launcher and in-VR title now display **RIVET XR**. The Android package remains
`com.questnewton`, so this rename preserves application identity. Updating an
existing installation also requires the same signing key.

## Checks completed

| Check | Result and scope |
|---|---|
| CPU JIT | Executed on device in four cold application launches. |
| CPU equivalence | Baseline determinism and cached/uncached comparisons each passed 780 operations and 67,947 exact field comparisons on the same device. |
| Cache execution | No dispatch fallbacks in 34 candidate execution snapshots; command and array-view caches remained within their bounds. |
| Synthetic scene | 17 checks over 517 physics steps, including 4,275 state-preservation comparisons. |
| Gripper hold/release | Eight checks over 1,441 steps using the Original collision profile in a fixed-base, table-supported fixture. |
| Native ARM64 tests | RGB decoding/invalid inputs, gimbal control math and gimbal client policy passed. |
| RGB representation fixtures | Twelve prepared/unprepared pairs matched on device, covering retained observations, invalid depth, tracking loss, image age and color-view coverage/occlusion. |
| App startup and menus | Normal cold launch advanced with saved room geometry loaded; menu text was readable. |

The tested runtime used Newton 1.6.0.dev0, Warp 1.18.0.dev2, NumPy 1.26.2,
Android MuJoCo 3.12.1 and MJWarp 3.12.0. The Windows host environment uses
MuJoCo 3.12.0; these results do not establish cross-platform equality.

## What these results do not establish

- **Manipulation quality:** the gripper fixture retained bilateral contact for
  its final 100 samples and released correctly, but raw force-bearing penetration
  still reached **4.321 mm**. This table-supported result does not qualify
  sustained pinch/lift/place or eliminate the known manipulation limitations.
- **Remote visual acceptance:** ARM64 parsing and geometry checks do not test
  GLES rasterization. RGB texture alignment, stereo depth, head-motion parallax
  and a live scene stream still require headset wearer checks.
- **Physical hardware:** real camera capture, exposure-pose integration,
  controller-driven camera interactions and physical gimbal/robot operation
  remain unqualified. The provided gimbal driver is simulated.
- **Performance limits:** diagnostic durations are not physics-rate benchmarks.
  This session does not establish a maximum scene rate, combined camera/physics
  capacity or thermal endurance.

Startup includes handled optional-SciPy warnings while Trimesh generates mesh
normals. The pinned Trimesh code falls back to its NumPy implementation; the
tested MuJoCo collision path uses mesh vertices and faces. The checks passed
with that fallback, but other SciPy-dependent features have not been qualified.

## Remaining visual check

In **Remote**, select **retained RGB / robot-built**, then **Load demo** and
**Enter inspection** or **Align view**. Inspect color, texture alignment and
depth while moving your head. Repeat with **retained RGB / Quest-built**.
Bundled synthetic demos freeze observation age; they do not validate live
stream timing or disconnect behavior. See [remote setup and controls](REMOTE_SCENE.md).

This repository distributes source and synthetic fixtures. Prebuilt APKs,
runtime bundles, device recordings and device-specific evidence are not included
in this update. See [building prerequisites](BUILDING.md) for the remaining
from-scratch build requirements.
