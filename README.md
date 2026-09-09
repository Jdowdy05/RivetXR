# Quest Newton

A native Quest robotics simulator and remote scene viewer. The headset runs a
single Franka Panda scene with Newton's CPU MuJoCo solver. A separate Python
pipeline combines calibrated depth and image observations into one stream for
stereo inspection in VR.

This initial release contains source only; no prebuilt APK or generated Android
runtime bundle is distributed. It is an experimental source release. The standalone host demos and tests
are the easiest place to start. Building the full Quest application additionally
requires generated Android physics bundles and separately obtained robot
assets; see [BUILDING.md](docs/BUILDING.md).

## Download

[Download the source ZIP](https://github.com/Jdowdy05/quest-newton/archive/refs/heads/main.zip)
and extract it, or clone the repository:

```sh
git clone https://github.com/Jdowdy05/quest-newton.git
cd quest-newton
```

## Try the host pipeline

Use **Python 3.12**. From the downloaded or cloned repository root, create an
isolated environment.

Windows PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Linux/macOS:

```sh
python3.12 -m venv .venv
. .venv/bin/activate
```

Then, on either platform:

```sh
python -m pip install -r requirements-remote.txt
python -m unittest discover -s tools/remote_scene/tests -v
python -m tools.remote_scene.demo --output out/demo/remote_scene/demo.rscn --all-modes
```

The generator writes five deterministic `.rscn` files: grayscale points,
prepared grayscale surfaces, unprepared grayscale observations, prepared RGB
surfaces with retained observations, and the equivalent unprepared RGB data.
These are scene packets for the application, not video files or a desktop GUI.
Synthetic generation does not require a camera, a headset, Newton, or a GPU.
TLS tests additionally need an OpenSSL executable discoverable on `PATH`;
skipped TLS tests are not network verification. [Remote setup and streaming](docs/REMOTE_SCENE.md) describes
the executable selection and a local two-channel demonstration.

## What is implemented

- A C++20/OpenXR headset application with passthrough, controller input, an
  in-VR settings menu and a local rigid/articulated physics worker.
- One built-in Franka scene, configurable simulation/control/render intervals,
  analog gripper control, scene objects and optional estimated-room collisions.
- One authenticated scene downlink with three representations: legacy grayscale
  points, robot-prepared textured geometry, or calibrated image observations
  reconstructed by the headset worker.
- A six-camera RGB/depth demo, bounded retained observations, tracking-loss
  invalidation and a separate **simulation-only** pan/tilt control endpoint.
- Raw camera recording/replay and protocol, geometry, timing, lifecycle and
  loopback tests that can run without hardware.

## Architecture

```text
Depth + Y8/RGB cameras
        |
        v
Robot/host: exposure poses, bounded observation cache, validation
        |
        +-- prepared: triangles + RGB/grayscale atlas
        +-- unprepared: depth + images + calibration + triangle masks
        |
        v
One pinned-TLS scene stream --> Quest receive worker --> stereo rendering

Quest native input/menu --> local Newton CPU simulation worker
Quest camera intent ----> separate simulated-gimbal control endpoint
```

The robot/host owns the retained cache in both remote modes. Unprepared mode
moves surface construction to the headset; it does not send six independent
video connections. Remote geometry is visual and is not inserted into Newton
as collision geometry.

## Current limits

The full CPU simulation has executed on Quest 3S. That does not establish the
performance or visual accuracy of every newer remote-rendering feature. The
RGB/retention/gimbal extension has host, native and packaging checks; real-camera,
headset visual and concurrent-workload qualification remain separate work.

Reliable pinch/lift/place is unresolved. Contact forces and collision counts do
not by themselves prove manipulation quality. Experimental finger-pad/friction
profiles are available for comparison and should not be treated as a solved
physical grasp model.

The retained scene is a bounded collection of observations, not TSDF/SLAM, a
watertight world, hidden-surface completion, or semantic self-filtering. It can
contain gaps, seams, stale surfaces and gray regions without usable image data.
There is no physical robot or physical gimbal driver in the remote-control path.
Unitree/B1/Z1 hardware control and walking-robot qualification are not implemented.

## Build and contribute

- [BUILDING.md](docs/BUILDING.md): host checks, Android prerequisites and generated
  bundle requirements.
- [REMOTE_SCENE.md](docs/REMOTE_SCENE.md): synthetic streaming, retention, replay,
  exposure/coordinate contracts and simulation-only control.
- `tools/remote_scene/`: host pipeline and its tests.
- `quest/app/src/main/cpp/`: native application, decoders and renderer.
- `quest/app/src/main/python/quest_sim/`: local simulation worker.

Keep hardware-independent tests and synthetic cases available when changing a
protocol, coordinate convention or solver adapter. Record raw failures rather
than clipping state or suppressing evidence to make a check pass.

## License

Original project source is provided under the [MIT license](LICENSE), except
files carrying their own license headers. In particular, the adapted Newton CPU
solver retains Apache-2.0. See [third-party notices](THIRD_PARTY_NOTICES.md). External
SDKs, libraries and robot assets retain their own licenses and notices; this
repository does not relicense them. In particular, the [Meta OpenXR SDK](https://github.com/meta-quest/Meta-OpenXR-SDK)
has its own SDK license.
