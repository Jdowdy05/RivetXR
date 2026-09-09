# Remote scene pipeline

The host combines observations into **one scene downlink**. The headset renders
that scene using native stereo graphics while local Newton simulation runs
independently. Synthetic sources and recorded replay need Python/NumPy only.
These tools do not provide a desktop interactive viewer.

## Choose the reconstruction location

| Launch option | Host work | Headset input |
|---|---|---|
| `--geometry prepared` | Builds triangles and image atlas | Prepared mesh, texture and observation metadata |
| `--geometry unprepared` | Calibrates, bounds, retains and masks observations | Depth, Y8/RGB images, calibration and masks; its receive worker builds surfaces |
| `--geometry points` | Legacy five-camera grayscale point fusion | RSCN v1 grayscale points |

Prepared is the default. RGB, six-camera or retained data cannot silently use
legacy points. Both surface modes use the same robot-owned cache and a single
authenticated connection. They do not send six independent video streams.

## Generate packets without hardware

Activate the environment described in [BUILDING.md](BUILDING.md), then run from
the repository root:

```sh
python -m tools.remote_scene.demo --output out/demo/remote_scene/demo.rscn --all-modes
python -c "from pathlib import Path; from tools.remote_scene.protocol import unpack_packet; p=unpack_packet(Path('out/demo/remote_scene/demo-rgb-prepared.rscn').read_bytes()); print(p.geometry_summary)"
```

The five files are `demo.rscn`, `demo-prepared.rscn`, `demo-unprepared.rscn`,
`demo-rgb-prepared.rscn` and `demo-rgb-unprepared.rscn`. The RGB pair includes
retained observations after a deterministic simulated pan sweep. These are binary
scene packets, not videos. RGB materials are raycast independently of depth;
they are not depth colorization.

## Local streaming and simulated control

Use a new, empty ignored output directory. Install an OpenSSL executable and
make it discoverable on `PATH`, or supply its explicit path. This local example
creates a one-day test certificate; deployments should supply their own
certificate/key instead.

Windows PowerShell:

```powershell
$OpenSsl = (Get-Command openssl -ErrorAction Stop).Source
python -m tools.remote_scene.provision --output-dir out/remote-private --test-certificate --openssl "$OpenSsl" --control-port 7444
```

Linux/macOS:

```sh
OPENSSL="$(command -v openssl)"
python -m tools.remote_scene.provision --output-dir out/remote-private --test-certificate --openssl "$OPENSSL" --control-port 7444
```

Provisioning writes `server.json`, `remote-scene-client.json`,
`gimbal-server.json` and `remote-gimbal-client.json`, plus the test certificate
and key. Scene and control use different random tokens; their certificate pin
may be shared. Do not commit these files. Replacing a certificate or token also
requires updating the corresponding client configuration.

Start the six-camera synthetic source and its **simulation-only** controller:

```sh
python -m tools.remote_scene.stream --source gimbal-demo --server-config out/remote-private/server.json --gimbal-control-config out/remote-private/gimbal-server.json --geometry prepared --retain-seconds 30 --batches 600
```

Change `prepared` to `unprepared` to construct surfaces on the headset. Omitting
`--gimbal-control-config` exposes no control endpoint. Control options are
rejected for real-camera, replay and legacy demo sources, so simulated feedback
cannot be attached to real hardware.

Default listeners are loopback ports 7443 for scenes and 7444 for control. They
are reachable only on the same machine. Intentional LAN deployment requires
`--host NUMERIC_BIND_ADDRESS` and the corresponding reachable numeric `host` in
each client JSON. Provisioning does not deploy those files to a headset.

The publisher requests 10 updates per second and stops at the batch limit
(maximum 600), or earlier with Ctrl-C. Overruns are reported. This is not a
camera-rig or headset throughput guarantee. Each live process has a random
source ID shared with its control feedback; frozen assets have different IDs.

## Loopback test and frozen publication

With OpenSSL discoverable, this test runs an actual local CLI subprocess and
pinned-TLS scene/control connections, checks measured virtual pan in the scene,
clears retained history, and tests lease expiry/disconnect:

```sh
python -m unittest tools.remote_scene.tests.test_gimbal_cli_loopback -v
```

If it reports a skip, OpenSSL was not found and that run has not verified TLS.
The helper checks `PATH` and supported platform fallback locations.

Serve one saved snapshot until Ctrl-C:

```sh
python -m tools.remote_scene.publisher --config out/remote-private/server.json --packet out/demo/remote_scene/demo-rgb-prepared.rscn
```

This serves the same complete frozen packet to clients. It does not move a
gimbal or fabricate refreshed observations.

## Headset first launch and controls

This release supplies source, not a prebuilt APK. After building and installing
a compatible full-runtime APK, launch **Quest Newton** (the legacy application
label for RIVET XR development builds) from the headset's
application library. Complete the headset's own boundary setup and wear the
headset before placing/recentering the robot. App room collisions and the
headset boundary are separate features.

These are the implemented default bindings; **Controls** can remap them:

| Control | Default behavior |
|---|---|
| Left controller Menu button | Open/close app settings |
| A, with right index trigger released | Calibrate the controller-to-arm frame |
| Hold right index trigger | Track the arm target; release holds the target |
| Right grip/squeeze | Analog gripper closing |
| Left index trigger | Local placement height clutch; simulated-camera clutch during eligible remote inspection |

Use the controller rays/cursors to select menu items. Arm/gripper input is
suppressed while menus or remote inspection are active. Release the arm trigger
to rearm after interruption; recalibrate when the status requests it.
**Placement > Recenter placement** corrects placement initialized at an
unhelpful headset position. **Simulation** contains reset, pause and timing
settings. Factory physics dt is 0.005 seconds (200 Hz); saved settings persist.

**Gripper** contains speed limiting and optional contact force holding. Defaults
are force holding Off, target speed 50 mm/s per finger, and full-squeeze force
target 5 N per finger. The simulated actuator cap is 20 N per finger, not a
guaranteed measured contact force or lifting capacity. Its contact-model
selector offers **Original mesh**, **Five pads**, and **Pads + friction**.
Original is the default. The alternatives are experiments; changing the model
resets the scene, and none establishes reliable grasping.

**Room** can request a scene scan/refresh and estimated-room collisions.
Availability depends on runtime permissions and room data. Applied estimated
surfaces are shown; this is not a detailed live furniture mesh.

For a bundled scene, open **Remote**, select a demo mode, choose **Load demo**,
then **Enter inspection**. **Align view**, view steps and scale adjust
inspection. **Exit inspection** returns to local simulation. Bundled demos need
no network credentials.

For a live source, install `remote-scene-client.json` in the app's private files
directory through your authorized developer deployment workflow, then use
**Remote > Connect > Enter inspection**. Its numeric host, certificate pin and
token must match the running publisher. The host provisioning command does not
perform this deployment.

Simulated gimbal operation additionally needs `remote-gimbal-client.json` in
that directory and a matching running control service. Open **Camera > Connect
camera**, choose **Aim: head + left trigger** or **Aim: right controller + left
trigger**, close the menu, release the left trigger, then hold it to clutch and
aim. Releasing holds the simulated camera. Menus, tracking/focus loss, stale
feedback, map/source changes and disconnect require release/rearm. Scene/control
identities must match; a frozen demo cannot arm live control. **Clear retained
remote map** clears history independently of motor intent and requires fresh
scene alignment.

These instructions describe source behavior. New RGB rendering, every menu/
controller interaction and new headset models still need device qualification.
There is no physical camera-motor driver behind these controls.

## Retention, occlusion and missing images

RGB/six-camera sources default to 30 seconds of history. Legacy five-camera
sources stay current-only unless retention is explicit. `--retain-seconds`
accepts 0 through 60. Zero disables historical keyframes; current RGB views
still have a 30-second display expiry and do not disappear immediately.

At most six current views and four retained keyframes are sent. Keyframes keep
original exposure timestamps. Source/world/calibration changes, explicit clear
and tracking loss retire the cache. Old depth is carved only when fresh valid
depth establishes free space. A nearer new occluder can hide old background
without erasing it; unknown depth does not establish free space.

Gray geometry is suppressed only when usable RGB observation geometry covers
it conservatively and remains valid at least as long. Overlap and seams may
remain. This preserves gray fallback when older RGB expires; it does not paint
unseen nearest colors onto new geometry.

Valid depth remains visible in neutral gray outside RGB coverage or where image
correspondence is unusable or occluded. Unusable images retain their real
capture timestamp for provenance, while geometry expiry uses depth time.
Usable observations age from the older depth/image exposure. Republishing an
observation does not refresh its age.

## Record and replay

Recordings preserve raw Z16, Y8/RGB8 images, calibration, exposure poses,
timestamps and hashes. `.rgb8` files contain raw bytes, not encoded pictures.
Recording v2 also preserves tracking-loss markers so replay retires history at
known localization loss.

Generate a small five-camera recording without hardware:

```sh
python -m tools.remote_scene.demo --output out/recorded-demo.rscn --recording-dir out/raw-demo
python -m tools.remote_scene.stream --source replay --manifest out/raw-demo/manifest.json --server-config out/remote-private/server.json --geometry prepared
```

This generated recording contains one batch and replay exits quickly. Use the
frozen publisher for manual inspection of a single saved packet. Longer
recordings preserve their producer clocks and recorded labels; they do not
become live observations.

## Real cameras and measured poses

RealSense is optional and imported only by explicit capture. The adapter audits
USB D400 timing/profile contracts and restricts RGB to the D435i path. It needs
an explicitly selected SDK version and queried profiles. Ordinary setup does
not install an arbitrary latest `pyrealsense2`. Qualify the intended camera,
firmware, synchronization, calibration and throughput independently.

Moving cameras require measured exposure-time robot and pan/tilt poses. The
pose-history adapter uses bracketed interpolation without extrapolation or
commanded/headset-pose substitution. RGB and depth have separate timestamps and
a cross-exposure transform. Missing RGB motion/readout evidence disables color
while preserving valid depth. Missing depth pose invalidates tracking and the
map. Read-only pose traces support controlled offline reproduction; they do not
retime old telemetry into current camera clocks or supply live localization.
Integrations can use `Capture(config, pose_provider=...)` with actual measured
telemetry. No physical gimbal backend is supplied.

## Coordinates and bounds

Map coordinates are metres: +X forward, +Y left, +Z up. Optical camera coordinates
are +X right, +Y down, +Z forward. Z16 multiplied by depth units is optical Z,
not Euclidean ray distance. Destination-from-source transforms are serialized
row-major; robot-relative and exposure-time world transforms both apply.

RSCN v3 permits 10 views, 32,768 vertices, 196,608 indices and 768,000 RGB atlas
pixels within a 4 MiB transport cap. Current raw presets reduce depth to at most
64x48 and images to at most 256x192 per view. These are deliberate bounded
resolution choices, not lossless full-resolution transmission. V1/v2 keep their
separate smaller compatibility bounds.

Triangles connect neighbors within a camera grid and leave depth-edge holes.
The result is not a watertight fused mesh, TSDF/SLAM, hidden-surface completion,
semantic self-filtering or collision map. Real-camera visual quality, hardware
gimbal integration, headset rendering and simultaneous physics/network
performance remain separate acceptance work.
