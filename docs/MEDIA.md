# Showcase media

The README separates recorded headset behavior from a synthetic demonstration
of the reconstruction pipeline. GIF playback rate is a presentation choice,
not a measurement of physics or XR frame rate.

## Headset recordings

| Asset | What it shows | Treatment |
|---|---|---|
| [`quest-arm-objects.gif`](media/quest-arm-objects.gif) | Controller-driven virtual gripper and a scene cube in passthrough | Five-second close-up; the crop excludes the monitor. |
| [`quest-vr-settings.gif`](media/quest-vr-settings.gif) | Native in-VR settings and timing controls | Five-second crop; monitor content is blurred throughout. |

Both clips come from the maintainer's actual **Quest 3S** development recording.
They are silent, looping, 600×360 GIFs at approximately eight frames per second.
No faces or readable monitor content are included. Original recordings are not
distributed. The overlays show the older development UI, before the RIVET XR
branding and newer RGB/Camera-tab work.

These clips demonstrate local virtual-robot interaction and UI behavior; they
do not establish reliable grasping, calibrated real-robot teleoperation or
performance of the newer remote reconstruction modes.

## Synthetic inspection demo

[`depth-inspection.gif`](media/depth-inspection.gif) is a **host-rendered preview
of real reconstruction output**, not a headset capture or measured camera data.
Its maintenance-bay scene contains a workbench, raised crates, a cabinet, pipes
and a valve wheel. Depth edges and unobserved gaps are left visible.

1. An analytic 3D scene supplies known geometry. The generated work-mat image
   is used only as the albedo texture of a flat bench.
2. Six explicitly synthetic RGB-D cameras raycast separate color and depth
   observations with defined intrinsics and exposure poses.
3. Those images pass through `build_rgb_batch`, the observation cache and
   RSCN v3 serialization. Four earlier posed observations are retained.
4. Prepared geometry and unprepared image packets are decoded and compared.
   The resulting vertices, indices and atlas agree.
5. A software rasterizer renders the decoded mesh from changing viewpoints to
   make the orbit GIF. It does not render the analytic scene directly for the
   final orbit, and it does not fill holes in the reconstructed mesh.

This example uses six RGB-D cameras to make the pipeline easy to inspect.
The proposed hardware layout of five depth/intensity cameras plus one D435i
RGB/depth camera has a different color-coverage pattern and still needs hardware
qualification. Generated image appearance is not used to infer depth.

[View the input RGB/depth images](media/inspection-inputs.png) ·
[Read the numeric provenance](media/inspection-provenance.json)

The included output contains 15,232 vertices and 24,275 triangles across six
current and four retained observations. Its prepared and unprepared packets
were additionally compared with the native C++ decoder: atlas bytes and indices
match exactly, and vertices agree within its 1e-6 tolerance. This validates this
fixture's reconstruction agreement; it does not validate headset rasterization.

### Reproduce

Use Python 3.12 from the repository root:

```sh
python -m pip install -r requirements-showcase.txt
python -m tools.showcase.render_inspection --texture docs/media/workbench-albedo.png --texture-origin ai-generated --output out/inspection-showcase
```

The script uses NumPy and Pillow; it does not require a headset or camera SDK.
Software rasterization can take several minutes. Defaults produce an eight-second
orbit at ten frames per second, 640×360, with 2× supersampling before encoding.
The output includes camera input previews, prepared/unprepared packets and
numeric provenance. This is a reproducible visualization example, not an XR
performance benchmark.

The generated material is [`workbench-albedo.png`](media/workbench-albedo.png).
Its [image-generation prompt](media/workbench-albedo.prompt.txt) is included.
The image was created with an image-generation tool; all camera geometry,
depth, surface reconstruction and output views are computed by the example.

## Rendering and device claims

RIVET XR's headset renderer is project-native GLES/OpenXR code built on Meta's
native sample framework. It is not Newton's desktop viewer. The physics path
uses Newton's CPU MuJoCo-backed solver, with custom dispatch/staging/array-reuse
optimizations around the solver. The equations and contact model are not
replaced by a new solver algorithm.

The application declares Quest 3 and Quest 3S targets. Recorded device evidence
and these headset clips use **Quest 3S**. New RGB surface rendering, actual
camera accuracy and concurrent sensing/physics performance need their own
device qualification.
