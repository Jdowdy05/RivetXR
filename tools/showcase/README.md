# Synthetic remote-inspection showcase

This optional tool produces a host-rendered orbit of **actual decoded RSCN v3
geometry**. It does not create a Quest capture or infer depth from a picture.

The scene contains analytic boxes and cylinders: a workbench, crates, machine
cabinet, pipes and valve. A supplied texture is sampled only as the known flat
workbench's albedo. Six synthetic RGB-D cameras raycast independent metric depth
and color frames. Four older posed views are retained through the production
`RetainedObservationMap`.

Both `build_rgb_batch(..., geometry='prepared')` and `geometry='unprepared'`
are exercised, including `pack_packet`, `unpack_packet` and worker-equivalent
surface construction. Vertices, indices and RGB atlas must match exactly before
any showcase image is rendered. The output mesh is not filled, smoothed or
replaced with the analytic source scene.

## Run

From the repository root with Python 3.12, NumPy and Pillow installed:

```sh
python -m pip install -r requirements-showcase.txt
python -m unittest tools.showcase.tests.test_render_inspection -v
python -m tools.showcase.render_inspection --texture docs/media/workbench-albedo.png --texture-origin ai-generated --output out/inspection-showcase --frames 80 --fps 10 --width 640 --height 360
```

For your own albedo use `--texture-origin user-provided`. `--font` optionally
selects an installed TrueType font; no font file is distributed by this tool.
Host raster antialiasing defaults to 2x supersampling and can be changed with
`--supersample 1`. It does not alter the decoded mesh.

Outputs:

- `inspection-orbit.gif`: 640x360, 80 frames at 10 FPS by default.
- `inspection-poster.png`: first orbit frame.
- `inspection-orbit-contact-sheet.png`: eight frames sampled from the actual GIF.
- `inspection-inputs.png`: actual synthetic RGB and depth input contact sheet.
- `inspection-prepared.rscn` / `inspection-unprepared.rscn`: production scene
  packets used for the equivalence proof.
- `provenance.json`: camera/observation count, geometry/packet sizes, input and
  packet hashes, calibration/poses, and the explicit host/synthetic scope.

Suggested caption:

> Reconstruction from a new angle. Synthetic RGB and depth observations are
> reconstructed by the RSCN pipeline and inspected along a host-rendered orbit.
> The generated workbench albedo is applied to known geometry; it does not
> provide depth. This is not a headset or real-camera capture.

This benchmark deliberately uses **six synthetic RGB-D cameras**, unlike the
proposed five-Y8 plus one-RGB physical rig. It includes four retained observations,
not four extra physical cameras. Reconstruction gaps and edge artifacts remain
visible. Runtime duration here is host media-generation time, not a Quest
physics, networking or rendering benchmark. No Newton settings, physical robot
commands or production protocol behavior are changed.
