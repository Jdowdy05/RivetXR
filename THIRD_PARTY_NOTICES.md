# Third-party notices

The repository's MIT license applies to original project code unless a file
states otherwise. It does not replace the licenses of third-party code,
dependencies, SDKs, robot descriptions, fonts or other assets.

## Included derivative source

`quest/app/src/main/python/quest_sim/cpu_solver.py` adapts the Newton MuJoCo CPU
adapter and retains these upstream notices:

Copyright (c) 2025 The Newton Developers

SPDX-License-Identifier: Apache-2.0

Its changes add solver-local staging/dispatch caching and CPU array reuse for
the pinned Android runtime. Preserve its existing copyright, license and
modification notices. The Apache License, Version 2.0 is reproduced in
[`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt).

Source: [Newton at the audited commit](https://github.com/newton-physics/newton/tree/d37f4d3d341ccce1e06a1dff21e9a054759b4855).

The patch files under `cmake/patches/` and `tools/full_port/patches/` modify separately obtained
Warp source. Their upstream code remains subject to its original
Apache-2.0 license; preserve those notices and the recorded modification history.

## External build and runtime dependencies

The source distribution does not include an APK, runtime wheels, external
engine checkouts, the Meta SDK, robot meshes, or SDK font/panel assets. Obtaining,
building or redistributing those components is governed by their own licenses.
The following identifies the principal dependencies; it is not a grant to
redistribute a built APK without its complete component notices and source obligations.

| Component | Audited source/version | Principal license |
|---|---|---|
| Newton | `d37f4d3d341ccce1e06a1dff21e9a054759b4855` | [Apache-2.0](https://github.com/newton-physics/newton/blob/d37f4d3d341ccce1e06a1dff21e9a054759b4855/LICENSE.md) |
| NVIDIA Warp | `d4de134b97b961f1a19bd76830e71ea7f9df2470` | [Apache-2.0 and component notices](https://github.com/NVIDIA/warp/tree/d4de134b97b961f1a19bd76830e71ea7f9df2470/licenses) |
| MuJoCo | Android 3.12.1, `10eeb8289598421cbca0b39459652ed57ec3d2e6` | [Apache-2.0](https://github.com/google-deepmind/mujoco/blob/10eeb8289598421cbca0b39459652ed57ec3d2e6/LICENSE) and dependency notices |
| MuJoCo Warp | 3.12.0 | [Apache-2.0](https://github.com/google-deepmind/mujoco_warp) |
| LLVM/Clang and Android libc++ | LLVM 22.1.8; Android NDK 27.0.12077973 | [Apache-2.0 with LLVM exceptions](https://github.com/llvm/llvm-project/blob/ca7933e47d3a3451d81e72ac174dcb5aa28b59d1/llvm/LICENSE.TXT), plus toolchain notices |
| Chaquopy | 17.0.0 | [MIT](https://github.com/chaquo/chaquopy/blob/17.0.0/LICENSE.txt) |
| CPython | Android 3.12.12 | [PSF license stack and incorporated-software licenses](https://docs.python.org/3.12/license.html) |
| NumPy | 1.26.2 Android wheel | [BSD-3-Clause and bundled notices](https://github.com/numpy/numpy/tree/v1.26.2) |
| OpenBLAS | Chaquopy 0.2.20 build 5 | [BSD-3-Clause and incorporated components](https://github.com/OpenMathLib/OpenBLAS/tree/v0.2.20) |
| GNU libgfortran | Chaquopy 4.9 build 0 | GPL-3.0-or-later with [GCC Runtime Library Exception 3.1](https://gcc.gnu.org/onlinedocs/libstdc++/manual/license.html); the library retains its own redistribution/source obligations |
| Eigen | `ea13a98decd497a8c5588fb5de71b57bcf10d864` | [MPL-2.0 and per-file licenses](https://gitlab.com/libeigen/eigen/-/tree/ea13a98decd497a8c5588fb5de71b57bcf10d864) |
| Trimesh | 4.8.3 | [MIT](https://github.com/mikedh/trimesh) |
| typing_extensions | 4.15.0 | [PSF-2.0](https://github.com/python/typing_extensions) |
| OpenXR loader | 1.1.53 | [Apache-2.0](https://github.com/KhronosGroup/OpenXR-SDK/tree/release-1.1.53) |
| Optional RealSense SDK | operator-selected runtime; source audit `e15c5d6bb1563e778d116f682aeefffbae2daedc` | [Apache-2.0 and component notices](https://github.com/IntelRealSense/librealsense) |
| Meta OpenXR SDK / SampleXrFramework / TinyUI | v85, `bbed2f20e38a5df7113630771c83cb8279e4fc26` | [Meta SDK license](https://developers.meta.com/horizon/licenses/oculussdk/) and [SDK third-party notices](https://github.com/meta-quest/Meta-OpenXR-SDK/blob/bbed2f20e38a5df7113630771c83cb8279e4fc26/OPENXR_SDK_THIRD_PARTY_NOTICES.txt) |
| Franka Panda description and assets | supplied separately by the builder | [Upstream franka_ros Apache-2.0](https://github.com/frankarobotics/franka_ros/blob/0.7.1/LICENSE); verify the provenance and terms of the particular input assets |

The Meta SDK and its content are not relicensed under MIT. SDK redistribution
is subject to its agreement, including its application/platform scope and
required notices. Its prescribed attribution is:

Copyright © Meta Platform Technologies, LLC and its affiliates. All rights reserved.

The SDK's KTX prebuilt contains additional components, including an Ericsson
ETC decoder with separate terms. Consult the SDK/KTX component licenses before
redistributing binaries. The SDK's OculusSans-derived font atlas and panel
are external SDK content, not original project assets.

Franka, Meta, NVIDIA, Unitree, and other product names identify compatibility or
upstream components. No endorsement or trademark permission is implied.
