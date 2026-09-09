# Building

## Start with the host tools

Use Python 3.12 and install `requirements-remote.txt` from the repository root in a virtual environment.
The remote pipeline uses NumPy and the Python standard library. It does not
need Newton, MuJoCo, Warp, Android Studio or `pyrealsense2` for synthetic data,
recorded replay or simulation-only gimbal operation.

The remote suite has been exercised with CPython 3.12 and NumPy 1.26.2. That
NumPy release has [official CPython 3.12 wheels](https://pypi.org/project/numpy/1.26.2/)
for common Windows, Linux and macOS platforms. Python 3.13+ and arbitrary newer
NumPy releases are not the baseline specified by this requirements file.

From the repository root:

```sh
python -m unittest discover -s tools/remote_scene/tests -v
python -m tools.remote_scene.surface_fixtures --output out/fixtures/grayscale
python -m tools.remote_scene.rgb_fixtures --output out/fixtures/rgb
```

The TLS fixtures require an OpenSSL executable. The helper checks `PATH` and
known platform fallback locations, and skips those tests if no executable is
available. Inspect the test
summary for skips. The provisioning CLI accepts an explicit `--openssl` path;
see [REMOTE_SCENE.md](REMOTE_SCENE.md). Installing a Python package named
`openssl` is not a substitute for this executable.

Optional Java transport tests need JDK 17 with `java` and `javac` on `PATH`:

```sh
python -m unittest discover -s tests/remote_scene_transport -v
```

Some interoperability checks additionally use native executables from the host
build. Follow the test output's prerequisite checks; a skipped optional test
has not been exercised.

## Native host checks

Install a C++20 compiler, CMake 3.24 or newer and Ninja. On Windows, run these
commands in a developer shell where the MSVC compiler is available. The existing
build has been exercised with MSVC; Linux/macOS build qualification is separate.

```sh
cmake -S . -B out/host -G Ninja -DBUILD_TESTING=ON
cmake --build out/host
ctest --test-dir out/host --output-on-failure
```

A multi-configuration generator may instead require `--config Debug` for the
build and `-C Debug` for CTest. These host checks cover native contracts and
mock rendering. They do not launch OpenXR or prove headset GPU performance.
The clean Windows source export passes 26 default CTests and 112 remote Python
tests. An additional room API contract can be enabled by setting
`QUEST_NEWTON_OPENXR_INCLUDE_DIR` to separately obtained OpenXR headers; it is not
part of the SDK-free default suite.

## Full Quest APK: additional inputs are required

The application is a native Android/ARM64 project. Its maintained full-runtime
build wrapper is currently **Windows/PowerShell-specific**. It invokes Windows
JDK, NDK and Gradle executables and builds LLVM host tools with MSVC. A Linux or
macOS Android build wrapper has not been qualified.

The source checkout alone is not yet a complete, single-command APK bootstrap.
The build requires two generated bundles and a matching external Franka model.
Do not replace missing inputs with desktop wheels or an arbitrary robot model.

| Input | Required selection / contract |
|---|---|
| Android SDK | Platform 34; Build Tools 34.0.0; Platform Tools for optional installation |
| Android NDK | Exact revision `27.0.12077973` |
| SDK CMake | `3.22.1` for the Android Gradle project |
| Host CMake / Ninja | CMake 3.24+; a working Ninja executable |
| Java | JDK 17 |
| Gradle / Android plugin | Gradle 8.5 wrapper from the pinned Meta SDK; Android Gradle Plugin 8.1.4 |
| Python embedding | Chaquopy Gradle plugin 17.0.0; Python 3.12 target |
| Meta OpenXR SDK | Exact commit `bbed2f20e38a5df7113630771c83cb8279e4fc26` (v85), clean tracked source |
| Newton | `d37f4d3d341ccce1e06a1dff21e9a054759b4855` |
| Warp | `d4de134b97b961f1a19bd76830e71ea7f9df2470` plus the repository's Android JIT portability patch in an isolated build copy |
| MuJoCo source | `10eeb8289598421cbca0b39459652ed57ec3d2e6` (Android core/bindings 3.12.1) |
| LLVM/Clang | `ca7933e47d3a3451d81e72ac174dcb5aa28b59d1` (`llvmorg-22.1.8`) |
| Legacy artifact bundle | Verified APIC Franka graphs/libraries with `artifact_manifest.json` |
| Full runtime bundle | Verified output containing `runtime_manifest.json`, Python packages, native ARM64 libraries, Android wheels and assets |
| Franka description | `robots/panda_arm_hand.urdf` and matching visual/collision meshes; identity must match the artifact manifest |

Obtain upstream sources from their respective projects: [Newton](https://github.com/newton-physics/newton),
[Warp](https://github.com/NVIDIA/warp), [MuJoCo](https://github.com/google-deepmind/mujoco),
[LLVM](https://github.com/llvm/llvm-project), and [Meta OpenXR SDK](https://github.com/meta-quest/Meta-OpenXR-SDK).
Check out the exact commits above rather than tracking their default branches.
Each dependency and robot asset remains subject to its own license.

### What the bundle tools do

1. `tools/newton_codegen/build_android_artifacts.py` creates the retained legacy
   APIC bundle. It needs the pinned Newton/Warp sources, an already working
   **host Warp CPU runtime/compiler**, the NDK, Ninja and the selected Franka
   asset. It builds and verifies the captured Android graphs.
2. `tools/full_port/build_warp_android.ps1` builds host LLVM table generators,
   Android LLVM/Clang and Warp's CPU JIT libraries. Use a short external build
   directory; the script requires at least 20 GB free before fetching LLVM.
3. `tools/full_port/build_mujoco_python_android.py` builds Python bindings
   against a supplied Android `libmujoco.so`. **It does not build that core
   library.** A verified MuJoCo core cross-build is a separate prerequisite.
4. `tools/full_port/stage_python_runtime.py` assembles the full bundle and
   checks source pins, downloads, native dependencies and assets.
5. `tools/build_quest.ps1` validates/stages both bundles, converts the matching
   visual meshes, generates demo packets, builds the APK and runs its package
   contract.

The full APK still requires the APIC bundle because an explicit legacy
verification mode remains in the application. Normal full-runtime execution
must not silently fall back to that mode.

### Downloads and gaps to account for

The bundle tools fetch exact, hash-checked inputs from Maven Central, PyPI and
Chaquopy's package index. These include the Chaquopy Python target archive,
NumPy 1.26.2 Android wheel, OpenBLAS/libgfortran wheels, MJWarp 3.12.0, trimesh
4.8.3 and typing_extensions 4.15.0. CMake also fetches pinned Abseil, Eigen and
pybind11 sources. Gradle resolves Android, Chaquopy and OpenXR loader artifacts.
LLVM is a separate large source checkout/build.

This source-only release does not provide prebuilt APKs, the selected Franka
asset or generated runtime bundles as downloads. There is no complete automated bootstrap for the
host Warp compiler, the MuJoCo Android core, or obtaining and validating the
exact robot asset. Those gaps must be closed or the corresponding verified
inputs supplied before claiming a from-scratch APK build is reproducible.

### Build after preparing the inputs

In PowerShell, define the following variables using your own installation and
build directories. Use absolute paths for inputs. Keep generated bundles,
certificates and build output outside tracked source.

```powershell
$RepoRoot = (Get-Location).Path
$HostPython = (Resolve-Path '.venv/Scripts/python.exe').Path
$BuildRoot = Join-Path $RepoRoot 'out/quest-build'

# Set these to existing, verified inputs before running the command:
# $ArtifactDir, $MetaOpenXrSdkRoot, $AndroidSdkRoot, $NdkRoot,
# $JavaHome, $FrankaDescriptionRoot, $FullRuntimeBundle

pwsh -NoProfile -File tools/build_quest.ps1 `
  -ArtifactDir $ArtifactDir `
  -MetaOpenXrSdkRoot $MetaOpenXrSdkRoot `
  -AndroidSdkRoot $AndroidSdkRoot `
  -NdkRoot $NdkRoot `
  -JavaHome $JavaHome `
  -FrankaDescriptionRoot $FrankaDescriptionRoot `
  -HostPython $HostPython `
  -FullRuntimeBundle $FullRuntimeBundle `
  -BuildRoot $BuildRoot
```

Pass `-CMake` and `-Ninja` if their executables are not on `PATH`. Set
`-VsDevShell` explicitly when using the Warp build script with a different
Visual Studio installation. The default APK staging directory is `out/quest-build`; use a short explicit
external path for the much larger LLVM/Warp build.
Use `-StageOnly` only to validate/refresh staging for a later Gradle build.

The expected full debug APK is:

```text
quest/app/build/outputs/apk/debug/quest-newton-full-debug.apk
```

The wrapper runs `tools/full_port/verify_apk.py`, which checks native libraries,
package contents, manifests, demo assets and signing. A passing APK contract
means the package is internally consistent; it does not prove runtime JIT,
visual quality, controller behavior or sustained headset performance.
