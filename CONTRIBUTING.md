# Contributing to RIVET XR

Start with [README.md](README.md) and [docs/BUILDING.md](docs/BUILDING.md).
Keep changes focused and include the relevant test results in a pull request.

The native host suite requires CMake and a C++20 compiler. Remote scene tests
require Python 3.12 and `requirements-remote.txt`. Camera tests use a simulated
SDK; real camera/headset tests are separate qualification steps.

Preserve explicit coordinate frames, units, timestamps and validity flags.
Keep reconstruction and networking off the XR render thread. Do not substitute
commanded motor positions for measured exposure poses. Preserve compatibility
with existing recording and packet formats, or version their contracts.

Do not submit credentials, private endpoints, personal recordings, local
configuration or generated build artifacts. Use synthetic data in tests and
document its generation. Preserve third-party copyright and license notices.

Physical actuation is outside the current public release. Contributions adding
it must define limits, stale-command behavior, loss/rearm handling and an
independent stop mechanism, with simulation tests before hardware use.

By contributing original work, you agree to license that contribution under
this project's MIT license. Identify any third-party material and its license.
