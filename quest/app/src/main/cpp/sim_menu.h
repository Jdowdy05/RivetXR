#pragma once

#include "sim_settings.h"

#include <functional>
#include <memory>
#include <string>

struct xrJava_;
using xrJava = xrJava_;

namespace OVR {
template <class T> class Pose;
using Posef = Pose<float>;
} // namespace OVR

namespace OVRFW {
class ovrFileSys;
struct ovrApplFrameIn;
struct ovrRendererOutput;
} // namespace OVRFW

namespace quest_newton {

enum class SimMenuCommand {
  Reset,
  TogglePause,
  SpawnBox,
  RemoveBox,
  RestoreDefaults,
  Recenter,
  RefreshRoom,
  ScanRoom,
  PlaceCube,
  SelectNextCube,
  MoveCube,
  ResetCube,
  DeleteCube,
  ToggleDiagnostics,
  RecordReset,
  StopRecording,
  RemoteDemo,
  RemoteConnect,
  RemoteDisconnect,
  RemoteInspect,
  RemoteAlign,
  RemoteForward,
  RemoteBackward,
  RemoteScale,
  RemoteDemoPrepared,
  RemoteDemoUnprepared,
  RemoteDemoRgbPrepared,
  RemoteDemoRgbUnprepared,
  GimbalConnect,
  GimbalDisconnect,
  GimbalMode,
  GimbalResetMap,
};

struct SimMenuCallbacks {
  std::function<bool(const SimSettings &, std::string &)> applySettings;
  std::function<void(SimMenuCommand)> command;
};

struct SimMenuStatus {
  bool paused = false;
  bool settings_pending = true;
  std::string backend = "Backend: starting";
  std::string physicsCpu = "Physics CPU: pending";
  std::string gpu = "GPU: pending";
  std::string contacts = "Contacts: pending";
  std::string gripper = "Gripper: starting";
  std::string room = "Room collisions are off";
  std::string diagnostic;
  std::string objects = "No user cubes; Place cube to add one";
  std::string recording = "Recording off";
  std::string control = "Control: starting";
  std::string latency = "Input/scene age: pending";
  bool show_diagnostics = false;
  bool placing_object = false;
  std::string remote = "Remote source off";
  bool remote_inspection = false;
  float remote_scale = 1;
  std::string camera = "Camera control disconnected; simulated gimbal only";
  unsigned camera_mode = 0; // session-only: off, head, right controller
};

// Render-thread owned TinyUI settings menu. The app owns input actions and
// suspends/rearms robot control around IsOpen(); this class owns only GUI
// state.
class SimMenu {
public:
  SimMenu();
  ~SimMenu();

  SimMenu(const SimMenu &) = delete;
  SimMenu &operator=(const SimMenu &) = delete;

  bool Init(const xrJava *context, OVRFW::ovrFileSys *fileSys,
            const SimSettings &initialSettings, SimMenuCallbacks callbacks);
  void Shutdown();
  // Pointer GPU resources follow the EGL session, not Activity lifetime.
  bool SessionInit();
  void SessionEnd();

  void SetVisible(bool visible, const OVR::Posef &stageFromHead);
  void Toggle(const OVR::Posef &stageFromHead);
  [[nodiscard]] bool IsOpen() const;

  void SetSettings(const SimSettings &settings);
  void SetStatus(const SimMenuStatus &status);

  void Update(const OVRFW::ovrApplFrameIn &in, const OVR::Posef &leftAim,
              bool leftAimValid, bool leftClick, const OVR::Posef &rightAim,
              bool rightAimValid, bool rightClick, bool hasInputFocus);
  void Render(const OVRFW::ovrApplFrameIn &in, OVRFW::ovrRendererOutput &out);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace quest_newton
