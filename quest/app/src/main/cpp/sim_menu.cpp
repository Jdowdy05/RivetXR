#include "sim_menu.h"

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-pedantic"
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "GUI/VRMenuObject.h"
#include "Input/TinyUI.h"
#include "menu_pointer_renderer.h"
#include "Render/BitmapFont.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#include <android/log.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace quest_newton {
namespace {

constexpr std::array<const char *, 10> kTabNames{"Simulation", "Controls", "Gripper",
                                                "Placement", "Room", "Objects", "Debug", "Record", "Remote", "Camera"};
// TinyUI surface dimensions are texels and VRMenuObject converts them at
// 500 texels per metre. Keep these centres consistent with their physical
// extents so independent TinyUI menus never overlap.
constexpr std::array<float, 10> kTabX{-.72F,-.56F,-.40F,-.24F,-.08F,.08F,.24F,.40F,.56F,.72F};
// Selected tab names fit the packaged EFIGS font above the 0.30 readable scale.
constexpr float kTabWidth = 76.0F;
constexpr const char *kApplicationFontUri = "apk:///res/raw/efigs.fnt";

std::string Fixed(double value, int precision) {
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::fixed << std::setprecision(precision) << value;
  return text.str();
}

} // namespace

struct SimMenu::Impl {
  enum class Tab : std::size_t {
    Simulation,
    Controls,
    Gripper,
    Placement,
    Room,
    Objects,
    Debug,
    Record,
    Remote,
    Camera,
    Count
  };
  enum class SettingField {
    PhysicsDt,
    ControlDecimation,
    RenderInterval,
    HandRoll,
    HandPitch,
    HandYaw,
    ShoulderOffset,
    Opacity,
    GripperSpeed,
    GripperForce,
  };
  struct Widget {
    OVRFW::VRMenuObject *object = nullptr;
    OVR::Vector3f localPosition{};
    int tab = -1;
    bool original_no_depth=false,original_no_depth_mask=false;
  };

  static constexpr std::size_t ToIndex(Tab tab) {
    return static_cast<std::size_t>(tab);
  }
  static constexpr int TabIndex(Tab tab) { return static_cast<int>(tab); }
  static constexpr std::size_t FieldIndex(SettingField field) {
    return static_cast<std::size_t>(field);
  }

  bool Init(const xrJava *context, OVRFW::ovrFileSys *fileSys,
            const SimSettings &initialSettings, SimMenuCallbacks callbacks);
  void Shutdown();
  void SetVisible(bool visible, const OVR::Posef &stageFromHead);
  void Toggle(const OVR::Posef &stageFromHead);
  void SetSettings(const SimSettings &settings);
  void SetStatus(const SimMenuStatus &status);
  void Update(const OVRFW::ovrApplFrameIn &in, const OVR::Posef &leftAim,
              bool leftAimValid, bool leftClick, const OVR::Posef &rightAim,
              bool rightAimValid, bool rightClick, bool hasInputFocus);
  void Render(const OVRFW::ovrApplFrameIn &in, OVRFW::ovrRendererOutput &out);
  OVRFW::VRMenuObject *AddLabel(const std::string &text,
                                const OVR::Vector3f &localPosition,
                                const OVR::Vector2f &size, int tab = -1);
  OVRFW::VRMenuObject *AddButton(const std::string &text,
                                 const OVR::Vector3f &localPosition,
                                 const OVR::Vector2f &size,
                                 std::function<void()> handler, int tab = -1);
  OVRFW::VRMenuObject *AddStepper(const std::string &label, SettingField field,
                                  float y, int tab, float left = -0.35F,
                                  float width = 0.0F);
  void BuildWidgets();
  void PlaceWidgets(const OVR::Posef &stageFromHead);
  void RefreshVisibility();
  void RefreshText();
  void SetFittedText(OVRFW::VRMenuObject *object, const std::string &text);
  void SetWrappedNotice(const std::string &text);
  void SetWrappedLabel(OVRFW::VRMenuObject *object, const std::string &text,
                       int maxLines);
  void SelectTab(Tab tab);
  void AdjustSetting(SettingField field, int direction);
  void ResetSetting(SettingField field);
  void ToggleSetting(bool SimSettings::*member);
  void CycleBinding(std::size_t actionIndex);
  void CycleContactProfile();
  bool ApplyCandidate(const SimSettings &candidate);
  void RunCommand(SimMenuCommand command);
  void RestoreDefaults();
  static std::string SettingValue(const SimSettings &settings,
                                  SettingField field);
  static std::string BoundedLine(std::string text, std::size_t maxLength);
  static std::string OneLine(std::string text);

  OVRFW::TinyUI ui_;
  std::unique_ptr<MenuPointerRenderer> pointers_;
  // Non-owning SDK geometry/program copies; only per-frame blend state differs.
  // Storage remains valid until the framework has drawn both eyes.
  std::vector<OVRFW::ovrSurfaceDef> ui_surfaces_;
  SimMenuCallbacks callbacks_;
  SimSettings settings_;
  SimMenuStatus status_;
  std::vector<Widget> widgets_;
  std::array<OVRFW::VRMenuObject *, static_cast<std::size_t>(Tab::Count)>
      tabButtons_{};
  std::array<OVRFW::VRMenuObject *, 10> settingValueButtons_{};
  std::array<OVRFW::VRMenuObject *, kActionCount> bindingButtons_{};
  OVRFW::VRMenuObject *pauseButton_ = nullptr;
  OVRFW::VRMenuObject *ratesLabel_ = nullptr;
  OVRFW::VRMenuObject *followButton_ = nullptr;
  OVRFW::VRMenuObject *gridButton_ = nullptr;
  OVRFW::VRMenuObject *roomCollisionsButton_ = nullptr;
  OVRFW::VRMenuObject *roomSurfacesButton_ = nullptr;
  OVRFW::VRMenuObject *roomStatusLabel_ = nullptr;
  OVRFW::VRMenuObject *backendLabel_ = nullptr;
  OVRFW::VRMenuObject *physicsCpuLabel_ = nullptr;
  OVRFW::VRMenuObject *gpuLabel_ = nullptr;
  OVRFW::VRMenuObject *contactsLabel_ = nullptr;
  OVRFW::VRMenuObject *objectsLabel_ = nullptr;
  OVRFW::VRMenuObject *recordingLabel_ = nullptr;
  OVRFW::VRMenuObject *controlLabel_ = nullptr;
  OVRFW::VRMenuObject *latencyLabel_ = nullptr;
  OVRFW::VRMenuObject *diagnosticsButton_ = nullptr;
  OVRFW::VRMenuObject *remoteLabel_=nullptr,*remoteInspectButton_=nullptr,*remoteScaleButton_=nullptr;
  OVRFW::VRMenuObject *remoteDemoModeButton_=nullptr;
  OVRFW::VRMenuObject *cameraLabel_=nullptr,*cameraModeButton_=nullptr;
  std::size_t remoteDemoMode_=0;
  OVRFW::VRMenuObject *noticeLabel_ = nullptr;
  OVRFW::VRMenuObject *gripperHoldButton_=nullptr,*gripperStatusLabel_=nullptr;
  OVRFW::VRMenuObject *contactProfileButton_=nullptr;
  Tab activeTab_ = Tab::Simulation;
  std::string localDiagnostic_;
  bool initialized_ = false;
  bool open_ = false;
  bool hasInputFocus_ = false;
  bool widgetsBuilt_ = true;
};

bool SimMenu::Impl::Init(const xrJava *context, OVRFW::ovrFileSys *fileSys,
                         const SimSettings &initialSettings,
                         SimMenuCallbacks callbacks) {
  if (initialized_) {
    return true;
  }
  std::string error;
  if (context == nullptr || fileSys == nullptr || !callbacks.applySettings ||
      !callbacks.command || !ValidateSettings(initialSettings, error)) {
    return false;
  }

  callbacks_ = std::move(callbacks);
  settings_ = initialSettings;
  if (!ui_.Init(context, fileSys, true, 64 * 1024)) {
    callbacks_ = {};
    return false;
  }

  // TinyUI's localized default is a bare "efigs.fnt". GuiSys rewrites bare
  // font names to apk://font/res/raw/..., whose `font` host is the installed
  // System Activities font package rather than this APK. The menu packages its
  // matching .fnt/.ktx pair in res/raw, so reload from the current-APK host.
  auto &font = ui_.GetGuiSys().GetDefaultFont();
  if (!font.Load(*fileSys, kApplicationFontUri)) {
    ui_.Shutdown();
    callbacks_ = {};
    return false;
  }
  const float probeWidth = font.CalcTextWidth("Newton");
  if (!std::isfinite(probeWidth) || probeWidth <= 0.0F) {
    ui_.Shutdown();
    callbacks_ = {};
    return false;
  }
  __android_log_print(ANDROID_LOG_INFO, "QuestNewton",
                      "QUEST_FULL_UI_FONT uri=%s probe_width=%g",
                      kApplicationFontUri, static_cast<double>(probeWidth));

  initialized_ = true;
  // Surface alpha reveals some passthrough in local mode without fading text.
  ui_.BackgroundColor={.015F,.025F,.08F,.60F};
  ui_.HoverColor={.025F,.24F,.32F,.78F};
  ui_.HighlightColor={.06F,.42F,.18F,.86F};
  widgetsBuilt_ = true;
  BuildWidgets();
  if (!widgetsBuilt_) {
    Shutdown();
    return false;
  }
  RefreshText();
  RefreshVisibility();
  return true;
}

void SimMenu::Impl::Shutdown() {
  if (!initialized_) {
    return;
  }
  open_ = false;
  hasInputFocus_ = false;
  ui_.HitTestDevices().clear();
  ui_.Shutdown();
  widgets_.clear();
  tabButtons_.fill(nullptr);
  settingValueButtons_.fill(nullptr);
  bindingButtons_.fill(nullptr);
  pauseButton_ = nullptr;
  ratesLabel_ = nullptr;
  followButton_ = nullptr;
  gridButton_ = nullptr;
  roomCollisionsButton_ = nullptr;
  roomSurfacesButton_ = nullptr;
  roomStatusLabel_ = nullptr;
  backendLabel_ = nullptr;
  physicsCpuLabel_ = nullptr;
  gpuLabel_ = nullptr;
  contactsLabel_ = nullptr;
  objectsLabel_=recordingLabel_=controlLabel_=latencyLabel_=diagnosticsButton_=nullptr;
  remoteLabel_=remoteInspectButton_=remoteScaleButton_=nullptr;
  remoteDemoModeButton_=nullptr;remoteDemoMode_=0;
  noticeLabel_ = nullptr;
  gripperHoldButton_=gripperStatusLabel_=nullptr;
  contactProfileButton_=nullptr;
  callbacks_ = {};
  initialized_ = false;
}

void SimMenu::Impl::SetVisible(bool visible, const OVR::Posef &stageFromHead) {
  if (!initialized_) {
    return;
  }
  if (visible) {
    PlaceWidgets(stageFromHead);
  }
  open_ = visible;
  if (!open_) {
    // A button callback can close the menu inside TinyUI's ray iteration.
    // Clear visuals now, but defer changing that vector until Update returns.
    if(pointers_)pointers_->Update({});
  }
  RefreshVisibility();
}

void SimMenu::Impl::Toggle(const OVR::Posef &stageFromHead) {
  SetVisible(!open_, stageFromHead);
}

void SimMenu::Impl::SetSettings(const SimSettings &settings) {
  std::string error;
  if (!ValidateSettings(settings, error)) {
    localDiagnostic_ = std::move(error);
  } else {
    settings_ = settings;
  }
  RefreshText();
}

void SimMenu::Impl::SetStatus(const SimMenuStatus &status) {
  const bool visibility_changed=status_.placing_object!=status.placing_object || status_.remote_inspection!=status.remote_inspection;
  if(status_.remote_inspection!=status.remote_inspection){
    const auto depth=OVRFW::VRMenuObjectFlags_t(OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH);
    const auto mask=OVRFW::VRMenuObjectFlags_t(OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH_MASK);
    for(const auto& widget:widgets_)if(widget.object){
      auto flags=widget.object->GetFlags();flags&=~(depth|mask);
      if(status.remote_inspection || widget.original_no_depth)flags|=depth;
      if(status.remote_inspection || widget.original_no_depth_mask)flags|=mask;
      widget.object->SetFlags(flags);
    }
  }
  status_ = status;
  RefreshText();
  if(visibility_changed)RefreshVisibility();
}

void SimMenu::Impl::Update(const OVRFW::ovrApplFrameIn &in,
                           const OVR::Posef &leftAim, bool leftAimValid,
                           bool leftClick, const OVR::Posef &rightAim,
                           bool rightAimValid, bool rightClick,
                           bool hasInputFocus) {
  if (!initialized_) {
    return;
  }
  hasInputFocus_ = hasInputFocus;
  ui_.HitTestDevices().clear();
  if (open_ && hasInputFocus_) {
    if (leftAimValid) {
      ui_.AddHitTestRay(leftAim, leftClick, 0);
    }
    if (rightAimValid) {
      ui_.AddHitTestRay(rightAim, rightClick, 1);
    }
  }
  ui_.Update(in);
  if(!open_)ui_.HitTestDevices().clear();
  if(pointers_)pointers_->Update(open_ && hasInputFocus_?
      std::span<const OVRFW::TinyUI::HitTestDevice>{ui_.HitTestDevices()}:
      std::span<const OVRFW::TinyUI::HitTestDevice>{});
}

void SimMenu::Impl::Render(const OVRFW::ovrApplFrameIn &in,
                           OVRFW::ovrRendererOutput &out) {
  if (initialized_ && (open_ || status_.placing_object || status_.remote_inspection)) {
    const auto first=out.Surfaces.size();
    ui_.Render(in, out);
    ui_surfaces_.resize(out.Surfaces.size()-first);
    for(std::size_t i=0;i<ui_surfaces_.size();++i){
      auto& surface=ui_surfaces_[i];surface=*out.Surfaces[first+i].surface;
      auto& state=surface.graphicsCommand.GpuState;
      if(state.blendEnable==OVRFW::ovrGpuState::BLEND_ENABLE){
        // Preserve SDK RGB blending, but accumulate coverage as source-over.
        // Shared SRC_ALPHA factors incorrectly square panel alpha and punch
        // passthrough holes through the opaque remote inspection background.
        state.blendEnable=OVRFW::ovrGpuState::BLEND_ENABLE_SEPARATE;
        state.blendSrcAlpha=OVRFW::ovrGpuState::kGL_ONE;
        state.blendDstAlpha=OVRFW::ovrGpuState::kGL_ONE_MINUS_SRC_ALPHA;
      }
      out.Surfaces[first+i].surface=&surface;
    }
    // Draw the same resolved hits after panel/text surfaces, including the
    // remote inspection overlay. Closed or unfocused menus never show rays.
    if(open_ && hasInputFocus_ && pointers_)pointers_->Render(in,out);
  }
}

OVRFW::VRMenuObject *SimMenu::Impl::AddLabel(const std::string &text,
                                             const OVR::Vector3f &localPosition,
                                             const OVR::Vector2f &size,
                                             int tab) {
  auto *object = ui_.AddLabel(text, {}, size);
  if (object == nullptr) {
    widgetsBuilt_ = false;
    return nullptr;
  }
  SetFittedText(object, text);
  object->SetSurfaceColor(0,ui_.BackgroundColor);
  widgets_.push_back({object, localPosition, tab,
      object->GetFlags() & OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH,
      object->GetFlags() & OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH_MASK});
  return object;
}

OVRFW::VRMenuObject *SimMenu::Impl::AddButton(
    const std::string &text, const OVR::Vector3f &localPosition,
    const OVR::Vector2f &size, std::function<void()> handler, int tab) {
  auto *object = ui_.AddButton(text, {}, size, std::move(handler));
  if (object == nullptr) {
    widgetsBuilt_ = false;
    return nullptr;
  }
  SetFittedText(object, text);
  object->SetSurfaceColor(0,ui_.BackgroundColor);
  widgets_.push_back({object, localPosition, tab,
      object->GetFlags() & OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH,
      object->GetFlags() & OVRFW::VRMENUOBJECT_FLAG_NO_DEPTH_MASK});
  return object;
}

OVRFW::VRMenuObject *SimMenu::Impl::AddStepper(const std::string &label,
                                               SettingField field, float y,
                                               int tab, float left,
                                               float width) {
  const float labelWidth = width > 0.0F ? width : 140.0F;
  AddLabel(label, {left, y, 0.0F}, {labelWidth, 38.0F}, tab);
  AddButton(
      "-", {left + 0.20F, y, 0.0F}, {36.0F, 38.0F},
      [this, field]() { AdjustSetting(field, -1); }, tab);
  auto *value = AddButton(
      "", {left + 0.32F, y, 0.0F}, {64.0F, 38.0F},
      [this, field]() { ResetSetting(field); }, tab);
  AddButton(
      "+", {left + 0.44F, y, 0.0F}, {36.0F, 38.0F},
      [this, field]() { AdjustSetting(field, 1); }, tab);
  settingValueButtons_[FieldIndex(field)] = value;
  return value;
}

void SimMenu::Impl::BuildWidgets() {
  AddLabel("Newton robot scene", {0.0F, 0.34F, 0.0F}, {280.0F, 46.0F});
  for (std::size_t i = 0; i < kTabNames.size(); ++i) {
    tabButtons_[i] =
        AddButton(kTabNames[i], {kTabX[i], 0.25F, 0.0F}, {kTabWidth, 40.0F},
                  [this, i]() { SelectTab(static_cast<Tab>(i)); });
  }

  const int simulation = TabIndex(Tab::Simulation);
  AddButton(
      "Reset scene", {-0.16F, 0.15F, 0.0F}, {140.0F, 42.0F},
      [this]() { RunCommand(SimMenuCommand::Reset); }, simulation);
  pauseButton_ = AddButton(
      "Pause", {0.16F, 0.15F, 0.0F}, {140.0F, 42.0F},
      [this]() { RunCommand(SimMenuCommand::TogglePause); }, simulation);
  AddStepper("Physics dt (s)", SettingField::PhysicsDt, 0.06F, simulation);
  AddStepper("Control steps", SettingField::ControlDecimation, -0.02F,
             simulation);
  AddStepper("Render steps", SettingField::RenderInterval, -0.10F, simulation);
  ratesLabel_ = AddLabel("", {0.0F, -0.20F, 0.0F}, {370.0F, 42.0F}, simulation);

  const int controls = TabIndex(Tab::Controls);
  AddStepper("Palm roll deg", SettingField::HandRoll, 0.15F, controls, -0.42F,
             105.0F);
  AddStepper("Palm pitch deg", SettingField::HandPitch, 0.06F, controls, -0.42F,
             105.0F);
  AddStepper("Palm yaw deg", SettingField::HandYaw, -0.03F, controls, -0.42F,
             105.0F);
  AddLabel("Bindings cycle; conflicts swap", {-0.29F, -0.14F, 0.0F},
           {210.0F, 38.0F}, controls);
  for (std::size_t i = 0; i < kActionCount; ++i) {
    const float y = 0.15F - static_cast<float>(i) * 0.08F;
    bindingButtons_[i] = AddButton(
        "", {0.30F, y, 0.0F}, {230.0F, 38.0F}, [this, i]() { CycleBinding(i); },
        controls);
  }

  const int gripper = TabIndex(Tab::Gripper);
  contactProfileButton_=AddButton("",{0,.155F,0},{450.F,36.F},[this](){CycleContactProfile();},gripper);
  gripperHoldButton_=AddButton("",{0,.074F,0},{310.F,36.F},
      [this](){ToggleSetting(&SimSettings::gripper_force_hold);},gripper);
  AddStepper("Finger speed mm/s",SettingField::GripperSpeed,-.006F,gripper);
  AddStepper("Force N / finger",SettingField::GripperForce,-.086F,gripper);
  AddLabel("Force at full squeeze; actuator limit 20 N/finger",{0,-.153F,0},{450.F,24.F},gripper);
  gripperStatusLabel_=AddLabel("",{0,-.217F,0},{450.F,40.F},gripper);

  const int placement = TabIndex(Tab::Placement);
  AddStepper("Shoulder offset m", SettingField::ShoulderOffset, 0.14F,
             placement);
  followButton_ = AddButton(
      "", {-0.20F, 0.04F, 0.0F}, {170.0F, 42.0F},
      [this]() { ToggleSetting(&SimSettings::follow_height); }, placement);
  AddStepper("Robot opacity", SettingField::Opacity, -0.06F, placement);
  gridButton_ = AddButton(
      "", {-0.20F, -0.16F, 0.0F}, {170.0F, 42.0F},
      [this]() { ToggleSetting(&SimSettings::show_grid); }, placement);
  AddButton(
      "Recenter placement", {0.20F, -0.16F, 0.0F}, {170.0F, 42.0F},
      [this]() { RunCommand(SimMenuCommand::Recenter); }, placement);

  const int room = TabIndex(Tab::Room);
  roomCollisionsButton_ = AddButton(
      "", {0.0F, 0.14F, 0.0F}, {300.0F, 42.0F},
      [this]() { ToggleSetting(&SimSettings::environment_collisions); }, room);
  roomSurfacesButton_ = AddButton(
      "", {0.0F, 0.04F, 0.0F}, {300.0F, 42.0F},
      [this]() { ToggleSetting(&SimSettings::show_room_surfaces); }, room);
  AddButton(
      "Refresh room", {-0.18F, -0.06F, 0.0F}, {160.0F, 42.0F},
      [this]() { RunCommand(SimMenuCommand::RefreshRoom); }, room);
  AddButton(
      "Scan room", {0.18F, -0.06F, 0.0F}, {160.0F, 42.0F},
      [this]() { RunCommand(SimMenuCommand::ScanRoom); }, room);
  roomStatusLabel_ = AddLabel("", {0.0F, -0.19F, 0.0F}, {400.0F, 62.0F}, room);

  const int objects=TabIndex(Tab::Objects);
  AddButton("Place cube",{-0.20F,0.15F,0.0F},{170.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::PlaceCube);},objects);
  AddButton("Next cube",{0.20F,0.15F,0.0F},{170.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::SelectNextCube);},objects);
  AddButton("Move selected",{-0.20F,0.04F,0.0F},{170.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::MoveCube);},objects);
  AddButton("Reset selected",{0.20F,0.04F,0.0F},{170.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::ResetCube);},objects);
  AddButton("Delete selected",{0.0F,-0.07F,0.0F},{220.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::DeleteCube);},objects);
  objectsLabel_=AddLabel("",{0.0F,-0.20F,0.0F},{420.0F,68.0F},objects);

  const int debug = TabIndex(Tab::Debug);
  AddButton(
      "Spawn box", {-0.28F, 0.15F, 0.0F}, {120.0F, 40.0F},
      [this]() { RunCommand(SimMenuCommand::SpawnBox); }, debug);
  AddButton(
      "Remove box", {-0.02F, 0.15F, 0.0F}, {120.0F, 40.0F},
      [this]() { RunCommand(SimMenuCommand::RemoveBox); }, debug);
  AddButton(
      "Restore defaults", {0.26F, 0.15F, 0.0F}, {160.0F, 40.0F},
      [this]() { RestoreDefaults(); }, debug);
  backendLabel_ = AddLabel("", {0.0F, 0.06F, 0.0F}, {400.0F, 38.0F}, debug);
  physicsCpuLabel_ = AddLabel("", {0.0F, -0.02F, 0.0F}, {400.0F, 38.0F}, debug);
  gpuLabel_ = AddLabel("", {0.0F, -0.10F, 0.0F}, {400.0F, 38.0F}, debug);
  contactsLabel_ = AddLabel("", {0.0F, -0.18F, 0.0F}, {400.0F, 38.0F}, debug);

  const int record=TabIndex(Tab::Record);
  AddButton("Record + reset",{-0.20F,0.15F,0.0F},{180.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::RecordReset);},record);
  AddButton("Stop and save",{0.20F,0.15F,0.0F},{180.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::StopRecording);},record);
  diagnosticsButton_=AddButton("",{0.0F,0.05F,0.0F},{300.0F,42.0F},
      [this](){RunCommand(SimMenuCommand::ToggleDiagnostics);},record);
  recordingLabel_=AddLabel("",{0.0F,-0.05F,0.0F},{420.0F,38.0F},record);
  controlLabel_=AddLabel("",{0.0F,-0.13F,0.0F},{420.0F,38.0F},record);
  latencyLabel_=AddLabel("",{0.0F,-0.21F,0.0F},{420.0F,38.0F},record);

  const int remote=TabIndex(Tab::Remote);
  remoteDemoModeButton_=AddButton("",{0,.22F,0},{350.F,30.F},[this](){remoteDemoMode_=(remoteDemoMode_+1)%5;RefreshText();},remote);
  AddButton("Load demo",{-.30F,.13F,0},{125.F,40.F},[this](){
      const std::array<SimMenuCommand,5> modes{SimMenuCommand::RemoteDemoPrepared,SimMenuCommand::RemoteDemoUnprepared,SimMenuCommand::RemoteDemo,
          SimMenuCommand::RemoteDemoRgbPrepared,SimMenuCommand::RemoteDemoRgbUnprepared};
      RunCommand(modes[remoteDemoMode_]);},remote);
  AddButton("Connect",{0,.13F,0},{125.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteConnect);},remote);
  AddButton("Disconnect",{.30F,.13F,0},{125.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteDisconnect);},remote);
  remoteInspectButton_=AddButton("",{-.20F,.05F,0},{180.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteInspect);},remote);
  AddButton("Align view",{.20F,.05F,0},{180.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteAlign);},remote);
  AddButton("View back",{-.28F,-.05F,0},{125.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteBackward);},remote);
  remoteScaleButton_=AddButton("",{0,-.05F,0},{125.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteScale);},remote);
  AddButton("View forward",{.28F,-.05F,0},{125.F,40.F},[this](){RunCommand(SimMenuCommand::RemoteForward);},remote);
  remoteLabel_=AddLabel("",{0,-.19F,0},{450.F,85.F},remote);

  const int camera=TabIndex(Tab::Camera);
  AddLabel("Remote camera - simulation only",{0,.23F,0},{420.F,30.F},camera);
  AddButton("Connect camera",{-.22F,.14F,0},{185.F,40.F},[this](){RunCommand(SimMenuCommand::GimbalConnect);},camera);
  AddButton("Disconnect",{.22F,.14F,0},{185.F,40.F},[this](){RunCommand(SimMenuCommand::GimbalDisconnect);},camera);
  cameraModeButton_=AddButton("",{0,.04F,0},{390.F,40.F},[this](){RunCommand(SimMenuCommand::GimbalMode);},camera);
  AddButton("Clear retained remote map",{0,-.05F,0},{390.F,35.F},[this](){RunCommand(SimMenuCommand::GimbalResetMap);},camera);
  cameraLabel_=AddLabel("",{0,-.19F,0},{450.F,85.F},camera);

  noticeLabel_ = AddLabel("", {0.0F, -0.36F, 0.0F}, {420.0F, 100.0F});
}

void SimMenu::Impl::PlaceWidgets(const OVR::Posef &stageFromHead) {
  OVR::Vector3f forward =
      stageFromHead.Rotation * OVR::Vector3f{0.0F, 0.0F, -1.0F};
  forward.y = 0.0F;
  if (forward.LengthSq() < 1.0e-6F) {
    forward = {0.0F, 0.0F, -1.0F};
  } else {
    forward.Normalize();
  }
  const float yaw = std::atan2(-forward.x, -forward.z);
  const OVR::Quatf facing(OVR::Axis_Y, yaw);
  const OVR::Vector3f center = stageFromHead.Translation + forward;
  const OVR::Posef anchor(facing, center);
  for (const Widget &widget : widgets_) {
    if (widget.object != nullptr) {
      widget.object->SetLocalPose(
          OVR::Posef(facing, anchor.Transform(widget.localPosition)));
    }
  }
}

void SimMenu::Impl::RefreshVisibility() {
  const int active = TabIndex(activeTab_);
  for (const Widget &widget : widgets_) {
    if (widget.object != nullptr) {
      widget.object->SetVisible((open_ && (widget.tab < 0 || widget.tab == active)) ||
                                (!open_ && (status_.placing_object || status_.remote_inspection) && widget.object==noticeLabel_));
    }
  }
}

void SimMenu::Impl::RefreshText() {
  const std::array<const char*,3> profiles{"Original mesh","Five pads","Pads + friction"};
  const auto profile=static_cast<std::size_t>(settings_.contact_profile);
  SetFittedText(contactProfileButton_,std::string(profile<profiles.size()?profiles[profile]:"Unknown model")+
      (settings_.contact_profile==ContactProfile::OriginalMesh?" (change resets scene)":" (experimental; resets scene)"));
  SetFittedText(gripperHoldButton_,settings_.gripper_force_hold?"Contact force hold: on":"Contact force hold: off");
  SetWrappedLabel(gripperStatusLabel_,OneLine(status_.gripper),2);
  if (!initialized_) {
    return;
  }
  for (std::size_t i = 0; i < tabButtons_.size(); ++i) {
    if (tabButtons_[i] != nullptr) {
      const bool selected = i == ToIndex(activeTab_);
      SetFittedText(tabButtons_[i],
                    selected ? ">" + std::string(kTabNames[i]) : kTabNames[i]);
    }
  }
  for (std::size_t i = 0; i < settingValueButtons_.size(); ++i) {
    if (settingValueButtons_[i] != nullptr) {
      SetFittedText(settingValueButtons_[i],
                    SettingValue(settings_, static_cast<SettingField>(i)));
    }
  }
  if (pauseButton_ != nullptr) {
    SetFittedText(pauseButton_, status_.paused ? "Resume" : "Pause");
  }
  if (ratesLabel_ != nullptr) {
    const std::string text =
        std::string(status_.settings_pending ? "Selected" : "Configured") +
        " control " + Fixed(settings_.ControlHz(), 1) + " Hz | scene " +
        Fixed(settings_.SceneHz(), 1) + " Hz";
    SetFittedText(ratesLabel_, text);
  }
  if (followButton_ != nullptr) {
    SetFittedText(followButton_, settings_.follow_height
                                     ? "Height follow: on"
                                     : "Height follow: off");
  }
  if (gridButton_ != nullptr) {
    SetFittedText(gridButton_, settings_.show_grid ? "Floor grid: shown"
                                                   : "Floor grid: hidden");
  }
  if (roomCollisionsButton_ != nullptr) {
    SetFittedText(roomCollisionsButton_, settings_.environment_collisions
                                            ? "Environment collisions: on"
                                            : "Environment collisions: off");
  }
  if (roomSurfacesButton_ != nullptr) {
    SetFittedText(roomSurfacesButton_, settings_.show_room_surfaces
                                          ? "Show room surfaces: on"
                                          : "Show room surfaces: off");
  }
  SetWrappedLabel(roomStatusLabel_, OneLine(status_.room), 3);
  SetWrappedLabel(objectsLabel_,OneLine(status_.objects),3);
  SetFittedText(recordingLabel_,OneLine(status_.recording));
  SetFittedText(controlLabel_,OneLine(status_.control));
  SetFittedText(latencyLabel_,OneLine(status_.latency));
  SetFittedText(diagnosticsButton_,status_.show_diagnostics?"Contact / target overlays: on":"Contact / target overlays: off");
  SetWrappedLabel(remoteLabel_,OneLine(status_.remote),4);
  SetFittedText(remoteInspectButton_,status_.remote_inspection?"Exit inspection":"Enter inspection");
  const std::array<const char*,5> demoNames{"Demo: robot-built surfaces","Demo: Quest-built surfaces","Demo: grayscale points",
      "Demo: retained RGB / robot-built","Demo: retained RGB / Quest-built"};
  SetFittedText(remoteDemoModeButton_,demoNames[remoteDemoMode_]);
  SetFittedText(remoteScaleButton_,"Scale "+Fixed(status_.remote_scale,2)+"x");
  const std::array<const char*,3> cameraModes{"Aim: off","Aim: head + left trigger","Aim: right controller + left trigger"};
  SetFittedText(cameraModeButton_,cameraModes[std::min(status_.camera_mode,2u)]);
  SetWrappedLabel(cameraLabel_,OneLine(status_.camera),4);
  for (std::size_t i = 0; i < bindingButtons_.size(); ++i) {
    if (bindingButtons_[i] != nullptr) {
      const auto action = static_cast<SimAction>(i);
      const std::string text = std::string(ActionName(action)) + ": " +
                               std::string(InputName(settings_.bindings[i]));
      SetFittedText(bindingButtons_[i], text);
    }
  }
  if (backendLabel_ != nullptr) {
    SetFittedText(backendLabel_, OneLine(status_.backend));
  }
  if (physicsCpuLabel_ != nullptr) {
    SetFittedText(physicsCpuLabel_, OneLine(status_.physicsCpu));
  }
  if (gpuLabel_ != nullptr) {
    SetFittedText(gpuLabel_, OneLine(status_.gpu));
  }
  if (contactsLabel_ != nullptr) {
    SetFittedText(contactsLabel_, OneLine(status_.contacts));
  }
  if (noticeLabel_ != nullptr) {
    const std::string notice = status_.remote_inspection
        ? "INSPECTION ONLY - robot controls held. "+status_.remote+". Menu opens settings."
        : !localDiagnostic_.empty() ? localDiagnostic_ : status_.diagnostic;
    SetWrappedNotice(notice.empty() ? "Aim and click with either controller"
                                    : notice);
  }
}

void SimMenu::Impl::SetFittedText(OVRFW::VRMenuObject *object,
                                  const std::string &text) {
  if (object == nullptr) {
    return;
  }
  constexpr float kMaximumScale = 0.50F;
  constexpr float kMinimumReadableScale = 0.30F;
  constexpr float kHorizontalPadding = 0.90F;
  const float panelWidth =
      object->GetSurfaceDims(0).x * OVRFW::VRMenuObject::DEFAULT_TEXEL_SCALE;
  const float textWidth = ui_.GetGuiSys().GetDefaultFont().CalcTextWidth(
      text.empty() ? " " : text.c_str());
  auto font = object->GetFontParms();
  font.Scale = kMaximumScale;
  if (std::isfinite(textWidth) && textWidth > 0.0F) {
    font.Scale = std::clamp(panelWidth * kHorizontalPadding / textWidth,
                            kMinimumReadableScale, kMaximumScale);
  }
  font.WrapWidth = -1.0F;
  font.MaxLines = 1;
  font.MultiLine = false;
  object->SetFontParms(font);
  object->SetText(text.c_str());
}

void SimMenu::Impl::SetWrappedNotice(const std::string &text) {
  SetWrappedLabel(noticeLabel_, text, 4);
}

void SimMenu::Impl::SetWrappedLabel(OVRFW::VRMenuObject *object,
                                     const std::string &text, int maxLines) {
  if (object == nullptr) {
    return;
  }
  constexpr float kReadableScale = 0.32F;
  constexpr float kHorizontalPadding = 0.90F;
  auto font = object->GetFontParms();
  font.Scale = kReadableScale;
  font.WrapWidth = object->GetSurfaceDims(0).x *
                   OVRFW::VRMenuObject::DEFAULT_TEXEL_SCALE *
                   kHorizontalPadding;
  font.MaxLines = maxLines;
  font.MultiLine = true;
  object->SetFontParms(font);
  object->SetText(BoundedLine(text, 180).c_str());
}

void SimMenu::Impl::SelectTab(Tab tab) {
  activeTab_ = tab;
  RefreshText();
  RefreshVisibility();
}

void SimMenu::Impl::AdjustSetting(SettingField field, int direction) {
  SimSettings candidate = settings_;
  switch (field) {
  case SettingField::PhysicsDt: {
    const int tick = std::clamp(
        static_cast<int>(std::lround(candidate.physics_dt / 0.0005)) +
            direction,
        1, 40);
    candidate.physics_dt = static_cast<double>(tick) * 0.0005;
    break;
  }
  case SettingField::ControlDecimation: {
    const int value = std::clamp(
        static_cast<int>(candidate.control_decimation) + direction, 1, 40);
    candidate.control_decimation = static_cast<std::uint32_t>(value);
    break;
  }
  case SettingField::RenderInterval: {
    const int value = std::clamp(
        static_cast<int>(candidate.render_interval) + direction, 1, 40);
    candidate.render_interval = static_cast<std::uint32_t>(value);
    break;
  }
  case SettingField::HandRoll:
  case SettingField::HandPitch:
  case SettingField::HandYaw: {
    const std::size_t index =
        FieldIndex(field) - FieldIndex(SettingField::HandRoll);
    candidate.hand_offset_degrees[index] = std::clamp(
        candidate.hand_offset_degrees[index] + static_cast<float>(direction),
        -180.0F, 180.0F);
    break;
  }
  case SettingField::ShoulderOffset: {
    const int tick = std::clamp(
        static_cast<int>(std::lround(candidate.shoulder_offset_m * 100.0F)) +
            direction,
        0, 100);
    candidate.shoulder_offset_m = static_cast<float>(tick) * 0.01F;
    break;
  }
  case SettingField::Opacity: {
    const int tick = std::clamp(
        static_cast<int>(std::lround(candidate.opacity * 20.0F)) + direction, 1,
        20);
    candidate.opacity = static_cast<float>(tick) * 0.05F;
    break;
  }
  case SettingField::GripperSpeed: {
    const int tick=std::clamp(static_cast<int>(std::lround(candidate.gripper_speed_mps/.005))+direction,1,40);
    candidate.gripper_speed_mps=static_cast<double>(tick)*.005;
    break;
  }
  case SettingField::GripperForce: {
    const int tick=std::clamp(static_cast<int>(std::lround(candidate.gripper_force_n/.5))+direction,1,40);
    candidate.gripper_force_n=static_cast<double>(tick)*.5;
    break;
  }
  }
  ApplyCandidate(candidate);
}

void SimMenu::Impl::ResetSetting(SettingField field) {
  const SimSettings defaults;
  SimSettings candidate = settings_;
  switch (field) {
  case SettingField::PhysicsDt:
    candidate.physics_dt = defaults.physics_dt;
    break;
  case SettingField::ControlDecimation:
    candidate.control_decimation = defaults.control_decimation;
    break;
  case SettingField::RenderInterval:
    candidate.render_interval = defaults.render_interval;
    break;
  case SettingField::HandRoll:
  case SettingField::HandPitch:
  case SettingField::HandYaw: {
    const std::size_t index =
        FieldIndex(field) - FieldIndex(SettingField::HandRoll);
    candidate.hand_offset_degrees[index] = defaults.hand_offset_degrees[index];
    break;
  }
  case SettingField::ShoulderOffset:
    candidate.shoulder_offset_m = defaults.shoulder_offset_m;
    break;
  case SettingField::Opacity:
    candidate.opacity = defaults.opacity;
    break;
  case SettingField::GripperSpeed:
    candidate.gripper_speed_mps=defaults.gripper_speed_mps;
    break;
  case SettingField::GripperForce:
    candidate.gripper_force_n=defaults.gripper_force_n;
    break;
  }
  ApplyCandidate(candidate);
}

void SimMenu::Impl::ToggleSetting(bool SimSettings::*member) {
  SimSettings candidate = settings_;
  candidate.*member = !(candidate.*member);
  ApplyCandidate(candidate);
}
void SimMenu::Impl::CycleContactProfile() {
  SimSettings candidate=settings_;
  candidate.contact_profile=static_cast<ContactProfile>((static_cast<unsigned>(candidate.contact_profile)+1)%3);
  ApplyCandidate(candidate);
}

void SimMenu::Impl::CycleBinding(std::size_t actionIndex) {
  if (actionIndex >= kActionCount) {
    return;
  }
  SimSettings candidate = settings_;
  const InputId previous = candidate.bindings[actionIndex];
  const std::size_t next =
      (static_cast<std::size_t>(previous) + 1) % kInputCount;
  const InputId selected = static_cast<InputId>(next);
  for (std::size_t i = 0; i < kActionCount; ++i) {
    if (i != actionIndex && candidate.bindings[i] == selected) {
      candidate.bindings[i] = previous;
      break;
    }
  }
  candidate.bindings[actionIndex] = selected;
  ApplyCandidate(candidate);
}

bool SimMenu::Impl::ApplyCandidate(const SimSettings &candidate) {
  std::string error;
  if (!ValidateSettings(candidate, error)) {
    localDiagnostic_ = std::move(error);
    RefreshText();
    return false;
  }
  if (!callbacks_.applySettings(candidate, error)) {
    localDiagnostic_ =
        error.empty() ? "Settings update rejected" : std::move(error);
    RefreshText();
    return false;
  }
  settings_ = candidate;
  localDiagnostic_.clear();
  RefreshText();
  return true;
}

void SimMenu::Impl::RunCommand(SimMenuCommand command) {
  callbacks_.command(command);
}

void SimMenu::Impl::RestoreDefaults() {
  if (ApplyCandidate(SimSettings{})) {
    callbacks_.command(SimMenuCommand::RestoreDefaults);
  }
}

std::string SimMenu::Impl::SettingValue(const SimSettings &settings,
                                        SettingField field) {
  switch (field) {
  case SettingField::PhysicsDt:
    return Fixed(settings.physics_dt, 4);
  case SettingField::ControlDecimation:
    return std::to_string(settings.control_decimation);
  case SettingField::RenderInterval:
    return std::to_string(settings.render_interval);
  case SettingField::HandRoll:
  case SettingField::HandPitch:
  case SettingField::HandYaw: {
    const std::size_t index =
        FieldIndex(field) - FieldIndex(SettingField::HandRoll);
    return Fixed(settings.hand_offset_degrees[index], 0);
  }
  case SettingField::ShoulderOffset:
    return Fixed(settings.shoulder_offset_m, 2);
  case SettingField::Opacity:
    return Fixed(settings.opacity, 2);
  case SettingField::GripperSpeed:
    return Fixed(settings.gripper_speed_mps*1000.,0);
  case SettingField::GripperForce:
    return Fixed(settings.gripper_force_n,1);
  }
  return {};
}

std::string SimMenu::Impl::BoundedLine(std::string text,
                                       std::size_t maxLength) {
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  if (maxLength < 16) {
    return text.substr(0, maxLength);
  }
  if (text.size() > maxLength) {
    // Preserve the end of diagnostics, where recovery instructions such as
    // "Restore defaults" are normally placed.
    const std::size_t tailLength = std::min<std::size_t>(48, maxLength / 3);
    const std::string tail = text.substr(text.size() - tailLength);
    text.resize(maxLength - tailLength - 5);
    text += " ... ";
    text += tail;
  }
  return text;
}

std::string SimMenu::Impl::OneLine(std::string text) {
  return BoundedLine(std::move(text), 120);
}

SimMenu::SimMenu() : impl_(std::make_unique<Impl>()) {}

SimMenu::~SimMenu() { Shutdown(); }

bool SimMenu::Init(const xrJava *context, OVRFW::ovrFileSys *fileSys,
                   const SimSettings &initialSettings,
                   SimMenuCallbacks callbacks) {
  return impl_->Init(context, fileSys, initialSettings, std::move(callbacks));
}

void SimMenu::Shutdown() {
  if (impl_) {
    impl_->Shutdown();
  }
}

bool SimMenu::SessionInit() {
  SessionEnd();
  if(!impl_ || !impl_->initialized_)return false;
  auto pointers=std::make_unique<MenuPointerRenderer>();
  if(!pointers->Init())return false;
  impl_->pointers_=std::move(pointers);
  return true;
}

void SimMenu::SessionEnd() {
  if(!impl_)return;
  impl_->pointers_.reset();
  impl_->ui_surfaces_.clear();
  impl_->hasInputFocus_=false;
  impl_->ui_.HitTestDevices().clear();
}

void SimMenu::SetVisible(bool visible, const OVR::Posef &stageFromHead) {
  impl_->SetVisible(visible, stageFromHead);
}

void SimMenu::Toggle(const OVR::Posef &stageFromHead) {
  impl_->Toggle(stageFromHead);
}

bool SimMenu::IsOpen() const { return impl_ != nullptr && impl_->open_; }

void SimMenu::SetSettings(const SimSettings &settings) {
  impl_->SetSettings(settings);
}

void SimMenu::SetStatus(const SimMenuStatus &status) {
  impl_->SetStatus(status);
}

void SimMenu::Update(const OVRFW::ovrApplFrameIn &in, const OVR::Posef &leftAim,
                     bool leftAimValid, bool leftClick,
                     const OVR::Posef &rightAim, bool rightAimValid,
                     bool rightClick, bool hasInputFocus) {
  impl_->Update(in, leftAim, leftAimValid, leftClick, rightAim, rightAimValid,
                rightClick, hasInputFocus);
}

void SimMenu::Render(const OVRFW::ovrApplFrameIn &in,
                     OVRFW::ovrRendererOutput &out) {
  impl_->Render(in, out);
}

} // namespace quest_newton
