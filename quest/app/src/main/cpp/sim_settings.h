#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace quest_newton {
enum class InputId : std::uint8_t {
    RightTrigger, RightSqueeze, LeftTrigger, LeftSqueeze, A, B, X, Y,
    LeftMenu, LeftStickClick, RightStickClick, Count
};
enum class SimAction : std::uint8_t { ArmEngage, Gripper, Calibrate, HeightClutch, Menu, Count };
enum class ContactProfile : std::uint8_t { OriginalMesh, FivePads, PadsWithFriction };
inline constexpr std::size_t kInputCount = static_cast<std::size_t>(InputId::Count);
inline constexpr std::size_t kActionCount = static_cast<std::size_t>(SimAction::Count);
struct InputValue { float value=0; bool active=false; };
using InputValues = std::array<InputValue,kInputCount>;
using ActionValues = std::array<InputValue,kActionCount>;

struct SimSettings {
    double physics_dt=.005;
    std::uint32_t control_decimation=2;
    std::uint32_t render_interval=2;
    std::array<float,3> hand_offset_degrees{};
    float shoulder_offset_m=.25F;
    float opacity=.35F;
    bool follow_height=true;
    bool show_grid=true;
    bool floating_base=false;
    bool environment_collisions=false;
    bool show_room_surfaces=true;
    float gravity_scale=1;
    bool gripper_force_hold=false;
    double gripper_speed_mps=.05;
    double gripper_force_n=5;
    ContactProfile contact_profile=ContactProfile::OriginalMesh;
    std::array<InputId,kActionCount> bindings{
        InputId::RightTrigger, InputId::RightSqueeze, InputId::A, InputId::LeftTrigger, InputId::LeftMenu};
    bool operator==(const SimSettings&) const = default;
    double ControlHz() const { return 1.0/(physics_dt*control_decimation); }
    double SceneHz() const { return 1.0/(physics_dt*render_interval); }
};
bool ValidateSettings(const SimSettings& settings, std::string& error);
std::string_view InputName(InputId input);
std::string_view ActionName(SimAction action);
std::string_view ContactProfileName(ContactProfile profile);
ActionValues ResolveActions(const SimSettings& settings, const InputValues& inputs);
std::string SerializeSettings(const SimSettings& settings);
// Live files may omit new keys. QITR v1 means legacy direct gripper targets;
// v2 declares applied-gripper settings; v3 additionally requires contact model.
// Older traces imply Original mesh and forbid a contact_profile field.
enum class SettingsParseMode { Live, TraceV1, TraceV2, TraceV3 };
bool ParseSettings(std::string_view text, SimSettings& settings, std::string& error,
                   SettingsParseMode mode=SettingsParseMode::Live);
std::string SettingsJson(const SimSettings& settings);
bool LoadSettings(const std::filesystem::path& path, SimSettings& settings, std::string& error);
bool SaveSettings(const std::filesystem::path& path, const SimSettings& settings, std::string& error);
} // namespace quest_newton
