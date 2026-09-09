#include "sim_settings.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <sstream>
#include <system_error>
#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#endif

namespace quest_newton {
namespace {
constexpr std::array<std::string_view,kInputCount> kInputs{
    "Right trigger", "Right grip", "Left trigger", "Left grip", "A", "B", "X", "Y",
    "Left menu", "Left stick click", "Right stick click"};
constexpr std::array<std::string_view,kActionCount> kActions{
    "Arm control", "Gripper close", "Calibrate", "Height clutch", "Settings menu"};
constexpr std::array<std::string_view,3> kContactProfiles{"legacy_mesh_v1","five_pads_v1","pad_manipulation_v1"};
constexpr std::array<std::string_view,24> kKeys{
    "version", "physics_dt", "control_decimation", "render_interval", "hand_roll", "hand_pitch", "hand_yaw",
    "shoulder_offset_m", "opacity", "follow_height", "show_grid", "floating_base", "gravity_scale",
    "binding_arm", "binding_gripper", "binding_calibrate", "binding_height", "binding_menu",
    "environment_collisions", "show_room_surfaces", "gripper_force_hold", "gripper_speed_mps", "gripper_force_n", "contact_profile"};
bool FiniteRange(double v,double low,double high) { return std::isfinite(v) && v>=low && v<=high; }
std::string_view Trim(std::string_view value) {
    const auto first=value.find_first_not_of(" \t\r");
    if(first==value.npos) return {};
    return value.substr(first,value.find_last_not_of(" \t\r")-first+1);
}
bool Number(std::string_view text,double& value) {
    if(text.empty()) return false;
    // NDK r27 libc++ lacks floating-point from_chars. Use the classic locale
    // independently of Python/Android's process locale and reject trailing text.
    std::istringstream input{std::string(text)};input.imbue(std::locale::classic());
    if(!(input>>value) || !std::isfinite(value)) return false;
    input>>std::ws;return input.eof();
}
}
bool ValidateSettings(const SimSettings& s,std::string& error) {
    const auto reject=[&](const char* why){error=why;return false;};
    if(!FiniteRange(s.physics_dt,.0005,.02)) return reject("Physics dt must be 0.0005 to 0.02 seconds");
    if(s.control_decimation<1 || s.control_decimation>40) return reject("Control decimation must be 1 to 40");
    if(s.render_interval<1 || s.render_interval>40) return reject("Render interval must be 1 to 40");
    for(float angle:s.hand_offset_degrees) if(!FiniteRange(angle,-180,180)) return reject("Hand angles must be -180 to 180 degrees");
    if(!FiniteRange(s.shoulder_offset_m,0,1)) return reject("Shoulder offset must be 0 to 1 metre");
    if(!FiniteRange(s.opacity,.05,1)) return reject("Opacity must be 0.05 to 1");
    if(!FiniteRange(s.gravity_scale,0,2)) return reject("Gravity scale must be 0 to 2");
    if(!((s.gripper_speed_mps==0 && !s.gripper_force_hold) || FiniteRange(s.gripper_speed_mps,.005,.2)))
        return reject("Gripper speed must be 0.005 to 0.2 m/s, or zero with force hold off");
    if(!FiniteRange(s.gripper_force_n,.5,20)) return reject("Gripper force must be 0.5 to 20 N");
    if(ContactProfileName(s.contact_profile).empty())return reject("Unknown contact model");
    std::array<bool,kInputCount> used{};
    for(auto binding:s.bindings) {
        const auto id=static_cast<std::size_t>(binding);
        if(id>=kInputCount || used[id]) return reject("Each action needs a different supported input");
        used[id]=true;
    }
    error.clear(); return true;
}
std::string_view InputName(InputId input) {
    const auto i=static_cast<std::size_t>(input); return i<kInputs.size()?kInputs[i]:"Unavailable";
}
std::string_view ActionName(SimAction action) {
    const auto i=static_cast<std::size_t>(action); return i<kActions.size()?kActions[i]:"Unavailable";
}
std::string_view ContactProfileName(ContactProfile profile) {
    const auto i=static_cast<std::size_t>(profile);return i<kContactProfiles.size()?kContactProfiles[i]:std::string_view{};
}
ActionValues ResolveActions(const SimSettings& s,const InputValues& inputs) {
    ActionValues values{};
    for(std::size_t i=0;i<values.size();++i) {
        const auto id=static_cast<std::size_t>(s.bindings[i]);
        if(id>=inputs.size()) continue;
        auto value=inputs[id];
        if(!value.active || !FiniteRange(value.value,0,1)) continue;
        if(id>=static_cast<std::size_t>(InputId::A)) value.value=value.value>.5F?1.F:0.F;
        values[i]=value;
    }
    return values;
}
std::string SerializeSettings(const SimSettings& s) {
    std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(17);
    out<<"version=1\nphysics_dt="<<s.physics_dt<<"\ncontrol_decimation="<<s.control_decimation
       <<"\nrender_interval="<<s.render_interval;
    for(std::size_t i=0;i<3;++i) out<<'\n'<<kKeys[4+i]<<'='<<s.hand_offset_degrees[i];
    out<<"\nshoulder_offset_m="<<s.shoulder_offset_m<<"\nopacity="<<s.opacity
       <<"\nfollow_height="<<s.follow_height<<"\nshow_grid="<<s.show_grid
       <<"\nfloating_base="<<s.floating_base<<"\ngravity_scale="<<s.gravity_scale;
    for(std::size_t i=0;i<kActionCount;++i) out<<'\n'<<kKeys[13+i]<<'='<<static_cast<unsigned>(s.bindings[i]);
    out<<"\nenvironment_collisions="<<s.environment_collisions<<"\nshow_room_surfaces="<<s.show_room_surfaces
       <<"\ngripper_force_hold="<<s.gripper_force_hold<<"\ngripper_speed_mps="<<s.gripper_speed_mps
       <<"\ngripper_force_n="<<s.gripper_force_n<<"\ncontact_profile="<<ContactProfileName(s.contact_profile);
    out<<'\n'; return out.str();
}
bool ParseSettings(std::string_view text,SimSettings& s,std::string& error,SettingsParseMode mode) {
    const auto reject=[&](const char* why){error=why;return false;};
    if(text.size()>65536) return reject("Settings file is too large");
    if(mode!=SettingsParseMode::Live && mode!=SettingsParseMode::TraceV1 && mode!=SettingsParseMode::TraceV2 && mode!=SettingsParseMode::TraceV3)
        return reject("Unknown settings parse mode");
    SimSettings candidate;
    if(mode==SettingsParseMode::TraceV1 || mode==SettingsParseMode::TraceV2)candidate.contact_profile=ContactProfile::OriginalMesh;
    if(mode==SettingsParseMode::TraceV1) {
        candidate.gripper_force_hold=false;candidate.gripper_speed_mps=0;candidate.gripper_force_n=5;
    }
    std::array<bool,kKeys.size()> seen{};
    while(!text.empty()) {
        const auto end=text.find('\n'); const auto line=Trim(text.substr(0,end));
        text=end==text.npos?std::string_view{}:text.substr(end+1);
        if(line.empty()) continue;
        const auto equals=line.find('=');
        if(equals==line.npos) return reject("Malformed settings line");
        const auto key=Trim(line.substr(0,equals));
        const auto found=std::find(kKeys.begin(),kKeys.end(),key);
        if(found==kKeys.end()) return reject("Unknown settings field");
        const auto i=static_cast<std::size_t>(found-kKeys.begin());
        if(i==23) {
            if(mode==SettingsParseMode::TraceV1 || mode==SettingsParseMode::TraceV2)return reject("Contact model requires QITR v3");
            const auto name=Trim(line.substr(equals+1));const auto profile=std::find(kContactProfiles.begin(),kContactProfiles.end(),name);
            if(seen[i] || profile==kContactProfiles.end())return reject("Repeated or invalid contact model");
            candidate.contact_profile=static_cast<ContactProfile>(profile-kContactProfiles.begin());seen[i]=true;continue;
        }
        if(mode==SettingsParseMode::TraceV1 && i>=20)return reject("Gripper postprocessing fields require QITR v2");
        double value=0;
        if(seen[i] || !Number(Trim(line.substr(equals+1)),value)) return reject("Repeated or invalid settings field");
        seen[i]=true;
        if(i==0) { if(value!=1) return reject("Unsupported settings version"); }
        else if(i==1) candidate.physics_dt=value;
        else if(i==21) candidate.gripper_speed_mps=value;
        else if(i==22) candidate.gripper_force_n=value;
        else if(i==2 || i==3) {
            if(value<1 || value>40 || std::floor(value)!=value) return reject("Interval must be an integer 1 to 40");
            (i==2?candidate.control_decimation:candidate.render_interval)=static_cast<std::uint32_t>(value);
        } else if(i>=4 && i<=6) candidate.hand_offset_degrees[i-4]=static_cast<float>(value);
        else if(i==7) candidate.shoulder_offset_m=static_cast<float>(value);
        else if(i==8) candidate.opacity=static_cast<float>(value);
        else if((i>=9 && i<=11) || (i>=18 && i<=20)) {
            if(value!=0 && value!=1) return reject("Boolean setting must be 0 or 1");
            if(i==9) candidate.follow_height=value==1;
            if(i==10) candidate.show_grid=value==1;
            if(i==11) candidate.floating_base=value==1;
            if(i==18) candidate.environment_collisions=value==1;
            if(i==19) candidate.show_room_surfaces=value==1;
            if(i==20) candidate.gripper_force_hold=value==1;
        } else if(i==12) candidate.gravity_scale=static_cast<float>(value);
        else {
            if(value<0 || value>=static_cast<double>(kInputCount) || std::floor(value)!=value) return reject("Invalid input binding");
            candidate.bindings[i-13]=static_cast<InputId>(static_cast<unsigned>(value));
        }
    }
    if(!seen[0]) return reject("Settings version is missing");
    if((mode==SettingsParseMode::TraceV2 || mode==SettingsParseMode::TraceV3) && (!seen[20] || !seen[21] || !seen[22]))
        return reject("QITR v2/v3 requires all gripper postprocessing settings");
    if(mode==SettingsParseMode::TraceV3 && !seen[23])return reject("QITR v3 requires contact model");
    if(!ValidateSettings(candidate,error)) return false;
    s=candidate;return true;
}
std::string SettingsJson(const SimSettings& s) {
    std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(17)<<std::boolalpha;
    out<<"{\"schema_version\":1,\"physics_dt\":"<<s.physics_dt
       <<",\"control_decimation\":"<<s.control_decimation<<",\"render_interval\":"<<s.render_interval
       <<",\"hand_offset_degrees\":["<<s.hand_offset_degrees[0]<<','<<s.hand_offset_degrees[1]<<','<<s.hand_offset_degrees[2]
       <<"],\"shoulder_offset_m\":"<<s.shoulder_offset_m<<",\"opacity\":"<<s.opacity
       <<",\"follow_height\":"<<s.follow_height<<",\"show_grid\":"<<s.show_grid
       <<",\"floating_base\":"<<s.floating_base<<",\"gravity_scale\":"<<s.gravity_scale
       <<",\"environment_collisions\":"<<s.environment_collisions<<",\"show_room_surfaces\":"<<s.show_room_surfaces
       <<",\"gripper_force_hold\":"<<s.gripper_force_hold<<",\"gripper_speed_mps\":"<<s.gripper_speed_mps
       <<",\"gripper_force_n\":"<<s.gripper_force_n<<",\"contact_profile\":\""<<ContactProfileName(s.contact_profile)<<"\"}";
    return out.str();
}
bool LoadSettings(const std::filesystem::path& path,SimSettings& s,std::string& error) {
    std::error_code ec; const auto size=std::filesystem::file_size(path,ec);
    if(ec || size>65536) {error=ec?"Settings unavailable; using defaults":"Settings file too large";return false;}
    std::ifstream file(path,std::ios::binary);
    std::string text(static_cast<std::size_t>(size),'\0');
    if(!file.read(text.data(),static_cast<std::streamsize>(text.size()))) {error="Cannot read settings";return false;}
    return ParseSettings(text,s,error);
}
bool SaveSettings(const std::filesystem::path& path,const SimSettings& s,std::string& error) {
    if(!ValidateSettings(s,error)) return false;
    auto temporary=path;temporary+=".pending";
    {
        std::ofstream file(temporary,std::ios::binary|std::ios::trunc);
        file<<SerializeSettings(s);file.flush();
        if(!file) {error="Cannot write settings";return false;}
        file.close();if(file.fail()) {error="Cannot close settings";return false;}
    }
#if defined(_WIN32)
    if(!MoveFileExW(temporary.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)) {
        error="Cannot publish settings";return false;
    }
#else
    std::error_code ec;std::filesystem::rename(temporary,path,ec);
    if(ec) {error="Cannot publish settings: "+ec.message();return false;}
#endif
    error.clear();return true;
}
} // namespace quest_newton
