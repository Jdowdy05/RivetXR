#include "sim_settings.h"
#include "simulation_clock.h"
#include "base_registration.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
void Near(double a, double b, const char* reason) { Check(std::abs(a-b) < 1e-5, reason); }
std::string WithoutContactModel(std::string text){
    const auto at=text.find("contact_profile=");
    if(at!=std::string::npos)text.erase(at,text.find('\n',at)-at+1);return text;
}
}
int main() {
    using namespace quest_newton;
    try {
        SimSettings settings;
        std::string error;
        Check(ValidateSettings(settings, error), "defaults valid");
        Check(SerializeSettings(settings).find("contact_profile=legacy_mesh_v1\n")!=std::string::npos,
              "live defaults must explicitly serialize Original contact model");
        Check(settings.contact_profile==ContactProfile::OriginalMesh,"Original contact model is the live default");
        Check(SerializeSettings(settings).find("gripper_speed_mps=")!=std::string::npos,
              "serialized live settings must include the applied gripper speed");
        Check(!settings.gripper_force_hold&&settings.gripper_speed_mps==.05&&settings.gripper_force_n==5,
              "live gripper defaults are speed limited with optional force hold off");
        Near(settings.physics_dt, .005, "200Hz default");
        Check(settings.control_decimation == 2 && settings.render_interval == 2, "independent interval defaults");
        Check(!settings.environment_collisions && settings.show_room_surfaces, "room defaults: collisions off, surfaces shown");
        Check(settings.bindings[static_cast<std::size_t>(SimAction::Gripper)] == InputId::RightSqueeze, "analog grip default");
        auto changed = settings;
        changed.physics_dt = .01;
        changed.control_decimation = 3;
        changed.render_interval = 4;
        changed.hand_offset_degrees = {15,-25,45};
        changed.environment_collisions = true;
        changed.show_room_surfaces = false;
        changed.gripper_force_hold=true;
        changed.gripper_speed_mps=std::nextafter(.037,.2);
        changed.gripper_force_n=std::nextafter(7.25,8.);
        changed.contact_profile=ContactProfile::PadsWithFriction;
        changed.bindings[static_cast<std::size_t>(SimAction::Gripper)] = InputId::B;
        SimSettings restored;
        Check(ParseSettings(SerializeSettings(changed), restored, error) && restored == changed, "settings round trip");
        Check(ParseSettings(SerializeSettings(changed),restored,error,SettingsParseMode::TraceV3)&&restored==changed,
              "QITR v3 settings preserve exact double values and selected model");
        auto v2_expected=changed;v2_expected.contact_profile=ContactProfile::OriginalMesh;
        Check(ParseSettings(WithoutContactModel(SerializeSettings(changed)),restored,error,SettingsParseMode::TraceV2)&&restored==v2_expected,
              "QITR v2 preserves gripper doubles and implies Original model");
        Check(SettingsJson(changed).find("\"gripper_force_hold\":true")!=std::string::npos&&
              SettingsJson(changed).find("\"gripper_speed_mps\":")!=std::string::npos&&
              SettingsJson(changed).find("\"gripper_force_n\":")!=std::string::npos,"all gripper settings reach Python JSON");
        Check(SettingsJson(changed).find("\"environment_collisions\":true") != std::string::npos &&
              SettingsJson(changed).find("\"show_room_surfaces\":false") != std::string::npos,
              "room settings JSON uses boolean values");
        auto legacy = restored;
        Check(ParseSettings("version=1\nphysics_dt=.01\nopacity=.65\n", legacy, error), "legacy v1 settings accepted");
        Check(!legacy.environment_collisions && legacy.show_room_surfaces, "missing room keys restore defaults, not prior values");
        Near(legacy.physics_dt, .01, "legacy physics value preserved");
        Near(legacy.opacity, .65, "legacy opacity preserved");
        Check(legacy.gripper_speed_mps==.05&&!legacy.gripper_force_hold&&legacy.gripper_force_n==5,
              "old live settings files adopt the live gripper defaults");
        Check(legacy.contact_profile==ContactProfile::OriginalMesh,"missing saved model defaults to Original");
        Check(ParseSettings("version=1\nphysics_dt=.01\n",legacy,error,SettingsParseMode::TraceV1)&&
              legacy.gripper_speed_mps==0&&!legacy.gripper_force_hold&&legacy.gripper_force_n==5,
              "old traces use legacy direct targets instead of new live defaults");
        for(const auto key:{"gripper_force_hold","gripper_speed_mps","gripper_force_n"}) {
            const auto full=WithoutContactModel(SerializeSettings(changed));const auto at=full.find(std::string(key)+"=");
            auto missing=full;missing.erase(at,full.find('\n',at)-at+1);
            const auto snapshot=restored;
            Check(!ParseSettings(missing,restored,error,SettingsParseMode::TraceV2)&&restored==snapshot,
                  "v2 missing gripper field cannot partially apply");
            const auto new_key=std::string("version=1\n")+key+"=0\n";
            Check(!ParseSettings(new_key,restored,error,SettingsParseMode::TraceV1),"v1 rejects each new-only key");
        }
        for(const auto profile:{ContactProfile::OriginalMesh,ContactProfile::FivePads,ContactProfile::PadsWithFriction}){
            auto model=settings;model.contact_profile=profile;SimSettings roundtrip;
            Check(ParseSettings(SerializeSettings(model),roundtrip,error)&&roundtrip==model,"all contact model settings roundtrip");
            Check(SettingsJson(model).find(std::string("\"contact_profile\":\"")+std::string(ContactProfileName(profile))+"\"")!=std::string::npos,
                  "Python contact model is emitted as a stable string");
            for(auto mode:{SettingsParseMode::TraceV1,SettingsParseMode::TraceV2})
                Check(!ParseSettings(SerializeSettings(model),roundtrip,error,mode),"old trace modes reject explicit contact model");
            Check(!ParseSettings(WithoutContactModel(SerializeSettings(model)),roundtrip,error,SettingsParseMode::TraceV3),
                  "v3 requires explicit contact model");
        }
        Check(ParseSettings("version=1\nenvironment_collisions=1\n", legacy, error) &&
              legacy.environment_collisions && legacy.show_room_surfaces, "omitted room visibility defaults on");
        Check(ParseSettings("version=1\nshow_room_surfaces=0\n", legacy, error) &&
              !legacy.environment_collisions && !legacy.show_room_surfaces, "omitted environment collision defaults off");
        const auto before = restored;
        for (const auto text : {"version=99\n", "version=1\nphysics_dt=nan\n", "version=1\nphysics_dt=0\n",
                                "version=1\ncontrol_decimation=0\n", "version=1\nrender_interval=100\n",
                                "version=1\nphysics_dt=.005\nphysics_dt=.01\n", "version=1\nunknown=1\n",
                                "version=1\nenvironment_collisions=1\nenvironment_collisions=0\n",
                                "version=1\nshow_room_surfaces=1\nshow_room_surfaces=0\n",
                                "version=1\nenvironment_collisions=2\n", "version=1\nshow_room_surfaces=-1\n",
                                "version=1\nenvironment_collisions=.5\n", "version=1\nshow_room_surfaces=true\n",
                                "version=1\nenvironment_collisions=1\nroom_unknown=0\n",
                                "version=1\ngripper_speed_mps=-.1\n", "version=1\ngripper_speed_mps=.004\n",
                                "version=1\ngripper_speed_mps=.201\n", "version=1\ngripper_speed_mps=nan\n",
                                "version=1\ngripper_force_hold=1\ngripper_speed_mps=0\n",
                                "version=1\ngripper_force_hold=2\n", "version=1\ngripper_force_n=.4\n",
                                "version=1\ngripper_force_n=20.1\n", "version=1\ngripper_force_n=inf\n",
                                "version=1\ngripper_force_n=5\ngripper_force_n=6\n",
                                "version=1\ncontact_profile=unknown\n", "version=1\ncontact_profile=1\n",
                                "version=1\ncontact_profile=legacy_mesh_v1\ncontact_profile=five_pads_v1\n"}) {
            Check(!ParseSettings(text, restored, error) && restored == before, "invalid config must not partially apply");
        }
        changed = settings;
        changed.bindings[0] = changed.bindings[1];
        Check(!ValidateSettings(changed, error), "conflicting bindings rejected");
        changed = settings;
        changed.opacity = std::numeric_limits<float>::quiet_NaN();
        Check(!ValidateSettings(changed, error), "nonfinite settings rejected");
        auto invalid_profile=settings;invalid_profile.contact_profile=static_cast<ContactProfile>(255);
        Check(!ValidateSettings(invalid_profile,error),"out-of-range contact model enum rejected");
        for(double speed:{0.,.005,.2})for(double force:{.5,20.}) {
            auto valid=settings;valid.gripper_speed_mps=speed;valid.gripper_force_n=force;
            Check(ValidateSettings(valid,error),"gripper speed/force boundary rejected");
            valid.gripper_force_hold=true;
            Check(ValidateSettings(valid,error)==(speed!=0),"force hold cannot use legacy zero speed");
        }
        for(double invalid:{-1.,.0049,.2001,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
            auto value=settings;value.gripper_speed_mps=invalid;
            Check(!ValidateSettings(value,error),"invalid gripper speed accepted");
        }
        for(double invalid:{.49,20.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
            auto value=settings;value.gripper_force_n=invalid;
            Check(!ValidateSettings(value,error),"invalid gripper force accepted");
        }
        InputValues inputs{};
        inputs[static_cast<std::size_t>(InputId::RightSqueeze)] = {.63F, true};
        auto actions = ResolveActions(settings, inputs);
        Near(actions[static_cast<std::size_t>(SimAction::Gripper)].value, .63, "analog squeeze preserved");
        inputs[static_cast<std::size_t>(InputId::RightSqueeze)] = {2.F, true};
        Check(!ResolveActions(settings, inputs)[static_cast<std::size_t>(SimAction::Gripper)].active, "invalid analog input inactive");

        SimulationClock clock;
        Check(clock.Configure(settings, 0), "clock configure");
        clock.Reset(0);
        unsigned controls = 0, publications = 0, steps = 0;
        for (int frame=1; frame<=90; ++frame) {
            const auto due = clock.Accumulate(frame/90., false);
            for (unsigned j=0;j<due;++j) {
                const auto tick = clock.Advance();
                controls += tick.control_due;
                publications += tick.publish_due;
                ++steps;
            }
        }
        Check(steps == 200 && controls == 100 && publications == 100, "physics/control/render timing ratios");
        Near(clock.SimulationTime(), 1, "simulation clock");
        Check(clock.Accumulate(10, true) == 0 && clock.Accumulate(10.001, false) == 0, "pause clears catchup");
        Check(clock.Accumulate(11, false) <= 4 && clock.DroppedWallSeconds() > .9, "overrun catchup bounded");
        changed = settings; changed.physics_dt=.01; changed.control_decimation=3; changed.render_interval=4;
        Check(clock.Configure(changed,11), "timing reconfigure");
        clock.Reset(11);
        controls=publications=steps=0;
        for (int i=1;i<=12;++i) for(unsigned n=clock.Accumulate(11+i*.01,false);n>0;--n) {
            const auto tick=clock.Advance(); controls+=tick.control_due; publications+=tick.publish_due; ++steps;
        }
        Check(steps==12 && controls==4 && publications==3, "control and render decimation are separate");

        BaseRegistration base;
        kinematics::Pose head; head.position={0,1.7F,0};
        Check(base.Update(head,true,false,settings), "initial registration");
        Near(base.WorldFromBase().position[2],1.45,"shoulder height");
        const auto up = kinematics::Compose(kinematics::Pose{{0,0,0},base.StageFromWorld().rotation},kinematics::Pose{{0,0,1}}).position;
        Near(up[0],0,"gravity aligned x"); Near(up[1],1,"simZ is stageY"); Near(up[2],0,"gravity aligned z");
        Near(base.StageFromWorld().position[1],0,"world zero at floor");
        head.position[1]=2; base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.75,"height follows");
        head.position[1]=1.1F; base.Update(head,true,true,settings);
        Near(base.WorldFromBase().position[2],1.75,"clutch freezes");
        base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.75,"release does not jump");
        head.position[1]=1.2F; base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.85,"new nominal offset follows");
        head.position[1]=.6F; base.Update(head,false,false,settings); base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.85,"tracking reacquisition does not jump");
        head.position[1]=.7F; base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.95,"tracking resumes from new height");
        settings.floating_base=true; head.position[1]=1.9F; base.Update(head,true,false,settings);
        Near(base.WorldFromBase().position[2],1.95,"dynamic base is not moved by head");
        const auto offset=HandOffsetPose({0,0,90});
        const auto rotated=kinematics::Compose(offset,kinematics::Pose{{1,0,0}}).position;
        Near(rotated[0],0,"hand offset x"); Near(rotated[1],1,"hand offset rotation");
        std::cout << "settings, independent clocks, analog bindings and gravity/height clutch passed\n";
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
