#pragma once
#include "sim_settings.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace quest_newton {
struct SimulationTick {
    std::uint64_t step_index;
    double physics_dt;
    double simulation_time;
    bool control_due;
    bool publish_due;
};
// The worker owns this clock. XR display frames are scheduled by OpenXR itself.
class SimulationClock {
public:
    bool Configure(const SimSettings& settings,double now) {
        std::string error;
        if(!std::isfinite(now) || !ValidateSettings(settings,error)) return false;
        settings_=settings;last_wall_=now;remainder_=0;phase_=0;return true;
    }
    void Reset(double now) { last_wall_=now;remainder_=0;sim_time_=0;steps_=0;phase_=0;dropped_=0; }
    unsigned Accumulate(double now,bool paused) {
        if(!std::isfinite(now) || now<last_wall_) {remainder_=0;return 0;}
        const double elapsed=now-last_wall_;last_wall_=now;
        if(paused) {remainder_=0;return 0;}
        remainder_+=elapsed;
        const double due=std::floor((remainder_+1e-12)/settings_.physics_dt);
        if(due<1) return 0;
        const auto count=static_cast<unsigned>(std::min(due,4.0));
        if(due>4) dropped_+=(due-4)*settings_.physics_dt;
        remainder_=std::max(0.0,remainder_-due*settings_.physics_dt);
        return count;
    }
    SimulationTick Advance() {
        const bool control=phase_%settings_.control_decimation==0;
        ++steps_;++phase_;sim_time_+=settings_.physics_dt;
        return {steps_,settings_.physics_dt,sim_time_,control,phase_%settings_.render_interval==0};
    }
    double SimulationTime() const {return sim_time_;}
    double DroppedWallSeconds() const {return dropped_;}
private:
    SimSettings settings_;
    double last_wall_=0,remainder_=0,sim_time_=0,dropped_=0;
    std::uint64_t steps_=0,phase_=0;
};
} // namespace quest_newton
