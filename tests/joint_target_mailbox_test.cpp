#include "joint_target_mailbox.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
void Check(bool value) { if (!value) throw std::runtime_error("joint target mailbox check failed"); }
}

int main() {
    using namespace quest_newton;
    using namespace std::chrono_literals;
    try {
        JointTargetMailbox mailbox;
        const auto now = JointTargetMailbox::Clock::time_point{1s};
        Check(!mailbox.Read(now).engaged);
        Check(mailbox.Publish(kinematics::kHome, true, now));
        Check(mailbox.Read(now + 99ms).engaged);
        Check(!mailbox.Read(now + 101ms).engaged);
        Check(!mailbox.Read(now - 1ms).engaged);
        auto invalid = kinematics::kHome;
        invalid[6] = std::numeric_limits<float>::quiet_NaN();
        Check(!mailbox.Publish(invalid, true, now));
        Check(mailbox.Read(now).generation == 1);
        Check(mailbox.Read(now).joints == kinematics::kHome);
        invalid = kinematics::kHome;
        invalid[4] = kinematics::kUpperLimits[4] + .01F;
        Check(!mailbox.Publish(invalid, true, now));
        Check(mailbox.Publish(kinematics::kHome, false, now));
        Check(!mailbox.Read(now).engaged);

        std::atomic<bool> done{false};
        std::atomic<bool> published{true};
        std::thread writer([&] {
            for (int i = 0; i < 10000; ++i) {
                auto joints = kinematics::kHome;
                for (auto& joint : joints) joint += (i % 2 == 0 ? .01F : -.01F);
                if (!mailbox.Publish(joints, true, now)) published.store(false);
            }
            done.store(true);
        });
        bool coherent = true;
        do {
            const auto sample = mailbox.Read(now);
            const float delta = sample.joints[0] - kinematics::kHome[0];
            for (std::size_t i = 1; i < sample.joints.size(); ++i) {
                coherent &= std::abs(sample.joints[i] - kinematics::kHome[i] - delta) < 1e-6F;
            }
        } while (!done.load());
        writer.join();
        Check(coherent && published.load());
        Check(mailbox.Read(now).generation == 10002);
        std::puts("joint target freshness, limits and complete publication passed");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
