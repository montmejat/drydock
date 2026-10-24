// Drives the simulated OpenArm with OpenArm's own CAN library, as
// openarm_ros2 drives the real one: its motor types, CAN ids and gains.
// Enables the motors, reads back their master ids from their registers,
// then holds the arm at its home pose with the first joint swinging, opens
// and closes the gripper, and prints where each joint is twice a second.
//
//   examples/openarm/hold [seconds] [--classic]     (vcan0, CAN FD unless --classic)

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <vector>

using namespace openarm::damiao_motor;

int main(int argc, char** argv) {
    double seconds = 10.0;
    bool can_fd = true;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--classic") {
            can_fd = false;
        } else {
            seconds = std::atof(argv[i]);
        }
    }

    // openarm_ros2's defaults for OpenArm v1, on vcan0.
    openarm::can::socket::OpenArm arm("vcan0", can_fd);
    arm.init_arm_motors({MotorType::DM8009, MotorType::DM8009, MotorType::DM4340, MotorType::DM4340,
                         MotorType::DM4310, MotorType::DM4310, MotorType::DM4310},
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
    arm.init_gripper_motor(MotorType::DM4310, 0x08, 0x18);
    const std::vector<double> kp = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
    const std::vector<double> kd = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};
    const std::vector<double> home = {0.0, 0.0, 0.0, 1.570796, 0.0, 0.0, 0.0};

    // recv_all waits for the first answer only, then takes what is already
    // there: answers still on their way would be read later, in the wrong
    // mode (a register answer read as state). So the setup steps let all
    // eight answers arrive first.
    const auto settle = std::chrono::milliseconds(5);
    arm.set_callback_mode_all(CallbackMode::IGNORE);
    arm.enable_all();
    std::this_thread::sleep_for(settle);
    arm.recv_all(2000);

    arm.set_callback_mode_all(CallbackMode::PARAM);
    arm.query_param_all(static_cast<int>(RID::MST_ID));
    std::this_thread::sleep_for(settle);
    arm.recv_all(2000);
    std::printf("master ids:");
    for (const auto& m : arm.get_arm().get_motors()) {
        std::printf(" 0x%02X", static_cast<int>(m.get_param(static_cast<int>(RID::MST_ID))));
    }
    for (const auto& m : arm.get_gripper().get_motors()) {
        std::printf(" 0x%02X", static_cast<int>(m.get_param(static_cast<int>(RID::MST_ID))));
    }
    std::printf("\n");

    arm.set_callback_mode_all(CallbackMode::STATE);
    const auto period = std::chrono::microseconds(2000);
    auto due = std::chrono::steady_clock::now();
    for (int cycle = 0; cycle * 0.002 < seconds; cycle++) {
        const double t = cycle * 0.002;
        std::vector<MITParam> params;
        for (size_t i = 0; i < home.size(); i++) {
            double q = home[i] + (i == 0 ? 0.4 * std::sin(2 * M_PI * 0.25 * t) : 0.0);
            params.push_back({kp[i], kd[i], q, 0.0, 0.0});
        }
        arm.get_arm().mit_control_all(params);
        // Open for two seconds, closed for two: 0 rad is open, -1.0472 closed.
        const double gripper = std::fmod(t, 4.0) < 2.0 ? 0.0 : -1.0472;
        arm.get_gripper().mit_control_all({{5.0, 0.1, gripper, 0.0, 0.0}});
        arm.recv_all(500);

        if (cycle % 250 == 0) {
            std::printf("t = %4.1f s", t);
            for (const auto& m : arm.get_arm().get_motors()) std::printf(" %+.2f", m.get_position());
            for (const auto& m : arm.get_gripper().get_motors()) {
                std::printf(" | gripper %+.2f rad %+.2f N m", m.get_position(), m.get_torque());
            }
            std::printf("\n");
        }
        due += period;
        std::this_thread::sleep_until(due);
    }
    arm.disable_all();
    arm.recv_all(1000);
    return 0;
}
