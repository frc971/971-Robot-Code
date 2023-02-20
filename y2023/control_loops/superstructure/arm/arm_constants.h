#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_CONSTANTS_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_CONSTANTS_H_

#include "frc971/control_loops/double_jointed_arm/dynamics.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {

constexpr double kEfficiencyTweak = 0.95;
constexpr double kStallTorque = 4.69 * kEfficiencyTweak;
constexpr double kFreeSpeed = (6380.0 / 60.0) * 2.0 * M_PI;
constexpr double kStallCurrent = 257.0;

constexpr ::frc971::control_loops::arm::ArmConstants kArmConstants = {
    .l1 = 20 * 0.0254,
    .l2 = 38 * 0.0254,

    .m1 = 9.34 / 2.2,
    .m2 = 9.77 / 2.2,

    // Moment of inertia of the joints in kg m^2
    .j1 = 2957.05 * 0.0002932545454545454,
    .j2 = 2824.70 * 0.0002932545454545454,

    // Radius of the center of mass of the joints in meters.
    .r1 = 21.64 * 0.0254,
    .r2 = 26.70 * 0.0254,

    // Gear ratios for the two joints.
    .g1 = 55.0,
    .g2 = 106.0,

    // Falcon motor constants.
    .efficiency_tweak = kEfficiencyTweak,
    .stall_torque = kStallTorque,
    .free_speed = kFreeSpeed,
    .stall_current = kStallCurrent,
    .resistance = 12.0 / kStallCurrent,
    .Kv = kFreeSpeed / 12.0,
    .Kt = kStallTorque / kStallCurrent,

    // Number of motors on the distal joint.
    .num_distal_motors = 1.0,
};

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_ARM_CONSTANTS_H_
