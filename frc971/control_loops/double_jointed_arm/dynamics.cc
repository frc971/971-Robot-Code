#include "frc971/control_loops/double_jointed_arm/dynamics.h"

DEFINE_bool(gravity, true, "If true, enable gravity.");

namespace frc971 {
namespace control_loops {
namespace arm {

Dynamics::Dynamics(ArmConstants arm_constants)
    : arm_constants_(arm_constants),

      K3_((::Eigen::Matrix<double, 2, 2>() << arm_constants_.g0 *
                                                  arm_constants_.Kt /
                                                  arm_constants_.resistance,
           0.0, 0.0,
           arm_constants_.g1 * arm_constants_.num_distal_motors *
               arm_constants_.Kt / arm_constants_.resistance)
              .finished()),

      K3_inverse_(K3_.inverse()),
      K4_((::Eigen::Matrix<double, 2, 2>()
               << arm_constants_.g0 * arm_constants_.g0 * arm_constants_.Kt /
                      (arm_constants_.Kv * arm_constants_.resistance),
           0.0, 0.0,
           arm_constants_.g1 * arm_constants_.g1 * arm_constants_.Kt *
               arm_constants_.num_distal_motors /
               (arm_constants_.Kv * arm_constants_.resistance))
              .finished()),
      alpha_(arm_constants_.j0 +
             arm_constants_.r0 * arm_constants_.r0 * arm_constants_.m0 +
             arm_constants_.l0 * arm_constants_.l0 * arm_constants_.m1),
      beta_(arm_constants_.l0 * arm_constants_.r1 * arm_constants_.m1),
      gamma_(arm_constants_.j1 +
             arm_constants_.r1 * arm_constants_.r1 * arm_constants_.m1) {}
}  // namespace arm
}  // namespace control_loops
}  // namespace frc971
