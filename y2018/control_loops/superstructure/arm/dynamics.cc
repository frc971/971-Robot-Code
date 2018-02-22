#include "y2018/control_loops/superstructure/arm/dynamics.h"

DEFINE_bool(gravity, true, "If true, enable gravity.");

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {

const ::Eigen::Matrix<double, 2, 2> Dynamics::K3 =
    (::Eigen::Matrix<double, 2, 2>()
         << Dynamics::kG1 * Dynamics::Kt / Dynamics::kResistance,
     0.0, 0.0, Dynamics::kG2 *Dynamics::kNumDistalMotors *Dynamics::Kt /
                   Dynamics::kResistance)
        .finished();

const ::Eigen::Matrix<double, 2, 2> Dynamics::K3_inverse = K3.inverse();

const ::Eigen::Matrix<double, 2, 2> Dynamics::K4 =
    (::Eigen::Matrix<double, 2, 2>()
         << Dynamics::kG1 * Dynamics::kG1 * Dynamics::Kt /
                (Dynamics::Kv * Dynamics::kResistance),
     0.0, 0.0,
     Dynamics::kG2 *Dynamics::kG2 *Dynamics::Kt *Dynamics::kNumDistalMotors /
         (Dynamics::Kv * Dynamics::kResistance))
        .finished();

constexpr double Dynamics::kAlpha;
constexpr double Dynamics::kBeta;
constexpr double Dynamics::kGamma;

}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
