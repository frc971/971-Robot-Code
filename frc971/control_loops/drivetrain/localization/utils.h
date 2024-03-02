#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
#include <Eigen/Dense>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"
#include "frc971/control_loops/drivetrain/rio_localizer_inputs_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/vision/calibration_generated.h"

namespace frc971::control_loops::drivetrain {
// This class provides a variety of checks that have generally proved useful for
// the localizer but which have no clear place to live otherwise.
// Specifically, it tracks:
// * Drivetrain voltages, including checks for whether the Output message
//   has timed out.
// * Offsets between monotonic clocks on different devices.
// * Whether we are in autonomous mode.
// * Drivetrain encoder voltages, as reported by the roborio.
class LocalizationUtils {
 public:
  LocalizationUtils(aos::EventLoop *event_loop);

  // Returns the latest drivetrain output voltage, or zero if no output is
  // available (which happens when the robot is disabled; when the robot is
  // disabled, the voltage is functionally zero). Return value will be
  // [left_voltage, right_voltage]
  Eigen::Vector2d VoltageOrZero(aos::monotonic_clock::time_point now);

  // Returns the latest drivetrain encoder values (in meters), or nullopt if no
  // position message is available (or if the message is stale).
  // Returns encoders as [left_position, right_position]
  std::optional<Eigen::Vector2d> Encoders(aos::monotonic_clock::time_point now);

  // Returns true if either there is no JoystickState message available or if
  // we are currently in autonomous mode.
  bool MaybeInAutonomous();
  aos::Alliance Alliance();

  // Returns the offset between our node and the specified node (or nullopt if
  // no offset is available). The sign of this will be such that the time on
  // the remote node = time on our node + ClockOffset().
  std::optional<aos::monotonic_clock::duration> ClockOffset(
      std::string_view node);

 private:
  aos::EventLoop *const event_loop_;
  aos::Fetcher<frc971::control_loops::drivetrain::Output> output_fetcher_;
  aos::Fetcher<frc971::control_loops::drivetrain::Position> position_fetcher_;
  aos::Fetcher<frc971::control_loops::drivetrain::RioLocalizerInputs>
      combined_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
};

// Converts a flatbuffer TransformationMatrix to an Eigen matrix.
Eigen::Matrix<double, 4, 4> FlatbufferToTransformationMatrix(
    const frc971::vision::calibration::TransformationMatrix &flatbuffer);

// This approximates the Jacobian of a vector of [heading, distance, skew]
// of a target with respect to the full state of a drivetrain EKF.
// Note that the only nonzero values in the returned matrix will be in the
// columns corresponding to the X, Y, and Theta components of the state.
// This is suitable for use as the H matrix in the kalman updates of the EKF,
// although due to the approximation it should not be used to actually
// calculate the expected measurement.
// target_pose is the global pose of the target that we have identified.
// camera_pose is the current estimate of the global pose of
//   the camera that can see the target.
template <typename Scalar>
Eigen::Matrix<double, 3, HybridEkf<Scalar>::kNStates>
HMatrixForCameraHeadingDistanceSkew(const TypedPose<Scalar> &target_pose,
                                    const TypedPose<Scalar> &camera_pose) {
  // For all of the below calculations, we will assume to a first
  // approximation that:
  //
  // dcamera_theta / dtheta ~= 1
  // dcamera_x / dx ~= 1
  // dcamera_y / dy ~= 1
  //
  // For cameras sufficiently far from the robot's origin, or if the robot were
  // spinning extremely rapidly, this would not hold.

  // To calculate dheading/d{x,y,theta}:
  // heading = arctan2(target_pos - camera_pos) - camera_theta
  Eigen::Matrix<Scalar, 3, 1> target_pos = target_pose.abs_pos();
  Eigen::Matrix<Scalar, 3, 1> camera_pos = camera_pose.abs_pos();
  Scalar diffx = target_pos.x() - camera_pos.x();
  Scalar diffy = target_pos.y() - camera_pos.y();
  Scalar norm2 = diffx * diffx + diffy * diffy;
  Scalar dheadingdx = diffy / norm2;
  Scalar dheadingdy = -diffx / norm2;
  Scalar dheadingdtheta = -1.0;

  // To calculate ddistance/d{x,y}:
  // distance = sqrt(diffx^2 + diffy^2)
  Scalar distance = ::std::sqrt(norm2);
  Scalar ddistdx = -diffx / distance;
  Scalar ddistdy = -diffy / distance;

  // Skew = target.theta - camera.theta - heading
  //      = target.theta - arctan2(target_pos - camera_pos)
  Scalar dskewdx = -dheadingdx;
  Scalar dskewdy = -dheadingdy;
  Eigen::Matrix<Scalar, 3, HybridEkf<Scalar>::kNStates> H;
  H.setZero();
  H(0, HybridEkf<Scalar>::kX) = dheadingdx;
  H(0, HybridEkf<Scalar>::kY) = dheadingdy;
  H(0, HybridEkf<Scalar>::kTheta) = dheadingdtheta;
  H(1, HybridEkf<Scalar>::kX) = ddistdx;
  H(1, HybridEkf<Scalar>::kY) = ddistdy;
  H(2, HybridEkf<Scalar>::kX) = dskewdx;
  H(2, HybridEkf<Scalar>::kY) = dskewdy;
  return H;
}

}  // namespace frc971::control_loops::drivetrain

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
