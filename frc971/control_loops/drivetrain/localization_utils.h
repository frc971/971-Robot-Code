#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
#include <Eigen/Dense>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
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
class LocalizationUtils {
 public:
  LocalizationUtils(aos::EventLoop *event_loop);

  // Returns the latest drivetrain output voltage, or zero if no output is
  // available (which happens when the robot is disabled; when the robot is
  // disabled, the voltage is functionally zero). Return value will be
  // [left_voltage, right_voltage]
  Eigen::Vector2d VoltageOrZero(aos::monotonic_clock::time_point now);

  // Returns true if either there is no JoystickState message available or if
  // we are currently in autonomous mode.
  bool MaybeInAutonomous();

  // Returns the offset between our node and the specified node (or nullopt if
  // no offset is available). The sign of this will be such that the time on
  // the remote node = time on our node + ClockOffset().
  std::optional<aos::monotonic_clock::duration> ClockOffset(
      std::string_view node);

 private:
  aos::Fetcher<frc971::control_loops::drivetrain::Output> output_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
};

// Converts a flatbuffer TransformationMatrix to an Eigen matrix.
Eigen::Matrix<double, 4, 4> FlatbufferToTransformationMatrix(
    const frc971::vision::calibration::TransformationMatrix &flatbuffer);

}  // namespace frc971::control_loops::drivetrain

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATION_UTILS_H_
