#include "frc971/control_loops/drivetrain/localization/utils.h"

namespace frc971::control_loops::drivetrain {

LocalizationUtils::LocalizationUtils(aos::EventLoop *event_loop)
    : output_fetcher_(event_loop->MakeFetcher<Output>("/drivetrain")),
      clock_offset_fetcher_(
          event_loop->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos")) {}

Eigen::Vector2d LocalizationUtils::VoltageOrZero(
    aos::monotonic_clock::time_point now) {
  output_fetcher_.Fetch();
  // Determine if the robot is likely to be disabled currently.
  const bool disabled = (output_fetcher_.get() == nullptr) ||
                        (output_fetcher_.context().monotonic_event_time +
                             std::chrono::milliseconds(10) <
                         now);
  return disabled ? Eigen::Vector2d::Zero()
                  : Eigen::Vector2d{output_fetcher_->left_voltage(),
                                    output_fetcher_->right_voltage()};
}

bool LocalizationUtils::MaybeInAutonomous() {
  joystick_state_fetcher_.Fetch();
  return (joystick_state_fetcher_.get() != nullptr)
             ? joystick_state_fetcher_->autonomous()
             : true;
}

std::optional<aos::monotonic_clock::duration> LocalizationUtils::ClockOffset(
    std::string_view node) {
  std::optional<aos::monotonic_clock::duration> monotonic_offset;
  clock_offset_fetcher_.Fetch();
  if (clock_offset_fetcher_.get() != nullptr) {
    for (const auto connection : *clock_offset_fetcher_->connections()) {
      if (connection->has_node() && connection->node()->has_name() &&
          connection->node()->name()->string_view() == node) {
        if (connection->has_monotonic_offset()) {
          monotonic_offset =
              std::chrono::nanoseconds(connection->monotonic_offset());
        } else {
          // If we don't have a monotonic offset, that means we aren't
          // connected.
          return std::nullopt;
        }
        break;
      }
    }
  }
  CHECK(monotonic_offset.has_value());
  return monotonic_offset;
}

// Technically, this should be able to do a single memcpy, but the extra
// verbosity here seems appropriate.
Eigen::Matrix<double, 4, 4> FlatbufferToTransformationMatrix(
    const frc971::vision::calibration::TransformationMatrix &flatbuffer) {
  CHECK_EQ(16u, CHECK_NOTNULL(flatbuffer.data())->size());
  Eigen::Matrix<double, 4, 4> result;
  result.setIdentity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result(row, col) = (*flatbuffer.data())[row * 4 + col];
    }
  }
  return result;
}

}  // namespace frc971::control_loops::drivetrain
