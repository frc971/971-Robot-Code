#include "frc971/control_loops/drivetrain/localization/utils.h"

namespace frc971::control_loops::drivetrain {

LocalizationUtils::LocalizationUtils(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      output_fetcher_(event_loop->TryMakeFetcher<Output>("/drivetrain")),
      position_fetcher_(event_loop->TryMakeFetcher<Position>("/drivetrain")),
      combined_fetcher_(
          event_loop->TryMakeFetcher<RioLocalizerInputs>("/drivetrain")),
      clock_offset_fetcher_(
          event_loop->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos")) {}

namespace {
template <typename T>
Eigen::Vector2d GetVoltage(T &fetcher, aos::monotonic_clock::time_point now) {
  fetcher.Fetch();
  // Determine if the robot is likely to be disabled currently.
  const bool disabled =
      (fetcher.get() == nullptr) ||
      (fetcher.context().monotonic_event_time + std::chrono::milliseconds(500) <
       now);
  return disabled ? Eigen::Vector2d::Zero()
                  : Eigen::Vector2d{fetcher->left_voltage(),
                                    fetcher->right_voltage()};
}
}  // namespace
Eigen::Vector2d LocalizationUtils::VoltageOrZero(
    aos::monotonic_clock::time_point now) {
  if (output_fetcher_.valid()) {
    return GetVoltage(output_fetcher_, now);
  } else {
    CHECK(combined_fetcher_.valid());
    return GetVoltage(combined_fetcher_, now);
  }
}
namespace {
template <typename T>
std::optional<Eigen::Vector2d> GetPosition(
    T &fetcher, aos::monotonic_clock::time_point now) {
  if (!fetcher.Fetch()) {
    return std::nullopt;
  }
  const bool stale =
      (fetcher.get() == nullptr) ||
      (fetcher.context().monotonic_event_time + std::chrono::milliseconds(10) <
       now);
  return stale ? std::nullopt
               : std::make_optional<Eigen::Vector2d>(fetcher->left_encoder(),
                                                     fetcher->right_encoder());
}
}  // namespace

std::optional<Eigen::Vector2d> LocalizationUtils::Encoders(
    aos::monotonic_clock::time_point now) {
  if (position_fetcher_.valid()) {
    return GetPosition(position_fetcher_, now);
  } else {
    CHECK(combined_fetcher_.valid());
    return GetPosition(combined_fetcher_, now);
  }
}

bool LocalizationUtils::MaybeInAutonomous() {
  joystick_state_fetcher_.Fetch();
  return (joystick_state_fetcher_.get() != nullptr)
             ? joystick_state_fetcher_->autonomous()
             : true;
}

aos::Alliance LocalizationUtils::Alliance() {
  joystick_state_fetcher_.Fetch();
  return (joystick_state_fetcher_.get() != nullptr)
             ? joystick_state_fetcher_->alliance()
             : aos::Alliance::kInvalid;
}

std::optional<aos::monotonic_clock::duration> LocalizationUtils::ClockOffset(
    std::string_view node) {
  if (node == event_loop_->node()->name()->string_view()) {
    return std::chrono::seconds(0);
  }
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
  CHECK(flatbuffer.data() != nullptr);
  CHECK_EQ(16u, flatbuffer.data()->size());
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
