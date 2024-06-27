#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include <functional>

#include "aos/flatbuffer_merge.h"
#if defined(__linux__)
#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#include "frc971/control_loops/hybrid_state_feedback_loop_converters.h"
#endif
#include "frc971/control_loops/drivetrain/drivetrain_config_static.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop_converters.h"
#include "frc971/shifter_hall_effect.h"

namespace frc971::control_loops::drivetrain {

// Configuration for line-following mode.
struct LineFollowConfig {
  // The line-following uses an LQR controller with states of [theta,
  // linear_velocity, angular_velocity] and inputs of [left_voltage,
  // right_voltage].
  // These Q and R matrices are the costs for state and input respectively.
  Eigen::Matrix3d Q =
      Eigen::Matrix3d((::Eigen::DiagonalMatrix<double, 3>().diagonal()
                           << 1.0 / ::std::pow(0.1, 2),
                       1.0 / ::std::pow(1.0, 2), 1.0 / ::std::pow(1.0, 2))
                          .finished()
                          .asDiagonal());
  Eigen::Matrix2d R =
      Eigen::Matrix2d((::Eigen::DiagonalMatrix<double, 2>().diagonal()
                           << 1.0 / ::std::pow(12.0, 2),
                       1.0 / ::std::pow(12.0, 2))
                          .finished()
                          .asDiagonal());

  // The driver can use their steering controller to adjust where we attempt to
  // place things laterally. This specifies how much range on either side of
  // zero we allow them, in meters.
  double max_controllable_offset = 0.1;

  static LineFollowConfig FromFlatbuffer(const fbs::LineFollowConfig *fbs) {
    if (fbs == nullptr) {
      return {};
    }
    CHECK(fbs->q() != nullptr);
    CHECK(fbs->r() != nullptr);
    return LineFollowConfig{
        .Q = ToEigenOrDie<3, 3>(*fbs->q()),
        .R = ToEigenOrDie<2, 2>(*fbs->r()),
        .max_controllable_offset = fbs->max_controllable_offset()};
  }
};

struct SplineFollowerConfig {
  // The line-following uses an LQR controller with states of
  // [longitudinal_position, lateral_positoin, theta, left_velocity,
  // right_velocity] and inputs of [left_voltage, right_voltage]. These Q and R
  // matrices are the costs for state and input respectively.
  Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>(
      (::Eigen::DiagonalMatrix<double, 5>().diagonal() << ::std::pow(60.0, 2.0),
       ::std::pow(60.0, 2.0), ::std::pow(40.0, 2.0), ::std::pow(30.0, 2.0),
       ::std::pow(30.0, 2.0))
          .finished()
          .asDiagonal());
  Eigen::Matrix2d R = Eigen::Matrix2d(
      (::Eigen::DiagonalMatrix<double, 2>().diagonal() << 5.0, 5.0)
          .finished()
          .asDiagonal());

  static SplineFollowerConfig FromFlatbuffer(
      const fbs::SplineFollowerConfig *fbs) {
    if (fbs == nullptr) {
      return {};
    }
    CHECK(fbs->q() != nullptr);
    CHECK(fbs->r() != nullptr);
    return SplineFollowerConfig{.Q = ToEigenOrDie<5, 5>(*fbs->q()),
                                .R = ToEigenOrDie<2, 2>(*fbs->r())};
  }
};

template <typename Scalar = double>
struct DrivetrainConfig {
  // Shifting method we are using.
  ShifterType shifter_type;

  // Type of loop to use.
  LoopType loop_type;

  // Type of gyro to use.
  GyroType gyro_type;

  // Type of IMU to use.
  ImuType imu_type;

  // Polydrivetrain functions returning various controller loops with plants.
  ::std::function<StateFeedbackLoop<4, 2, 2, Scalar>()> make_drivetrain_loop;
  ::std::function<StateFeedbackLoop<2, 2, 2, Scalar>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<7, 2, 4, Scalar>()> make_kf_drivetrain_loop;
#if defined(__linux__)
  ::std::function<
      StateFeedbackLoop<2, 2, 2, Scalar, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>()>
      make_hybrid_drivetrain_velocity_loop;
#endif

  ::std::chrono::nanoseconds dt;  // Control loop time step.
  Scalar robot_radius;            // Robot radius, in meters.
  Scalar wheel_radius;            // Wheel radius, in meters.
  Scalar v;                       // Motor velocity constant.

  // Gear ratios, from wheel to motor shaft.
  Scalar high_gear_ratio;
  Scalar low_gear_ratio;

  // Moment of inertia and mass.
  Scalar J;
  Scalar mass;

  // Hall effect constants. Unused if not applicable to shifter type.
  ShifterHallEffect left_drive;
  ShifterHallEffect right_drive;

  // Variable that holds the default gear ratio. We use this in ZeroOutputs().
  // (ie. true means high gear is default).
  bool default_high_gear;

  Scalar down_offset;

  Scalar wheel_non_linearity;

  Scalar quickturn_wheel_multiplier;

  Scalar wheel_multiplier;

  // Whether the shift button on the pistol grip enables line following mode.
  bool pistol_grip_shift_enables_line_follow = false;

  // Rotation matrix from the IMU's coordinate frame to the robot's coordinate
  // frame.
  // I.e., imu_transform * imu_readings will give the imu readings in the
  // robot frame.
  Eigen::Matrix<Scalar, 3, 3> imu_transform =
      Eigen::Matrix<Scalar, 3, 3>::Identity();

  // True if we are running a simulated drivetrain.
  bool is_simulated = false;

  DownEstimatorConfigT down_estimator_config{};

  LineFollowConfig line_follow_config{};

  PistolTopButtonUse top_button_use = PistolTopButtonUse::kShift;
  PistolSecondButtonUse second_button_use = PistolSecondButtonUse::kShiftLow;
  PistolBottomButtonUse bottom_button_use = PistolBottomButtonUse::kSlowDown;

  SplineFollowerConfig spline_follower_config{};

  // If set, then the IMU must be zeroed before we will send any outputs.
  bool require_imu_for_output = true;

  // Converts the robot state to a linear distance position, velocity.
  static Eigen::Matrix<Scalar, 2, 1> LeftRightToLinear(
      const Eigen::Matrix<Scalar, 7, 1> &left_right) {
    Eigen::Matrix<Scalar, 2, 1> linear;
    linear(0, 0) = (left_right(0, 0) + left_right(2, 0)) / 2.0;
    linear(1, 0) = (left_right(1, 0) + left_right(3, 0)) / 2.0;
    return linear;
  }
  // Converts the robot state to an anglular distance, velocity.
  Eigen::Matrix<Scalar, 2, 1> LeftRightToAngular(
      const Eigen::Matrix<Scalar, 7, 1> &left_right) const {
    Eigen::Matrix<Scalar, 2, 1> angular;
    angular(0, 0) =
        (left_right(2, 0) - left_right(0, 0)) / (this->robot_radius * 2.0);
    angular(1, 0) =
        (left_right(3, 0) - left_right(1, 0)) / (this->robot_radius * 2.0);
    return angular;
  }

  Eigen::Matrix<Scalar, 2, 2> Tlr_to_la() const {
    return (::Eigen::Matrix<Scalar, 2, 2>() << 0.5, 0.5,
            -1.0 / (2 * robot_radius), 1.0 / (2 * robot_radius))
        .finished();
  }

  Eigen::Matrix<Scalar, 2, 2> Tla_to_lr() const {
    return Tlr_to_la().inverse();
  }

  // Converts the linear and angular position, velocity to the top 4 states of
  // the robot state.
  Eigen::Matrix<Scalar, 4, 1> AngularLinearToLeftRight(
      const Eigen::Matrix<Scalar, 2, 1> &linear,
      const Eigen::Matrix<Scalar, 2, 1> &angular) const {
    Eigen::Matrix<Scalar, 2, 1> scaled_angle = angular * this->robot_radius;
    Eigen::Matrix<Scalar, 4, 1> state;
    state(0, 0) = linear(0, 0) - scaled_angle(0, 0);
    state(1, 0) = linear(1, 0) - scaled_angle(1, 0);
    state(2, 0) = linear(0, 0) + scaled_angle(0, 0);
    state(3, 0) = linear(1, 0) + scaled_angle(1, 0);
    return state;
  }

  static DrivetrainConfig FromFlatbuffer(const fbs::DrivetrainConfig &fbs) {
    std::shared_ptr<aos::FlatbufferDetachedBuffer<fbs::DrivetrainConfig>>
        fbs_copy = std::make_shared<
            aos::FlatbufferDetachedBuffer<fbs::DrivetrainConfig>>(
            aos::RecursiveCopyFlatBuffer(&fbs));
    CHECK(fbs_copy->message().loop_config()->drivetrain_loop() != nullptr);
    CHECK(fbs_copy->message().loop_config()->velocity_drivetrain_loop() !=
          nullptr);
    CHECK(fbs_copy->message().loop_config()->kalman_drivetrain_loop() !=
          nullptr);
    CHECK(
        fbs_copy->message().loop_config()->hybrid_velocity_drivetrain_loop() !=
        nullptr);
    CHECK(fbs.imu_transform() != nullptr);
    return {
#define ASSIGN(field) .field = fbs.field()
      ASSIGN(shifter_type), ASSIGN(loop_type), ASSIGN(gyro_type),
          ASSIGN(imu_type),
          .make_drivetrain_loop =
              [fbs_copy]() {
                return MakeStateFeedbackLoop<4, 2, 2>(
                    *fbs_copy->message().loop_config()->drivetrain_loop());
              },
          .make_v_drivetrain_loop =
              [fbs_copy]() {
                return MakeStateFeedbackLoop<2, 2, 2>(
                    *fbs_copy->message()
                         .loop_config()
                         ->velocity_drivetrain_loop());
              },
          .make_kf_drivetrain_loop =
              [fbs_copy]() {
                return MakeStateFeedbackLoop<7, 2, 4>(
                    *fbs_copy->message()
                         .loop_config()
                         ->kalman_drivetrain_loop());
              },
#if defined(__linux__)
          .make_hybrid_drivetrain_velocity_loop =
              [fbs_copy]() {
                return MakeHybridStateFeedbackLoop<2, 2, 2>(
                    *fbs_copy->message()
                         .loop_config()
                         ->hybrid_velocity_drivetrain_loop());
              },
#endif
          .dt = std::chrono::nanoseconds(fbs.loop_config()->dt()),
          .robot_radius = fbs.loop_config()->robot_radius(),
          .wheel_radius = fbs.loop_config()->wheel_radius(),
          .v = fbs.loop_config()->motor_kv(),
          .high_gear_ratio = fbs.loop_config()->high_gear_ratio(),
          .low_gear_ratio = fbs.loop_config()->low_gear_ratio(),
          .J = fbs.loop_config()->moment_of_inertia(),
          .mass = fbs.loop_config()->mass(),
          .left_drive =
              fbs.has_left_drive() ? *fbs.left_drive() : ShifterHallEffect{},
          .right_drive =
              fbs.has_right_drive() ? *fbs.right_drive() : ShifterHallEffect{},
          ASSIGN(default_high_gear), ASSIGN(down_offset),
          ASSIGN(wheel_non_linearity), ASSIGN(quickturn_wheel_multiplier),
          ASSIGN(wheel_multiplier),
          ASSIGN(pistol_grip_shift_enables_line_follow),
          .imu_transform = ToEigenOrDie<3, 3>(*fbs.imu_transform()),
          ASSIGN(is_simulated),
          .down_estimator_config =
              aos::UnpackFlatbuffer(fbs.down_estimator_config()),
          .line_follow_config =
              LineFollowConfig::FromFlatbuffer(fbs.line_follow_config()),
          ASSIGN(top_button_use), ASSIGN(second_button_use),
          ASSIGN(bottom_button_use),
          .spline_follower_config = SplineFollowerConfig::FromFlatbuffer(
              fbs.spline_follower_config()),
          ASSIGN(require_imu_for_output),
#undef ASSIGN
    };
  }
};
}  // namespace frc971::control_loops::drivetrain

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
