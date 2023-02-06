#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"
#include "aos/util/log_interval.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_states.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/gear.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/splinedrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/polytope.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "frc971/zeroing/imu_zeroer.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace chrono = std::chrono;

// A class to hold all the estimators in use in the drivetrain.  This lets us
// run them on a log file without running the controllers, making them easier to
// tune.
class DrivetrainFilters {
 public:
  DrivetrainFilters(const DrivetrainConfig<double> &dt_config,
                    ::aos::EventLoop *event_loop,
                    LocalizerInterface *localizer);

  double localizer_theta() const { return localizer_->theta(); }
  double x() const { return localizer_->x(); }
  double y() const { return localizer_->y(); }

  // Returns the current gear for both sides.
  Gear left_gear() const { return left_gear_; }
  Gear right_gear() const { return right_gear_; }

  // Tracks the shift requests for each side.
  void set_left_high_requested(bool value) { left_high_requested_ = value; }
  void set_right_high_requested(bool value) { right_high_requested_ = value; }

  // Returns a pointer to the drivetrain kalman filter.
  StateFeedbackLoop<7, 2, 4> *kf() { return &kf_; }

  // Populates the various state flatbuffers used for diagnostics.
  flatbuffers::Offset<LocalizerState> PopulateLocalizerState(
      flatbuffers::FlatBufferBuilder *fbb);
  flatbuffers::Offset<ImuZeroerState> PopulateImuZeroerState(
      flatbuffers::FlatBufferBuilder *fbb);
  flatbuffers::Offset<DownEstimatorState> PopulateDownEstimatorState(
      flatbuffers::FlatBufferBuilder *fbb,
      aos::monotonic_clock::time_point monotonic_now);
  flatbuffers::Offset<GearLogging> CreateGearLogging(
      flatbuffers::FlatBufferBuilder *fbb) const;

  // Returns the current localizer state.
  Eigen::Matrix<double, 5, 1> trajectory_state() {
    return (Eigen::Matrix<double, 5, 1>() << localizer_->x(), localizer_->y(),
            localizer_->theta(), localizer_->left_velocity(),
            localizer_->right_velocity())
        .finished();
  }

  // Resets all filters when wpilib_interface resets.
  void Reset(aos::monotonic_clock::time_point monotonic_now,
             const drivetrain::Position *position);

  // Corrects all the filters.
  void Correct(aos::monotonic_clock::time_point monotonic_now,
               const drivetrain::Position *position);
  // Runs the predict step for all filters.  Should be called with the undelayed
  // U.
  void UpdateObserver(Eigen::Matrix<double, 2, 1> U);

  // Returns the negative of the voltage error from the drivetrain controller.
  // This can be used for integral control.
  Eigen::Matrix<double, 2, 1> VoltageError() const;

  // Returns the current drivetrain state.
  Eigen::Matrix<double, 7, 1> DrivetrainXHat() const { return kf_.X_hat(); }
  double DrivetrainXHat(int index) const { return kf_.X_hat(index); }

  // Returns the current uncapped voltage from the kalman filter.
  double DrivetrainUUncapped(int index) const { return kf_.U_uncapped(index); }

  bool Ready() const { return ready_; }

 private:
  // Returns the current controller index for the current gear.
  int ControllerIndexFromGears() const;

  // Computes which gear a shifter is in.
  Gear ComputeGear(double shifter_position,
                   const constants::ShifterHallEffect &shifter_config,
                   bool high_requested) const;

  const DrivetrainConfig<double> dt_config_;

  aos::Fetcher<LocalizerControl> localizer_control_fetcher_;
  aos::Fetcher<frc971::IMUValuesBatch> imu_values_fetcher_;
  aos::Fetcher<frc971::sensors::GyroReading> gyro_reading_fetcher_;

  zeroing::ImuZeroer imu_zeroer_;
  DrivetrainUkf down_estimator_;
  aos::monotonic_clock::time_point last_imu_update_ =
      aos::monotonic_clock::min_time;
  LocalizerInterface *localizer_;
  StateFeedbackLoop<7, 2, 4> kf_;

  // Current gears for each drive side.
  Gear left_gear_;
  Gear right_gear_;

  // Shift request.
  bool left_high_requested_;
  bool right_high_requested_;

  // Last acceleration and yaw rate.
  aos::monotonic_clock::time_point last_gyro_time_ =
      aos::monotonic_clock::min_time;
  double last_accel_ = 0.0;
  double last_gyro_rate_ = 0.0;

  bool ready_ = false;

  // Last applied voltage.
  Eigen::Matrix<double, 2, 1> last_voltage_;
  Eigen::Matrix<double, 2, 1> last_last_voltage_;

  std::optional<double> yaw_gyro_zero_;
  zeroing::Averager<double, 200> yaw_gyro_zeroer_;
};

class DrivetrainLoop
    : public frc971::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  // Note that we only actually store N - 1 splines consistently, since we need
  // to keep one fetcher free to check whether there are any new splines.
  static constexpr size_t kNumSplineFetchers =
      SplineDrivetrain::kMaxTrajectories;

  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(const DrivetrainConfig<double> &dt_config,
                          ::aos::EventLoop *event_loop,
                          LocalizerInterface *localizer,
                          const ::std::string &name = "/drivetrain");

  virtual ~DrivetrainLoop() {}

 protected:
  struct TrajectoryFetcherState {
    aos::Fetcher<fb::Trajectory> fetcher;
    bool in_use = false;
  };

  // Executes one cycle of the control loop.
  void RunIteration(
      const ::frc971::control_loops::drivetrain::Goal *goal,
      const ::frc971::control_loops::drivetrain::Position *position,
      aos::Sender<::frc971::control_loops::drivetrain::Output>::Builder *output,
      aos::Sender<::frc971::control_loops::drivetrain::Status>::Builder *status)
      override;

  flatbuffers::Offset<drivetrain::Output> Zero(
      aos::Sender<drivetrain::Output>::Builder *builder) override;

  void UpdateTrajectoryFetchers();

  const DrivetrainConfig<double> dt_config_;
  DrivetrainFilters filters_;
  std::array<TrajectoryFetcherState, kNumSplineFetchers> trajectory_fetchers_;

  PolyDrivetrain<double> dt_openloop_;
  DrivetrainMotorsSS dt_closedloop_;
  SplineDrivetrain dt_spline_;
  LineFollowDrivetrain dt_line_follow_;

  bool has_been_enabled_ = false;

  aos::SendFailureCounter status_failure_counter_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
