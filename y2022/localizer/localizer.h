#ifndef Y2022_LOCALIZER_LOCALIZER_H_
#define Y2022_LOCALIZER_LOCALIZER_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "aos/containers/ring_buffer.h"
#include "aos/containers/sized_array.h"
#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/zeroing/imu_zeroer.h"
#include "frc971/zeroing/wrap.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/localizer/localizer_output_generated.h"
#include "y2022/localizer/localizer_status_generated.h"
#include "y2022/localizer/localizer_visualization_generated.h"
#include "y2022/vision/target_estimate_generated.h"

namespace frc971::controls {

namespace testing {
class LocalizerTest;
}

// Localizer implementation that makes use of a 6-axis IMU, encoder readings,
// drivetrain voltages, and camera returns to localize the robot. Meant to
// be run on a raspberry pi.
//
// This operates on the principle that the drivetrain can be in one of two
// modes:
// 1) A "normal" mode where it is obeying the regular drivetrain model, with
//    minimal lateral motion and no major external disturbances. This is
//    referred to as the "model" mode in the code/variable names.
// 2) An non-holonomic mode where the robot is just flying around on a 2-D
//    plane with no meaningful constraints (referred to as an "accel" model
//    in the code, because we rely primarily on the accelerometer readings).
//
// In order to determine which mode to be in, we attempt to track whether the
// two models are diverging substantially. In order to do this, we maintain a
// 1-second long queue of "branches". A branch is generated every X iterations
// and contains a model state and an accel state. When the branch starts, the
// two will have identical states. For the remaining 1 second, the model state
// will evolve purely according to the drivetrian model, and the accel state
// will evolve purely using IMU readings.
//
// When the branch reaches 1 second in age, we calculate a residual associated
// with how much the accel and model based states diverged. If they have
// diverged substantially, that implies that the model is a poor match for
// whatever has been happening to the robot in the past second, so if we were
// previously relying on the model, we will override the current "actual"
// state with the branched accel state, and then continue to update the accel
// state based on IMU readings.
// If we are currently in the accel state, we will continue generating branches
// until the branches stop diverging--this will indicate that the model
// matches the accelerometer readings again, and so we will swap back to
// the model-based state.
//
// TODO:
// * Implement paying attention to camera readings.
// * Tune for ADIS16505/real robot.
class ModelBasedLocalizer {
 public:
  static constexpr size_t kX = 0;
  static constexpr size_t kY = 1;
  static constexpr size_t kTheta = 2;

  static constexpr size_t kVelocityX = 3;
  static constexpr size_t kVelocityY = 4;
  static constexpr size_t kNAccelStates = 5;

  static constexpr size_t kLeftEncoder = 3;
  static constexpr size_t kLeftVelocity = 4;
  static constexpr size_t kLeftVoltageError = 5;
  static constexpr size_t kRightEncoder = 6;
  static constexpr size_t kRightVelocity = 7;
  static constexpr size_t kRightVoltageError = 8;
  static constexpr size_t kNModelStates = 9;

  static constexpr size_t kNModelOutputs = 3;

  static constexpr size_t kNAccelOuputs = 1;

  static constexpr size_t kAccelX = 0;
  static constexpr size_t kAccelY = 1;
  static constexpr size_t kThetaRate = 2;
  static constexpr size_t kNAccelInputs = 3;

  static constexpr size_t kLeftVoltage = 0;
  static constexpr size_t kRightVoltage = 1;
  static constexpr size_t kNModelInputs = 2;

  // Branching period, in cycles.
  // Needs 10 to even stay alive, and still at ~96% CPU.
  // ~20 gives ~55-60% CPU.
  static constexpr int kBranchPeriod = 100;

  static constexpr size_t kDebugBufferSize = 10;

  typedef Eigen::Matrix<double, kNModelStates, 1> ModelState;
  typedef Eigen::Matrix<double, kNAccelStates, 1> AccelState;
  typedef Eigen::Matrix<double, kNModelInputs, 1> ModelInput;
  typedef Eigen::Matrix<double, kNAccelInputs, 1> AccelInput;

  ModelBasedLocalizer(
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config);
  void HandleImu(aos::monotonic_clock::time_point t,
                 const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel,
                 const Eigen::Vector2d encoders, const Eigen::Vector2d voltage);
  void HandleTurret(aos::monotonic_clock::time_point sample_time,
                    double turret_position, double turret_velocity);
  void HandleImageMatch(aos::monotonic_clock::time_point sample_time,
                        const y2022::vision::TargetEstimate *target,
                        int camera_index);
  void HandleReset(aos::monotonic_clock::time_point,
                   const Eigen::Vector3d &xytheta);

  flatbuffers::Offset<ModelBasedStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  Eigen::Vector3d xytheta() const {
    if (using_model_) {
      return current_state_.model_state.block<3, 1>(0, 0);
    } else {
      return current_state_.accel_state.block<3, 1>(0, 0);
    }
  }

  Eigen::Quaterniond orientation() const { return last_orientation_; }

  AccelState accel_state() const { return current_state_.accel_state; };

  void set_longitudinal_offset(double offset) { long_offset_ = offset; }

  void TallyRejection(const RejectionReason reason);

  flatbuffers::Offset<LocalizerVisualization> PopulateVisualization(
      flatbuffers::FlatBufferBuilder *fbb);

  size_t NumQueuedImageDebugs() const { return image_debugs_.size(); }

 private:
  struct CombinedState {
    AccelState accel_state = AccelState::Zero();
    ModelState model_state = ModelState::Zero();
    aos::monotonic_clock::time_point branch_time =
        aos::monotonic_clock::min_time;
    double accumulated_divergence = 0.0;
  };

  // Struct to store state data needed to perform a camera correction, since
  // camera corrections require looking back in time.
  struct OldPosition {
    aos::monotonic_clock::time_point sample_time =
        aos::monotonic_clock::min_time;
    Eigen::Vector3d xytheta = Eigen::Vector3d::Zero();
    double turret_position = 0.0;
    double turret_velocity = 0.0;
  };

  static flatbuffers::Offset<AccelBasedState> BuildAccelState(
      flatbuffers::FlatBufferBuilder *fbb, const AccelState &state);

  static flatbuffers::Offset<ModelBasedState> BuildModelState(
      flatbuffers::FlatBufferBuilder *fbb, const ModelState &state);

  Eigen::Matrix<double, kNModelStates, kNModelStates> AModel(
      const ModelState &state) const;
  Eigen::Matrix<double, kNAccelStates, kNAccelStates> AAccel() const;
  ModelState DiffModel(const ModelState &state, const ModelInput &U) const;
  AccelState DiffAccel(const AccelState &state, const AccelInput &U) const;

  ModelState UpdateModel(const ModelState &model, const ModelInput &input,
                         aos::monotonic_clock::duration dt) const;
  AccelState UpdateAccel(const AccelState &accel, const AccelInput &input,
                         aos::monotonic_clock::duration dt) const;

  AccelState AccelStateForModelState(const ModelState &state) const;
  ModelState ModelStateForAccelState(const AccelState &state,
                                     const Eigen::Vector2d &encoders,
                                     const double yaw_rate) const;
  double ModelDivergence(const CombinedState &state,
                         const AccelInput &accel_inputs,
                         const Eigen::Vector2d &filtered_accel,
                         const ModelInput &model_inputs);
  void UpdateState(
      CombinedState *state,
      const Eigen::Matrix<double, kNModelStates, kNModelOutputs> &K,
      const Eigen::Matrix<double, kNModelOutputs, 1> &Z,
      const Eigen::Matrix<double, kNModelOutputs, kNModelStates> &H,
      const AccelInput &accel_input, const ModelInput &model_input,
      aos::monotonic_clock::duration dt);

  const OldPosition GetStateForTime(aos::monotonic_clock::time_point time);

  // Returns the transformation to get from the camera frame to the robot frame
  // for the specified state.
  const Eigen::Matrix<double, 4, 4> CameraTransform(
      const OldPosition &state,
      const frc971::vision::calibration::CameraCalibration *calibration,
      std::optional<RejectionReason> *rejection_reason) const;

  // Returns the robot x/y position implied by the specified camera data and
  // estimated state from that time.
  // H_field_camera is passed in so that it can be used as a debugging output.
  const std::optional<Eigen::Vector2d> CameraMeasuredRobotPosition(
      const OldPosition &state, const y2022::vision::TargetEstimate *target,
      std::optional<RejectionReason> *rejection_reason,
      Eigen::Matrix<double, 4, 4> *H_field_camera_measured) const;

  flatbuffers::Offset<TargetEstimateDebug> PackTargetEstimateDebug(
      const TargetEstimateDebugT &debug, flatbuffers::FlatBufferBuilder *fbb);

  flatbuffers::Offset<CumulativeStatistics> PopulateStatistics(
      flatbuffers::FlatBufferBuilder *fbb);

  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  const StateFeedbackHybridPlantCoefficients<2, 2, 2, double>
      velocity_drivetrain_coefficients_;
  Eigen::Matrix<double, kNModelStates, kNModelStates> A_continuous_model_;
  Eigen::Matrix<double, kNAccelStates, kNAccelStates> A_continuous_accel_;
  Eigen::Matrix<double, kNAccelStates, kNAccelStates> A_discrete_accel_;
  Eigen::Matrix<double, kNModelStates, kNModelInputs> B_continuous_model_;
  Eigen::Matrix<double, kNAccelStates, kNAccelInputs> B_continuous_accel_;

  Eigen::Matrix<double, kNModelStates, kNModelStates> Q_continuous_model_;

  Eigen::Matrix<double, kNModelStates, kNModelStates> P_model_;

  Eigen::Matrix<double, kNAccelStates, kNAccelStates> Q_continuous_accel_;
  Eigen::Matrix<double, kNAccelStates, kNAccelStates> Q_discrete_accel_;

  Eigen::Matrix<double, kNAccelStates, kNAccelStates> P_accel_;

  control_loops::drivetrain::DrivetrainUkf down_estimator_;

  // When we are following the model, we will, on each iteration:
  // 1) Perform a model-based update of a single state.
  // 2) Add a hypothetical non-model-based entry based on the current state.
  // 3) Evict too-old non-model-based entries.

  // Buffer of old branches these are all created by initializing a new
  // model-based state based on the current state, and then initializing a new
  // accel-based state on top of that new model-based state (to eliminate the
  // impact of any lateral motion).
  // We then integrate up all of these states and observe how much the model and
  // accel based states of each branch compare to one another.
  aos::RingBuffer<CombinedState, 2000 / kBranchPeriod> branches_;
  int branch_counter_ = 0;

  // Buffer of old x/y/theta positions. This is used so that we can have a
  // reference for exactly where we thought a camera was when it took an image.
  aos::RingBuffer<OldPosition, 500 / kBranchPeriod> old_positions_;

  CombinedState current_state_;
  aos::monotonic_clock::time_point t_ = aos::monotonic_clock::min_time;
  bool using_model_;

  // X position of the IMU, in meters. 0 = center of robot, positive = ahead of
  // center, negative = behind center.
  double long_offset_ = -0.15;

  double last_residual_ = 0.0;
  double filtered_residual_ = 0.0;
  Eigen::Vector2d filtered_residual_accel_ = Eigen::Vector2d::Zero();
  Eigen::Vector3d abs_accel_ = Eigen::Vector3d::Zero();
  double velocity_residual_ = 0.0;
  double accel_residual_ = 0.0;
  double theta_rate_residual_ = 0.0;
  int hysteresis_count_ = 0;
  Eigen::Quaterniond last_orientation_ = Eigen::Quaterniond::Identity();

  int clock_resets_ = 0;

  std::optional<aos::monotonic_clock::time_point> last_turret_update_;
  double latest_turret_position_ = 0.0;
  double latest_turret_velocity_ = 0.0;

  // Stuff to track faults.
  static constexpr size_t kNumRejectionReasons =
      static_cast<int>(RejectionReason::MAX) -
      static_cast<int>(RejectionReason::MIN) + 1;

  struct Statistics {
    int total_accepted = 0;
    int total_candidates = 0;
    static_assert(0 == static_cast<int>(RejectionReason::MIN));
    static_assert(
        kNumRejectionReasons ==
            sizeof(
                std::invoke_result<decltype(EnumValuesRejectionReason)>::type) /
                sizeof(RejectionReason),
        "RejectionReason has non-contiguous error values.");
    std::array<int, kNumRejectionReasons> rejection_counts;
  };
  Statistics statistics_;

  aos::SizedArray<TargetEstimateDebugT, kDebugBufferSize> image_debugs_;

  friend class testing::LocalizerTest;
};

class EventLoopLocalizer {
 public:
  static constexpr std::chrono::milliseconds kMinVisualizationPeriod{100};

  EventLoopLocalizer(
      aos::EventLoop *event_loop,
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config);

  ModelBasedLocalizer *localizer() { return &model_based_; }

 private:
  std::optional<aos::monotonic_clock::duration> ClockOffset(
      std::string_view pi);
  aos::EventLoop *event_loop_;
  ModelBasedLocalizer model_based_;
  aos::Sender<LocalizerStatus> status_sender_;
  aos::Sender<LocalizerOutput> output_sender_;
  aos::Sender<LocalizerVisualization> visualization_sender_;
  aos::Fetcher<frc971::control_loops::drivetrain::Output> output_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;
  zeroing::ImuZeroer zeroer_;
  aos::monotonic_clock::time_point last_output_send_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point last_visualization_send_ =
      aos::monotonic_clock::min_time;
  std::optional<aos::monotonic_clock::time_point> last_pico_timestamp_;
  aos::monotonic_clock::duration pico_offset_error_;
  // t = pico_offset_ + pico_timestamp.
  // Note that this can drift over sufficiently long time periods!
  std::optional<std::chrono::nanoseconds> pico_offset_;

  zeroing::UnwrapSensor left_encoder_;
  zeroing::UnwrapSensor right_encoder_;
};
}  // namespace frc971::controls
#endif  // Y2022_LOCALIZER_LOCALIZER_H_
