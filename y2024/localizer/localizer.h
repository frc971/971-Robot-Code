#ifndef Y2024_LOCALIZER_LOCALIZER_H_
#define Y2024_LOCALIZER_LOCALIZER_H_

#include <array>
#include <map>

#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "frc971/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc971/control_loops/drivetrain/localization/utils.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/imu_reader/imu_watcher.h"
#include "frc971/vision/target_map_generated.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/localizer/status_generated.h"
#include "y2024/localizer/visualization_static.h"

namespace y2024::localizer {

class Localizer {
 public:
  static constexpr size_t kNumCameras = 4;
  using Pose = frc971::control_loops::Pose;
  typedef Eigen::Matrix<double, 4, 4> Transform;
  typedef frc971::control_loops::drivetrain::HybridEkf<double> HybridEkf;
  typedef HybridEkf::State State;
  typedef HybridEkf::Output Output;
  typedef HybridEkf::Input Input;
  typedef HybridEkf::StateIdx StateIdx;
  Localizer(aos::EventLoop *event_loop);

 private:
  class Corrector : public HybridEkf::ExpectedObservationFunctor {
   public:
    // Indices used for each of the members of the output vector for this
    // Corrector.
    enum OutputIdx {
      kHeading = 0,
      kDistance = 1,
      kSkew = 2,
    };
    Corrector(const State &state_at_capture, const Transform &H_field_target,
              const Transform &H_robot_camera,
              const Transform &H_camera_target);

    using HMatrix = Eigen::Matrix<double, Localizer::HybridEkf::kNOutputs,
                                  Localizer::HybridEkf::kNStates>;

    Output H(const State &, const Input &) final;
    HMatrix DHDX(const State &) final { return H_; }
    const Eigen::Vector3d &observed() const { return observed_; }
    const Eigen::Vector3d &expected() const { return expected_; }
    const Pose &expected_robot_pose() const { return expected_robot_pose_; }
    const Pose &expected_camera_pose() const { return expected_camera_; }
    const Pose &observed_camera_pose() const { return observed_camera_; }

    static Eigen::Vector3d HeadingDistanceSkew(const Pose &relative_pose);

    static Corrector CalculateHeadingDistanceSkewH(
        const State &state_at_capture, const Transform &H_field_target,
        const Transform &H_robot_camera, const Transform &H_camera_target);

    static void PopulateMeasurement(const Eigen::Vector3d &vector,
                                    MeasurementStatic *builder) {
      builder->set_heading(vector(kHeading));
      builder->set_distance(vector(kDistance));
      builder->set_skew(vector(kSkew));
    }

   private:
    Corrector(const Pose &expected_robot_pose, const Pose &observed_camera,
              const Pose &expected_camera, const Eigen::Vector3d &expected,
              const Eigen::Vector3d &observed, const HMatrix &H)
        : expected_robot_pose_(expected_robot_pose),
          observed_camera_(observed_camera),
          expected_camera_(expected_camera),
          expected_(expected),
          observed_(observed),
          H_(H) {}
    // For debugging.
    const Pose expected_robot_pose_;
    const Pose observed_camera_;
    const Pose expected_camera_;
    // Actually used.
    const Eigen::Vector3d expected_;
    const Eigen::Vector3d observed_;
    const HMatrix H_;
  };

  // A corrector that just does x/y/theta based corrections rather than doing
  // heading/distance/skew corrections.
  class XyzCorrector : public HybridEkf::ExpectedObservationFunctor {
   public:
    // Indices used for each of the members of the output vector for this
    // Corrector.
    enum OutputIdx {
      kX = 0,
      kY = 1,
      kTheta = 2,
    };
    XyzCorrector(const State &state_at_capture, const Eigen::Vector3d &Z)
        : state_at_capture_(state_at_capture), Z_(Z) {
      H_.setZero();
      H_(kX, StateIdx::kX) = 1;
      H_(kY, StateIdx::kY) = 1;
      H_(kTheta, StateIdx::kTheta) = 1;
    }
    Output H(const State &, const Input &) final;
    Eigen::Matrix<double, HybridEkf::kNOutputs, HybridEkf::kNStates> DHDX(
        const State &) final {
      return H_;
    }

   private:
    Eigen::Matrix<double, HybridEkf::kNOutputs, HybridEkf::kNStates> H_;
    const State state_at_capture_;
    const Eigen::Vector3d &Z_;
  };

  struct CameraState {
    aos::Sender<VisualizationStatic> debug_sender;
    Transform extrinsics = Transform::Zero();
    aos::util::ArrayErrorCounter<RejectionReason, RejectionCount>
        rejection_counter;
    size_t total_candidate_targets = 0;
    size_t total_accepted_targets = 0;
  };

  // Returns true if we should use a lower weight for the specified april tag.
  // This is used for tags where we do not trust the placement as much.
  bool DeweightAprilTag(uint64_t target_id);
  static std::array<CameraState, kNumCameras> MakeCameras(
      const Constants &constants, aos::EventLoop *event_loop);
  void HandleTarget(int camera_index,
                    const aos::monotonic_clock::time_point capture_time,
                    const frc971::vision::TargetPoseFbs &target,
                    TargetEstimateDebugStatic *debug_builder);
  void HandleImu(aos::monotonic_clock::time_point sample_time_pico,
                 aos::monotonic_clock::time_point sample_time_pi,
                 std::optional<Eigen::Vector2d> encoders, Eigen::Vector3d gyro,
                 Eigen::Vector3d accel);
  void RejectImage(int camera_index, RejectionReason reason,
                   TargetEstimateDebugStatic *builder);

  void SendOutput();
  static flatbuffers::Offset<frc971::control_loops::drivetrain::LocalizerState>
  PopulateState(const State &X_hat, flatbuffers::FlatBufferBuilder *fbb);
  flatbuffers::Offset<ImuStatus> PopulateImu(
      flatbuffers::FlatBufferBuilder *fbb) const;
  void SendStatus();
  static flatbuffers::Offset<CumulativeStatistics> StatisticsForCamera(
      const CameraState &camera, flatbuffers::FlatBufferBuilder *fbb);
  static void StatisticsForCamera(const CameraState &camera,
                                  CumulativeStatisticsStatic *builder);

  bool UseAprilTag(uint64_t target_id);
  void HandleControl(
      const frc971::control_loops::drivetrain::LocalizerControl &msg);

  aos::EventLoop *const event_loop_;
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  std::array<CameraState, kNumCameras> cameras_;
  const std::map<uint64_t, Transform> target_poses_;

  frc971::control_loops::drivetrain::DrivetrainUkf down_estimator_;
  HybridEkf ekf_;
  HybridEkf::ExpectedObservationAllocator<Corrector> observations_;
  HybridEkf::ExpectedObservationAllocator<XyzCorrector> xyz_observations_;

  frc971::controls::ImuWatcher imu_watcher_;
  frc971::control_loops::drivetrain::LocalizationUtils utils_;

  aos::Sender<Status> status_sender_;
  aos::Sender<frc971::controls::LocalizerOutput> output_sender_;
  aos::monotonic_clock::time_point t_ = aos::monotonic_clock::min_time;
  size_t clock_resets_ = 0;

  size_t total_candidate_targets_ = 0;
  size_t total_accepted_targets_ = 0;

  // For the status message.
  std::optional<Eigen::Vector2d> last_encoder_readings_;

  aos::Fetcher<aos::message_bridge::ServerStatistics>
      server_statistics_fetcher_;
  aos::Fetcher<aos::message_bridge::ClientStatistics>
      client_statistics_fetcher_;
  aos::Fetcher<frc971::control_loops::drivetrain::LocalizerControl>
      control_fetcher_;
};
}  // namespace y2024::localizer
#endif  // Y2024_LOCALIZER_LOCALIZER_H_
