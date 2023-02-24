#ifndef Y2023_LOCALIZER_LOCALIZER_H_
#define Y2023_LOCALIZER_LOCALIZER_H_

#include <array>
#include <map>

#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "frc971/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc971/control_loops/drivetrain/localization/utils.h"
#include "frc971/imu_reader/imu_watcher.h"
#include "frc971/vision/target_map_generated.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/localizer/status_generated.h"
#include "y2023/localizer/visualization_generated.h"

namespace y2023::localizer {

class Localizer {
 public:
  static constexpr size_t kNumCameras = 4;
  typedef Eigen::Matrix<double, 4, 4> Transform;
  typedef frc971::control_loops::drivetrain::HybridEkf<double> HybridEkf;
  typedef HybridEkf::State State;
  typedef HybridEkf::Output Output;
  typedef HybridEkf::Input Input;
  typedef HybridEkf::StateIdx StateIdx;
  Localizer(aos::EventLoop *event_loop,
            const frc971::control_loops::drivetrain::DrivetrainConfig<double>
                dt_config);

 private:
  class Corrector : public HybridEkf::ExpectedObservationFunctor {
   public:
    // Indices used for each of the members of the output vector for this
    // Corrector.
    enum OutputIdx {
      kX = 0,
      kY = 1,
      kTheta = 2,
    };
    Corrector(const State &state_at_capture, const Eigen::Vector3d &Z)
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
    aos::Sender<Visualization> debug_sender;
    Transform extrinsics = Transform::Zero();
    aos::util::ArrayErrorCounter<RejectionReason, RejectionCount>
        rejection_counter;
    size_t total_candidate_targets = 0;
    size_t total_accepted_targets = 0;
  };

  static std::array<CameraState, kNumCameras> MakeCameras(
      const Constants &constants, aos::EventLoop *event_loop);
  flatbuffers::Offset<TargetEstimateDebug> HandleTarget(
      int camera_index, const aos::monotonic_clock::time_point capture_time,
      const frc971::vision::TargetPoseFbs &target,
      flatbuffers::FlatBufferBuilder *debug_fbb);
  void HandleImu(aos::monotonic_clock::time_point sample_time_pico,
                 aos::monotonic_clock::time_point sample_time_pi,
                 std::optional<Eigen::Vector2d> encoders, Eigen::Vector3d gyro,
                 Eigen::Vector3d accel);
  flatbuffers::Offset<TargetEstimateDebug> RejectImage(
      int camera_index, RejectionReason reason,
      TargetEstimateDebug::Builder *builder);

  void SendOutput();
  flatbuffers::Offset<frc971::control_loops::drivetrain::LocalizerState>
  PopulateState(flatbuffers::FlatBufferBuilder *fbb) const;
  flatbuffers::Offset<ImuStatus> PopulateImu(
      flatbuffers::FlatBufferBuilder *fbb) const;
  void SendStatus();
  static flatbuffers::Offset<CumulativeStatistics> StatisticsForCamera(
      const CameraState &camera, flatbuffers::FlatBufferBuilder *fbb);

  aos::EventLoop *const event_loop_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  std::array<CameraState, kNumCameras> cameras_;
  const std::array<Transform, kNumCameras> camera_extrinsics_;
  const std::map<uint64_t, Transform> target_poses_;

  frc971::control_loops::drivetrain::DrivetrainUkf down_estimator_;
  HybridEkf ekf_;
  HybridEkf::ExpectedObservationAllocator<Corrector> observations_;

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
};
}  // namespace y2023::localizer
#endif  // Y2023_LOCALIZER_LOCALIZER_H_
