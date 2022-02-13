#ifndef Y2022_CONTROL_LOOPS_DRIVETRAIN_LOCALIZER_H_
#define Y2022_CONTROL_LOOPS_DRIVETRAIN_LOCALIZER_H_

#include <string_view>

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "y2022/control_loops/localizer/localizer_output_generated.h"
#include "aos/network/message_bridge_server_generated.h"

namespace y2022 {
namespace control_loops {
namespace drivetrain {

// This class handles the localization for the 2022 robot. Rather than actually
// doing any work on the roborio, we farm all the localization out to a
// raspberry pi and it then sends out LocalizerOutput messages that we treat as
// measurement updates. See //y2022/control_loops/localizer.
// TODO(james): Needs tests. Should refactor out some of the code from the 2020
// localizer test.
class Localizer : public frc971::control_loops::drivetrain::LocalizerInterface {
 public:
  typedef frc971::control_loops::TypedPose<float> Pose;
  typedef frc971::control_loops::drivetrain::HybridEkf<float> HybridEkf;
  typedef typename HybridEkf::State State;
  typedef typename HybridEkf::StateIdx StateIdx;
  typedef typename HybridEkf::StateSquare StateSquare;
  typedef typename HybridEkf::Input Input;
  typedef typename HybridEkf::Output Output;
  Localizer(aos::EventLoop *event_loop,
            const frc971::control_loops::drivetrain::DrivetrainConfig<double>
                &dt_config);
  frc971::control_loops::drivetrain::HybridEkf<double>::State Xhat()
      const override {
    return ekf_.X_hat().cast<double>();
  }
  frc971::control_loops::drivetrain::TrivialTargetSelector *target_selector()
      override {
    return &target_selector_;
  }

  void Update(const ::Eigen::Matrix<double, 2, 1> &U,
              aos::monotonic_clock::time_point now, double left_encoder,
              double right_encoder, double gyro_rate,
              const Eigen::Vector3d &accel) override;

  void Reset(aos::monotonic_clock::time_point t,
             const frc971::control_loops::drivetrain::HybridEkf<double>::State
                 &state) override;

  void ResetPosition(aos::monotonic_clock::time_point t, double x, double y,
                     double theta, double /*theta_override*/,
                     bool /*reset_theta*/) override {
    const double left_encoder = ekf_.X_hat(StateIdx::kLeftEncoder);
    const double right_encoder = ekf_.X_hat(StateIdx::kRightEncoder);
    ekf_.ResetInitialState(t,
                           (HybridEkf::State() << x, y, theta, left_encoder, 0,
                            right_encoder, 0, 0, 0, 0, 0, 0)
                               .finished(),
                           ekf_.P());
  }

 private:
  aos::EventLoop *const event_loop_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  HybridEkf ekf_;

  aos::Fetcher<frc971::controls::LocalizerOutput> localizer_output_fetcher_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> clock_offset_fetcher_;

  // Target selector to allow us to satisfy the LocalizerInterface requirements.
  frc971::control_loops::drivetrain::TrivialTargetSelector target_selector_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2022

#endif  // Y2022_CONTROL_LOOPS_DRIVETRAIN_LOCALIZER_H_
