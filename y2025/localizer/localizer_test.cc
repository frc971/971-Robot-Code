#include "y2025/localizer/localizer.h"

#include <chrono>
#include <optional>
#include <random>

#include "absl/flags/flag.h"
#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "y2025/constants/simulated_constants_sender.h"

namespace y2025::localizer::testing {

class LocalizerTest : public ::testing::Test {
 protected:
  LocalizerTest()
      : configuration_(aos::configuration::ReadConfig("y2025/aos_config.json")),
        event_loop_factory_(&configuration_.message()),
        imu_node_(
            absl::GetFlag(FLAGS_use_one_orin) ? std::make_optional([this]() {
              // Get the constants sent before anything else happens.
              // It has nothing to do with the roborio node.
              SendSimulationConstants(&event_loop_factory_, 971,
                                      "y2025/constants/test_constants.json");
              return aos::configuration::GetNode(
                  &configuration_.message(),
                  absl::GetFlag(FLAGS_use_one_orin) ? "/orin1" : "/imu");
            }())
                                              : std::nullopt),
        orin1_node_(
            aos::configuration::GetNode(&configuration_.message(), "orin1")),
        roborio_node_(
            aos::configuration::GetNode(&configuration_.message(), "roborio")),
        localizer_event_loop_(event_loop_factory_.MakeEventLoop(
            "localizer", absl::GetFlag(FLAGS_use_one_orin)
                             ? orin1_node_
                             : imu_node_.value())),
        imu_event_loop_(event_loop_factory_.MakeEventLoop(
            "test", absl::GetFlag(FLAGS_use_one_orin) ? orin1_node_
                                                      : imu_node_.value())),
        orin1_event_loop_(
            event_loop_factory_.MakeEventLoop("test", orin1_node_)),
        localizer_(localizer_event_loop_.get()) {
    robot_pose_.setZero();

    // just directly use calib data
    Localizer::Transform H_robot_camera;
    H_robot_camera << -1, -0, 0, -0.18289103, 0, 0.17364818, -0.9848077,
        -0.24428031, 0, -0.9848077, -0.17364818, 0.5905655, 0, 0, 0, 1;

    orin1_event_loop_->AddPhasedLoop(
        [this, H_robot_camera](int) {
          for (const auto info : orin1_detected_tags) {
            double flakey_chance = GaussianRandomVector<1>(1.0)(0, 0);
            if (info.flakey && (flakey_chance - 0.5) < 0.075) {
              continue;
            }
          }

          const Localizer::Pose pose({robot_pose_(Localizer::PoseIdx::kX),
                                      robot_pose_(Localizer::PoseIdx::kY), 0.0},
                                     robot_pose_(Localizer::PoseIdx::kTheta));

          const Localizer::Transform H_robot_field =
              pose.AsTransformationMatrix();

          const Localizer::Transform H_camera_field =
              H_robot_field * H_robot_camera.inverse();

          (void)H_robot_field;
          (void)H_camera_field;
        },
        std::chrono::milliseconds(20));
  }

  enum Node { IMU, Orin1 };

  struct TagInfo {
    int camera;
    int target_id;
    bool flakey;
  };

  void SeeTag(Node node, TagInfo info) {
    switch (node) {
      case IMU:
        imu_detected_tags.push_back(info);
        break;
      case Orin1:
        orin1_detected_tags.push_back(info);
        break;
    };
  }

 private:
  template <int N>
  Eigen::Matrix<double, N, 1> GaussianRandomVector(double standard_deviation) {
    std::normal_distribution distribution{0.0, standard_deviation};
    Eigen::Matrix<double, N, 1> random;
    for (int index = 0; index < N; ++index) {
      random(index) = distribution(random_engine_);
    }
    return random;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  aos::SimulatedEventLoopFactory event_loop_factory_;

  std::optional<const aos::Node *const> imu_node_;
  const aos::Node *const orin1_node_;
  const aos::Node *const roborio_node_;
  std::unique_ptr<aos::EventLoop> localizer_event_loop_;
  std::unique_ptr<aos::EventLoop> imu_event_loop_;
  std::unique_ptr<aos::EventLoop> orin1_event_loop_;

  WeightedAverageLocalizer localizer_;

  Localizer::XYThetaPose robot_pose_;

  std::vector<TagInfo> imu_detected_tags;
  std::vector<TagInfo> orin1_detected_tags;

  std::mt19937 random_engine_;
};

// TODO(max): Add actual test in future commit
TEST_F(LocalizerTest, DoNothing) { LOG(INFO) << "test"; }

}  // namespace y2025::localizer::testing
