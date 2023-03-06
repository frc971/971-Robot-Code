#include <string>

#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aprilrobotics.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/vision_generated.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/constants/constants_list_generated.h"

namespace y2023::vision::testing {
class AprilDetectionTest : public ::testing::Test {
 public:
  AprilDetectionTest()
      : config_(aos::configuration::ReadConfig("y2023/aos_config.json")),
        event_loop_factory_(&config_.message()),
        pi_(aos::configuration::GetNode(event_loop_factory_.configuration(),
                                        "pi4")),
        send_pose_event_loop_(
            event_loop_factory_.MakeEventLoop("Send pose", pi_)),
        receive_pose_event_loop_(
            event_loop_factory_.MakeEventLoop("Receive pose", pi_)),
        april_pose_fetcher_(
            receive_pose_event_loop_->MakeFetcher<frc971::vision::TargetMap>(
                "/camera")),
        image_sender_(
            receive_pose_event_loop_->MakeSender<frc971::vision::CameraImage>(
                "/camera")),
        constants_sender_(receive_pose_event_loop_.get(),
                          "y2023/constants/constants.json", 7971, "/constants"),
        detector_(
            AprilRoboticsDetector(send_pose_event_loop_.get(), "/camera")) {}

  void SendImage(std::string path) {
    aos::FlatbufferVector<frc971::vision::CameraImage> image =
        aos::FileToFlatbuffer<frc971::vision::CameraImage>(
            "external/apriltag_test_bfbs_images/" + path);

    auto builder = image_sender_.MakeBuilder();
    flatbuffers::Offset<frc971::vision::CameraImage> image_fbs =
        aos::CopyFlatBuffer(image, builder.fbb());

    builder.CheckOk(builder.Send(image_fbs));
  }

  void TestDistanceAngle(std::string image_path, double expected_distance,
                         double expected_angle) {
    receive_pose_event_loop_->OnRun([&]() { SendImage(image_path); });
    event_loop_factory_.RunFor(std::chrono::milliseconds(5));

    ASSERT_TRUE(april_pose_fetcher_.Fetch());
    ASSERT_EQ(april_pose_fetcher_->target_poses()->size(), 1);

    frc971::vision::TargetMapper::TargetPose target_pose =
        frc971::vision::PoseUtils::TargetPoseFromFbs(
            *april_pose_fetcher_->target_poses()->Get(0));

    ASSERT_EQ(april_pose_fetcher_->target_poses()->Get(0)->id(), 8);

    // Height
    EXPECT_NEAR(target_pose.pose.p.y(), -0.28, 0.05);

    // Tag to camera horizontal distance
    double distance_norm =
        sqrt(pow(target_pose.pose.p.x(), 2) + pow(target_pose.pose.p.z(), 2));
    EXPECT_NEAR(distance_norm, expected_distance, 0.1);

    Eigen::Vector3d rotation_euler =
        frc971::vision::PoseUtils::QuaternionToEulerAngles(target_pose.pose.q);

    EXPECT_NEAR(rotation_euler[0], 0, 0.1);
    EXPECT_NEAR(rotation_euler[1], expected_angle, 0.05);
    EXPECT_NEAR(rotation_euler[2], 0, 0.1);
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const pi_;
  ::std::unique_ptr<::aos::EventLoop> send_pose_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> receive_pose_event_loop_;
  aos::Fetcher<frc971::vision::TargetMap> april_pose_fetcher_;
  aos::Sender<frc971::vision::CameraImage> image_sender_;
  frc971::constants::ConstantSender<y2023::Constants, y2023::ConstantsList>
      constants_sender_;
  AprilRoboticsDetector detector_;
};

TEST_F(AprilDetectionTest, CheckPose5Feet) {
  TestDistanceAngle("bfbs-capture-2013-01-18_08-54-09.501047728.bfbs", 1.5, 0);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-54-16.869057537.bfbs", 1.5,
                    0.22);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-54-24.931661979.bfbs", 1.5,
                    -0.37);
}

TEST_F(AprilDetectionTest, CheckPose10Feet) {
  TestDistanceAngle("bfbs-capture-2013-01-18_08-51-24.861065764.bfbs", 3.07, 0);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-52-01.846912552.bfbs", 3.07,
                    0.31);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-52-33.462848049.bfbs", 3.07,
                    -0.27);
}

TEST_F(AprilDetectionTest, CheckPose15Feet) {
  // The camera was not at a perfect angle of 0, so the angle is -0.15 radians
  // instead of 0
  TestDistanceAngle("bfbs-capture-2013-01-18_09-29-16.806073486.bfbs", 4.57,
                    -0.15);
  TestDistanceAngle("bfbs-capture-2013-01-18_09-33-00.993756514.bfbs", 4.57,
                    0.38);
}

TEST_F(AprilDetectionTest, CheckPose20Feet) {
  // The camera was not at a perfect angle of 0, so the angle is 0.09 radians
  // instead of 0
  TestDistanceAngle("bfbs-capture-2013-01-18_08-57-00.171120695.bfbs", 6.06,
                    0.09);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-57-17.858752817.bfbs", 6.06,
                    0.35);
  TestDistanceAngle("bfbs-capture-2013-01-18_08-57-08.096597542.bfbs", 6.06,
                    -0.45);
}
}  // namespace y2023::vision::testing
