#include "y2017/control_loops/superstructure/vision_time_adjuster.h"

#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/test_logging.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/vision/vision_generated.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

class VisionTimeAdjusterTest : public ::testing::Test {
 public:
  VisionTimeAdjusterTest()
      : ::testing::Test(),
        configuration_(aos::configuration::ReadConfig("y2017/config.json")),
        event_loop_factory_(&configuration_.message()),
        simulation_event_loop_(event_loop_factory_.MakeEventLoop("drivetrain")),
        drivetrain_status_sender_(
            simulation_event_loop_
                ->MakeSender<::frc971::control_loops::drivetrain::Status>(
                    "/drivetrain")),
        vision_status_sender_(
            simulation_event_loop_->MakeSender<::y2017::vision::VisionStatus>(
                "/vision")),
        vision_time_adjuster_event_loop_(
            event_loop_factory_.MakeEventLoop("vision_time_adjuster")),
        vision_status_fetcher_(
            vision_time_adjuster_event_loop_
                ->MakeFetcher<::y2017::vision::VisionStatus>("/vision")),
        adjuster_(vision_time_adjuster_event_loop_.get()) {
    ::aos::testing::EnableTestLogging();
    static_assert(kVisionDelay % kTimeTick == ::std::chrono::milliseconds(0),
                  "kVisionDelay must be a multiple of kTimeTick.");
    static_assert(
        kVisionDelay / kTimeTick == decltype(turret_history_)::kBufferSize,
        "We need a different amount of space to hold the turret data.");
  }

  ~VisionTimeAdjusterTest() {}

 protected:
  static constexpr ::std::chrono::milliseconds kTimeTick{5};

  // We get frames at 20 Hz and we assume a 15 ms delay.
  static constexpr ::std::chrono::milliseconds kVisionTick{50};
  static constexpr ::std::chrono::milliseconds kVisionDelay{15};

  void SetTurretPosition(double turret) { turret_angle_ = turret; }

  void SetDrivetrainPosition(double left, double right) {
    drivetrain_left_ = left;
    drivetrain_right_ = right;
  }

  void SetVisionAngle(double angle) { vision_angle_ = angle; }

  void RunIteration() {
    SendMessages();
    const vision::VisionStatus *vision_status = nullptr;
    if (vision_status_fetcher_.Fetch()) {
      vision_status = vision_status_fetcher_.get();
    }
    adjuster_.Tick(vision_time_adjuster_event_loop_->monotonic_now(),
                   turret_angle_, vision_status);
    TickTime();
  }

  static constexpr size_t GetVisionDelayCount() {
    return kVisionDelay / kTimeTick;
  }

  VisionTimeAdjuster *adjuster() { return &adjuster_; }

 private:
  void TickTime() { event_loop_factory_.RunFor(kTimeTick); }

  void SendMessages() {
    SendDrivetrainPosition();
    if (event_loop_factory_.monotonic_now().time_since_epoch() %
            kVisionTick ==
        kVisionDelay) {
      SendVisionTarget();
    }
    turret_history_.Push(turret_angle_);
    drivetrain_history_.Push(GetDriveTrainAngle());
  }

  void SendDrivetrainPosition() {
    auto builder = drivetrain_status_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::Status::Builder status_builder =
        builder.MakeBuilder<frc971::control_loops::drivetrain::Status>();
    status_builder.add_estimated_left_position(drivetrain_left_);
    status_builder.add_estimated_right_position(drivetrain_right_);

    ASSERT_TRUE(builder.Send(status_builder.Finish()));
  }

  void SendVisionTarget() {
    auto builder = vision_status_sender_.MakeBuilder();

    vision::VisionStatus::Builder vision_status_builder =
        builder.MakeBuilder<vision::VisionStatus>();
    vision_status_builder.add_target_time(
        (simulation_event_loop_->monotonic_now() - kVisionDelay)
            .time_since_epoch()
            .count());
    vision_status_builder.add_distance(vision_distance_);
    ASSERT_EQ(turret_history_.capacity(), turret_history_.size());
    ASSERT_EQ(drivetrain_history_.capacity(), drivetrain_history_.size());
    vision_status_builder.add_angle(vision_angle_ - turret_history_[0] -
                                    drivetrain_history_[0]);
    vision_status_builder.add_image_valid(true);

    ASSERT_TRUE(builder.Send(vision_status_builder.Finish()));
  }

  double GetDriveTrainAngle() const {
    return -(drivetrain_left_ - drivetrain_right_) /
           (drivetrain::kRobotRadius * 2.0);
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  ::aos::SimulatedEventLoopFactory event_loop_factory_;
  ::std::unique_ptr<::aos::EventLoop> simulation_event_loop_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_sender_;
  ::aos::Sender<::y2017::vision::VisionStatus> vision_status_sender_;

  ::std::unique_ptr<::aos::EventLoop> vision_time_adjuster_event_loop_;
  ::aos::Fetcher<::y2017::vision::VisionStatus> vision_status_fetcher_;

  VisionTimeAdjuster adjuster_;

  ::aos::monotonic_clock::time_point current_time_ =
      ::aos::monotonic_clock::epoch();

  ::aos::RingBuffer<double, 3> turret_history_;
  ::aos::RingBuffer<double, 3> drivetrain_history_;

  // The turret angle with respect to the robot base.
  double turret_angle_ = 0.0;

  // Absolute angle of the robot with respect to the ground.
  double drivetrain_left_ = 0.0;
  double drivetrain_right_ = 0.0;

  // The absolute angle for the target (i.e. with respect to the ground),
  // irrespective of the turret and robot angle and such.
  double vision_distance_ = 0.0;
  double vision_angle_ = 0.0;
};

constexpr ::std::chrono::milliseconds VisionTimeAdjusterTest::kTimeTick;
constexpr ::std::chrono::milliseconds VisionTimeAdjusterTest::kVisionTick;
constexpr ::std::chrono::milliseconds VisionTimeAdjusterTest::kVisionDelay;

// Test that moving only the turret around results in the correct turret goal.
TEST_F(VisionTimeAdjusterTest, TurretRotationOnly) {
  SetDrivetrainPosition(0, 0);
  SetVisionAngle(M_PI / 5);
  for (size_t i = 0; i < 200; ++i) {
    SetTurretPosition(static_cast<int>(i) * 0.01);
    ASSERT_NO_FATAL_FAILURE(RunIteration());

    if (i < GetVisionDelayCount()) {
      EXPECT_FALSE(adjuster()->valid());
    } else {
      ASSERT_TRUE(adjuster()->valid());
      EXPECT_NEAR(M_PI / 5, adjuster()->goal(), 0.00001);
    }
  }
}

// Tests that moving only the drivetrain around results in the correct turret
// goal.
TEST_F(VisionTimeAdjusterTest, DrivetrainRotationOnly) {
  SetTurretPosition(0.0);
  SetVisionAngle(0.0);
  for (size_t i = 0; i < 200; ++i) {
    double angle = static_cast<double>(i) * 0.01;
    SetDrivetrainPosition(angle * drivetrain::kRobotRadius * 2.0,
                          -angle * drivetrain::kRobotRadius * 2.0);
    ASSERT_NO_FATAL_FAILURE(RunIteration());

    if (i < GetVisionDelayCount()) {
      EXPECT_FALSE(adjuster()->valid());
    } else {
      ASSERT_TRUE(adjuster()->valid());
      EXPECT_NEAR(2 * angle, adjuster()->goal(), 0.00001);
    }
  }
}

// Tests that moving both the drivetrain and the turret results in the correct
// turret angle.
TEST_F(VisionTimeAdjusterTest, DrivetrainAndTurretTogether) {
  SetVisionAngle(-M_PI / 6);

  for (size_t i = 0; i < 200; ++i) {
    double drivetrain_angle = static_cast<double>(i) * 0.01;
    double turret_angle = (100.0 - static_cast<double>(i)) * 0.005;
    SetDrivetrainPosition(-drivetrain_angle * drivetrain::kRobotRadius * 2.0,
                          drivetrain_angle * drivetrain::kRobotRadius * 2.0);
    SetTurretPosition(turret_angle);
    ASSERT_NO_FATAL_FAILURE(RunIteration());

    if (i < GetVisionDelayCount()) {
      EXPECT_FALSE(adjuster()->valid());
    } else {
      ASSERT_TRUE(adjuster()->valid());
      EXPECT_NEAR(-M_PI / 6 - 2 * drivetrain_angle, adjuster()->goal(),
                  0.00001);
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
