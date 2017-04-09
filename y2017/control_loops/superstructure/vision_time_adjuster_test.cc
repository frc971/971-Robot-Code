#include "y2017/control_loops/superstructure/vision_time_adjuster.h"

#include "gtest/gtest.h"

#include "aos/common/time.h"
#include "aos/testing/test_shm.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/vision/vision.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

class VisionTimeAdjusterTest : public ::testing::Test {
 public:
  VisionTimeAdjusterTest() : ::testing::Test() {
    static_assert(kVisionDelay % kTimeTick == ::std::chrono::milliseconds(0),
                  "kVisionDelay must be a multiple of kTimeTick.");
    static_assert(
        kVisionDelay / kTimeTick == decltype(turret_history_)::kBufferSize,
        "We need a different amount of space to hold the turret data.");
    ::aos::time::EnableMockTime(current_time_);
    ::frc971::control_loops::drivetrain_queue.status.Clear();
    vision::vision_status.Clear();
  }

  ~VisionTimeAdjusterTest() {
    ::frc971::control_loops::drivetrain_queue.status.Clear();
    vision::vision_status.Clear();
    ::aos::time::DisableMockTime();
  }

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

  void RunForTime(::aos::monotonic_clock::duration run_for) {
    const auto start_time = ::aos::monotonic_clock::now();
    while (::aos::monotonic_clock::now() < start_time + run_for) {
      RunIteration();
    }
  }

  void RunIteration() {
    SendMessages();
    const vision::VisionStatus *vision_status = nullptr;
    if (vision::vision_status.FetchLatest()) {
      vision_status = vision::vision_status.get();
    }
    adjuster_.Tick(::aos::monotonic_clock::now(), turret_angle_, vision_status);
    TickTime();
  }

  static constexpr size_t GetVisionDelayCount() {
    return kVisionDelay / kTimeTick;
  }

  VisionTimeAdjuster adjuster_;

 private:
  void TickTime() { ::aos::time::SetMockTime(current_time_ += kTimeTick); }

  void SendMessages() {
    SendDrivetrainPosition();
    if (::aos::monotonic_clock::now().time_since_epoch() % kVisionTick ==
        kVisionDelay) {
      SendVisionTarget();
    }
    turret_history_.Push(turret_angle_);
    drivetrain_history_.Push(GetDriveTrainAngle());
  }

  void SendDrivetrainPosition() {
    auto message =
        ::frc971::control_loops::drivetrain_queue.status.MakeMessage();
    message->estimated_left_position = drivetrain_left_;
    message->estimated_right_position = drivetrain_right_;
    ASSERT_TRUE(message.Send());
  }

  void SendVisionTarget() {
    auto message = vision::vision_status.MakeMessage();
    message->target_time = (::aos::monotonic_clock::now() - kVisionDelay)
                               .time_since_epoch()
                               .count();
    message->distance = vision_distance_;
    ASSERT_EQ(turret_history_.capacity(), turret_history_.size());
    ASSERT_EQ(drivetrain_history_.capacity(), drivetrain_history_.size());
    message->angle =
        vision_angle_ - turret_history_[0] - drivetrain_history_[0];
    message->image_valid = true;
    ASSERT_TRUE(message.Send());
  }

  double GetDriveTrainAngle() const {
    return -(drivetrain_left_ - drivetrain_right_) /
           (drivetrain::kRobotRadius * 2.0);
  }

  ::aos::testing::TestSharedMemory my_shm_;

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
      EXPECT_FALSE(adjuster_.valid());
    } else {
      ASSERT_TRUE(adjuster_.valid());
      EXPECT_NEAR(M_PI / 5, adjuster_.goal(), 0.00001);
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
      EXPECT_FALSE(adjuster_.valid());
    } else {
      ASSERT_TRUE(adjuster_.valid());
      EXPECT_NEAR(2 * angle, adjuster_.goal(), 0.00001);
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
      EXPECT_FALSE(adjuster_.valid());
    } else {
      ASSERT_TRUE(adjuster_.valid());
      EXPECT_NEAR(-M_PI / 6 - 2 * drivetrain_angle, adjuster_.goal(), 0.00001);
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
