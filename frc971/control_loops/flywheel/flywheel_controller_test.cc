#include "frc971/control_loops/flywheel/flywheel_controller.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/flywheel/flywheel_controller_test_plant.h"
#include "frc971/control_loops/flywheel/flywheel_test_plant.h"
#include "frc971/control_loops/flywheel/integral_flywheel_controller_test_plant.h"

namespace frc971 {
namespace control_loops {
namespace flywheel {
namespace testing {
class FlywheelTest : public ::frc971::testing::ControlLoopTest {
 public:
  FlywheelTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "frc971/control_loops/flywheel/"
                "flywheel_controller_test_config.json"),
            std::chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        flywheel_plant_(
            new FlywheelPlant(MakeFlywheelTestPlant(), kBemf, kResistance)),
        flywheel_controller_(MakeIntegralFlywheelTestLoop(), kBemf,
                             kResistance),
        flywheel_controller_sender_(
            test_event_loop_->MakeSender<FlywheelControllerStatus>("/loop")) {
    phased_loop_handle_ =
        test_event_loop_->AddPhasedLoop([this](int) { Simulate(); }, dt());
  }

  void Simulate() {
    const aos::monotonic_clock::time_point timestamp =
        test_event_loop_->context().monotonic_event_time;
    ::Eigen::Matrix<double, 1, 1> flywheel_U;
    flywheel_U << flywheel_voltage_ + flywheel_plant_->voltage_offset();

    // Confirm that we aren't drawing too much current.  2 motors -> twice the
    // lumped current since our model can't tell them apart.
    CHECK_NEAR(flywheel_plant_->battery_current(flywheel_U), 0.0, 200.0);

    flywheel_plant_->Update(flywheel_U);

    flywheel_controller_.set_position(flywheel_plant_->Y(0, 0), timestamp);

    flywheel_controller_.set_goal(goal_);

    flywheel_controller_.Update(false);
    aos::FlatbufferFixedAllocatorArray<FlywheelControllerStatus, 512>
        flywheel_status_buffer;

    flywheel_status_buffer.Finish(
        flywheel_controller_.SetStatus(flywheel_status_buffer.fbb()));

    flywheel_voltage_ = flywheel_controller_.voltage();

    last_angular_velocity_ =
        flywheel_status_buffer.message().angular_velocity();
  }

  void VerifyNearGoal() { EXPECT_NEAR(last_angular_velocity_, goal_, 0.1); }

  void set_goal(double goal) { goal_ = goal; }

 private:
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  std::unique_ptr<FlywheelPlant> flywheel_plant_;
  FlywheelController flywheel_controller_;

  aos::Sender<FlywheelControllerStatus> flywheel_controller_sender_;

  double last_angular_velocity_ = 0.0;

  double flywheel_voltage_ = 0.0;
  double goal_ = 0.0;
};

TEST_F(FlywheelTest, DoNothing) {
  set_goal(0);
  RunFor(std::chrono::seconds(2));
  VerifyNearGoal();
}

TEST_F(FlywheelTest, PositiveTest) {
  set_goal(700.0);
  RunFor(std::chrono::seconds(4));
  VerifyNearGoal();
}

TEST_F(FlywheelTest, NegativeTest) {
  set_goal(-700.0);
  RunFor(std::chrono::seconds(8));
  VerifyNearGoal();
}
}  // namespace testing
}  // namespace flywheel
}  // namespace control_loops
}  // namespace frc971
