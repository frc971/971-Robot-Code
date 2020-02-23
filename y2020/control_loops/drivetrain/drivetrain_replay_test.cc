// This file serves to test replaying the localizer over existing logfiles to
// check for regressions. Currently, it just uses a single logfile pulled from
// running the 2016 robot against a wall and confirming that the X/Y estimate
// does not change too much.
//
// Note that the current logfile test will break once we update the drivetrain
// config for 2020, since both the gear ratios and IMU transformation wil no
// longer be valid.
// TODO(james): Do something about that when the time comes--could just copy
// the existing drivetrain config into this file and use it directly.
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "gflags/gflags.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

DEFINE_string(
    logfile, "external/drivetrain_replay/file/spinning_wheels_while_still.bfbs",
    "Name of the logfile to read from.");
DEFINE_string(config, "y2020/config.json",
              "Name of the config file to replay using.");

namespace y2020 {
namespace control_loops {
namespace drivetrain {
namespace testing {

class DrivetrainReplayTest : public ::testing::Test {
 public:
  DrivetrainReplayTest()
      : config_(aos::configuration::ReadConfig(FLAGS_config)),
        reader_(FLAGS_logfile, &config_.message()) {
    aos::network::OverrideTeamNumber(971);

    // TODO(james): Actually enforce not sending on the same buses as the
    // logfile spews out.
    reader_.RemapLoggedChannel("/drivetrain",
                              "frc971.control_loops.drivetrain.Status");
    reader_.RemapLoggedChannel("/drivetrain",
                              "frc971.control_loops.drivetrain.Output");
    reader_.Register();

    roborio_ = aos::configuration::GetNode(reader_.configuration(), "roborio");

    drivetrain_event_loop_ =
        reader_.event_loop_factory()->MakeEventLoop("drivetrain", roborio_);
    drivetrain_event_loop_->SkipTimingReport();

    localizer_ =
        std::make_unique<frc971::control_loops::drivetrain::DeadReckonEkf>(
            drivetrain_event_loop_.get(), GetDrivetrainConfig());
    drivetrain_ =
        std::make_unique<frc971::control_loops::drivetrain::DrivetrainLoop>(
            GetDrivetrainConfig(), drivetrain_event_loop_.get(),
            localizer_.get());

    test_event_loop_ =
        reader_.event_loop_factory()->MakeEventLoop("drivetrain", roborio_);
    status_fetcher_ = test_event_loop_->MakeFetcher<
        frc971::control_loops::drivetrain::Status>("/drivetrain");
  }

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::logger::LogReader reader_;
  const aos::Node *roborio_;

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop_;
  std::unique_ptr<frc971::control_loops::drivetrain::DeadReckonEkf> localizer_;
  std::unique_ptr<frc971::control_loops::drivetrain::DrivetrainLoop>
      drivetrain_;
  std::unique_ptr<aos::EventLoop> test_event_loop_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status> status_fetcher_;
};

// Tests that we do a good job of trusting the IMU when the wheels are spinning
// and the actual robot is not moving.
TEST_F(DrivetrainReplayTest, SpinningWheels) {
  reader_.event_loop_factory()->Run();

  ASSERT_TRUE(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->has_x());
  ASSERT_TRUE(status_fetcher_->has_y());
  ASSERT_TRUE(status_fetcher_->has_theta());
  EXPECT_LT(std::abs(status_fetcher_->x()), 0.1);
  // Because the encoders should not be affecting the y or yaw axes, expect a
  // reasonably precise result (although, since this is a real worl dtest, the
  // robot probably did actually move be some non-zero amount).
  EXPECT_LT(std::abs(status_fetcher_->y()), 0.05);
  EXPECT_LT(std::abs(status_fetcher_->theta()), 0.02);
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020
