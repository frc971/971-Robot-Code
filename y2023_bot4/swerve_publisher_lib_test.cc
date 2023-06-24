#include "y2023_bot4/swerve_publisher_lib.h"

#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"

namespace y2023_bot4 {
namespace testing {
class SwervePublisherTest : public ::testing::Test {
 public:
  SwervePublisherTest()
      : config_(aos::configuration::ReadConfig("y2023_bot4/aos_config.json")),
        event_loop_factory_(&config_.message()),
        roborio_(aos::configuration::GetNode(
            event_loop_factory_.configuration(), "roborio")),
        event_loop_(
            event_loop_factory_.MakeEventLoop("swerve_publisher", roborio_)),
        exit_handle_(event_loop_factory_.MakeExitHandle()),
        drivetrain_swerve_output_fetcher_(
            event_loop_->MakeFetcher<
                frc971::control_loops::drivetrain::swerve::Output>(
                "/drivetrain")),
        swerve_publisher_(event_loop_.get(), exit_handle_.get(),
                          "y2023_bot4/swerve_drivetrain_output.json", 100) {}

  void SendOutput() { event_loop_factory_.Run(); }

  void CheckOutput() {
    drivetrain_swerve_output_fetcher_.Fetch();

    ASSERT_TRUE(drivetrain_swerve_output_fetcher_.get() != nullptr)
        << ": No drivetrain output";
  }

 private:
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const roborio_;

  std::unique_ptr<aos::EventLoop> event_loop_;
  std::unique_ptr<aos::ExitHandle> exit_handle_;

  aos::Fetcher<frc971::control_loops::drivetrain::swerve::Output>
      drivetrain_swerve_output_fetcher_;

  y2023_bot4::SwervePublisher swerve_publisher_;
};

TEST_F(SwervePublisherTest, CheckSentFb) {
  SendOutput();
  CheckOutput();
}
}  // namespace testing
}  // namespace y2023_bot4
