#include <chrono>
#include <memory>

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2022/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace testing {

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2022/config.json"),
            std::chrono::microseconds(5050)),
        superstructure_event_loop(MakeEventLoop("Superstructure")),
        superstructure_(superstructure_event_loop.get()),
        test_event_loop_(MakeEventLoop("test")),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/superstructure")),
        superstructure_position_fetcher_(
            test_event_loop_->MakeFetcher<Position>("/superstructure")),
        superstructure_position_sender_(
            test_event_loop_->MakeSender<Position>("/superstructure")) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);
    SetEnabled(true);

    phased_loop_handle_ = test_event_loop_->AddPhasedLoop(
        [this](int) { SendPositionMessage(); }, dt());

    if (!FLAGS_output_folder.empty()) {
      unlink(FLAGS_output_folder.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingLocalNamerOnRun(FLAGS_output_folder);
    }
  }

  void SendPositionMessage() {
    auto builder = superstructure_position_sender_.MakeBuilder();
    Position::Builder position_builder = builder.MakeBuilder<Position>();
    builder.CheckOk(builder.Send(position_builder.Finish()));
  }

  // Because the third robot is single node, the roborio node is nullptr
  const aos::Node *const roborio_ = nullptr;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2022::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<Position> superstructure_position_sender_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
