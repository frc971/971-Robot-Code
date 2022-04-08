#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/input/joystick_state_generated.h"

// Takes in log file and gives back the Match Type and Match #
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();
  const aos::Node *roborio =
      aos::configuration::GetNode(reader.configuration(), "roborio");

  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("roborio", roborio);

  aos::MatchType match_type = aos::MatchType::kNone;
  int match_number = 0;

  auto joystick_state_fetcher =
      event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos");

  event_loop->AddPhasedLoop(
      [&](int) {
        // Fetch joystick state if null then don't give type and number
        if (!joystick_state_fetcher.Fetch()) {
          return;
        }
        match_type = joystick_state_fetcher->match_type();
        match_number = joystick_state_fetcher->match_number();
        // Exits if the match type isn't kNone if match type isn't kNone the
        // match has been found
        if (match_type != aos::MatchType::kNone) {
          reader.event_loop_factory()->Exit();
          return;
        }
      },
      std::chrono::seconds(1));

  reader.event_loop_factory()->Run();

  LOG(INFO) << "Match Type: " << aos::EnumNameMatchType(match_type);
  LOG(INFO) << "Match #: " << match_number;
}
