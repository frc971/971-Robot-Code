// This binary allows us to replay the superstructure code over existing
// logfile. When you run this code, it generates a new logfile with the data all
// replayed, so that it can then be run through the plotting tool or analyzed
// in some other way. The original superstructure status data will be on the
// /original/superstructure channel.
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/log_message_generated.h"
#include "aos/network/team_number.h"
#include "gflags/gflags.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/superstructure.h"

DEFINE_int32(team, 971, "Team number to use for logfile replay.");
DEFINE_string(output_folder, "/tmp/superstructure_replay/",
              "Logs all channels to the provided logfile.");
DEFINE_string(arm_trajectories, "arm/arm_trajectories_generated.bfbs",
              "The path to the generated arm trajectories bfbs file.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(FLAGS_team);

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  // TODO(james): Actually enforce not sending on the same buses as the logfile
  // spews out.
  reader.RemapLoggedChannel("/superstructure",
                            "y2023.control_loops.superstructure.Status");
  reader.RemapLoggedChannel("/superstructure",
                            "y2023.control_loops.superstructure.Output");

  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.Register(&factory);

  aos::NodeEventLoopFactory *roborio =
      factory.GetNodeEventLoopFactory("roborio");

  unlink(FLAGS_output_folder.c_str());
  std::unique_ptr<aos::EventLoop> logger_event_loop =
      roborio->MakeEventLoop("logger");
  auto logger = std::make_unique<aos::logger::Logger>(logger_event_loop.get());
  logger->StartLoggingOnRun(FLAGS_output_folder);

  auto trajectories =
      y2023::control_loops::superstructure::Superstructure::GetArmTrajectories(
          FLAGS_arm_trajectories);

  roborio->OnStartup([roborio, &trajectories]() {
    roborio->AlwaysStart<y2023::control_loops::superstructure::Superstructure>(
        "superstructure",
        std::make_shared<y2023::constants::Values>(
            y2023::constants::MakeValues()),
        trajectories);
  });

  std::unique_ptr<aos::EventLoop> print_loop = roborio->MakeEventLoop("print");
  print_loop->SkipAosLog();
  print_loop->MakeWatcher(
      "/aos", [&print_loop](const aos::logging::LogMessageFbs &msg) {
        LOG(INFO) << print_loop->context().monotonic_event_time << " "
                  << aos::FlatbufferToJson(&msg);
      });
  print_loop->MakeWatcher(
      "/superstructure",
      [&](const y2023::control_loops::superstructure::Status &status) {
        if (status.estopped()) {
          LOG(ERROR) << "Estopped";
        }
      });

  factory.Run();

  reader.Deregister();

  return 0;
}
