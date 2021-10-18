// This binary allows us to replay the drivetrain code over existing logfile,
// primarily for use in testing changes to the localizer code.
// When you run this code, it generates a new logfile with the data all
// replayed, so that it can then be run through the plotting tool or analyzed
// in some other way. The original drivetrain status data will be on the
// /original/drivetrain channel.
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/log_message_generated.h"
#include "aos/network/team_number.h"
#include "gflags/gflags.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/superstructure.h"

DEFINE_int32(team, 971, "Team number to use for logfile replay.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(FLAGS_team);
  y2020::constants::InitValues();

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  // TODO(james): Actually enforce not sending on the same buses as the logfile
  // spews out.
  reader.RemapLoggedChannel("/superstructure",
                            "y2020.control_loops.superstructure.Status");
  reader.RemapLoggedChannel("/superstructure",
                            "y2020.control_loops.superstructure.Output");

  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.Register(&factory);

  aos::NodeEventLoopFactory *roborio =
      factory.GetNodeEventLoopFactory("roborio");

  roborio->OnStartup([roborio]() {
    roborio->AlwaysStart<y2020::control_loops::superstructure::Superstructure>(
        "superstructure");
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
      [&print_loop](
          const y2020::control_loops::superstructure::Position &position) {
        LOG(INFO) << print_loop->context().monotonic_event_time << " "
                  << aos::FlatbufferToJson(position.hood());
      });
  print_loop->MakeWatcher(
      "/superstructure",
      [&print_loop](
          const y2020::control_loops::superstructure::Status &status) {
        if (status.estopped()) {
          LOG(INFO) << "Estopped";
        }
        LOG(INFO) << print_loop->context().monotonic_event_time << " "
                  << aos::FlatbufferToJson(status.hood());
      });

  factory.Run();

  reader.Deregister();

  return 0;
}
