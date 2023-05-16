#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/util/simulation_logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "y2022/localizer/localizer.h"
#include "y2022/localizer/localizer_schema.h"
#include "gflags/gflags.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"

DEFINE_string(config, "y2022/aos_config.json",
              "Name of the config file to replay using.");
DEFINE_int32(team, 7971, "Team number to use for logfile replay.");
DEFINE_string(output_folder, "/tmp/replayed",
              "Name of the folder to write replayed logs to.");

// TODO(james): Currently, this replay produces logfiles that can't be read due
// to time estimation issues. Pending the active refactorings of the
// timestamp-related code, fix this.
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(FLAGS_team);

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  // find logfiles
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(unsorted_logfiles);

  // open logfiles
  aos::logger::LogReader reader(logfiles, &config.message());

  reader.RemapLoggedChannel("/localizer",
                            "frc971.controls.LocalizerStatus");
  reader.RemapLoggedChannel("/localizer",
                            "frc971.controls.LocalizerOutput");
  reader.RemapLoggedChannel("/localizer",
                            "frc971.controls.LocalizerVisualization");

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  reader.Register(factory.get());

  // List of nodes to create loggers for (note: currently just roborio; this
  // code was refactored to allow easily adding new loggers to accommodate
  // debugging and potential future changes).
  const std::vector<std::string> nodes_to_log = {"imu"};
  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers =
      aos::util::MakeLoggersForNodes(reader.event_loop_factory(), nodes_to_log,
                                     FLAGS_output_folder);

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "imu");
  }

  std::unique_ptr<aos::EventLoop> localizer_event_loop =
      reader.event_loop_factory()->MakeEventLoop("localizer", node);
  localizer_event_loop->SkipTimingReport();

  frc971::controls::EventLoopLocalizer localizer(
      localizer_event_loop.get(),
      y2022::control_loops::drivetrain::GetDrivetrainConfig());

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
