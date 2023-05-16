// This binary allows us to replay the drivetrain code over existing logfile,
// primarily for use in testing changes to the localizer code.
// When you run this code, it generates a new logfile with the data all
// replayed, so that it can then be run through the plotting tool or analyzed
// in some other way. The original drivetrain status data will be on the
// /original/drivetrain channel.
#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/util/simulation_logger.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/trajectory_generator.h"
#include "gflags/gflags.h"
#include "y2020/constants.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/drivetrain/localizer.h"
#include "y2020/control_loops/superstructure/superstructure.h"

DEFINE_string(config, "y2020/aos_config.json",
              "Name of the config file to replay using.");
DEFINE_string(output_folder, "/tmp/replayed",
              "Name of the folder to write replayed logs to.");
DEFINE_int32(team, 971, "Team number to use for logfile replay.");
DEFINE_bool(log_all_nodes, false, "Whether to rerun the logger on every node.");

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
  // TODO(james): Actually enforce not sending on the same buses as the logfile
  // spews out.
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Status");
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Output");
  reader.RemapLoggedChannel("/drivetrain",
                            "y2020.control_loops.drivetrain.LocalizerDebug");
  reader.RemapLoggedChannel("/superstructure",
                            "y2020.control_loops.superstructure.Status");
  reader.RemapLoggedChannel("/superstructure",
                            "y2020.control_loops.superstructure.Output");
  reader.Register();

  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;
  if (FLAGS_log_all_nodes) {
    loggers = aos::util::MakeLoggersForAllNodes(reader.event_loop_factory(),
                                                FLAGS_output_folder);
  } else {
    // List of nodes to create loggers for (note: currently just roborio; this
    // code was refactored to allow easily adding new loggers to accommodate
    // debugging and potential future changes).
    const std::vector<std::string> nodes_to_log = {"roborio"};
    loggers = aos::util::MakeLoggersForNodes(reader.event_loop_factory(),
                                             nodes_to_log, FLAGS_output_folder);
  }

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "roborio");
  }

  std::unique_ptr<aos::EventLoop> trajectory_generator_event_loop =
      reader.event_loop_factory()->MakeEventLoop("trajectory_generator", node);
  trajectory_generator_event_loop->SkipTimingReport();

  y2020::constants::InitValues();

  frc971::control_loops::drivetrain::TrajectoryGenerator trajectory_generator(
      trajectory_generator_event_loop.get(),
      y2020::control_loops::drivetrain::GetDrivetrainConfig());

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop =
      reader.event_loop_factory()->MakeEventLoop("drivetrain", node);
  drivetrain_event_loop->SkipTimingReport();

  y2020::control_loops::drivetrain::Localizer localizer(
      drivetrain_event_loop.get(),
      y2020::control_loops::drivetrain::GetDrivetrainConfig());
  frc971::control_loops::drivetrain::DrivetrainLoop drivetrain(
      y2020::control_loops::drivetrain::GetDrivetrainConfig(),
      drivetrain_event_loop.get(), &localizer);

  std::unique_ptr<aos::EventLoop> superstructure_event_loop =
      reader.event_loop_factory()->MakeEventLoop("superstructure", node);
  superstructure_event_loop->SkipTimingReport();

  y2020::control_loops::superstructure::Superstructure superstructure(
      superstructure_event_loop.get());

  reader.event_loop_factory()->Run();

  return 0;
}
