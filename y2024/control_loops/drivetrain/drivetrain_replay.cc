#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/util/simulation_logger.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/localization/puppet_localizer.h"
#include "frc971/control_loops/drivetrain/trajectory_generator.h"
#include "frc971/imu_fdcan/dual_imu_blender_lib.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"

ABSL_FLAG(std::string, config, "y2024/aos_config.json",
          "Name of the config file to replay using.");
ABSL_FLAG(bool, override_config, false,
          "If set, override the logged config with --config.");
ABSL_FLAG(int32_t, team, 971, "Team number to use for logfile replay.");
ABSL_FLAG(std::string, output_folder, "/tmp/replayed",
          "Name of the folder to write replayed logs to.");
ABSL_FLAG(std::string, constants_path, "y2024/constants/constants.json",
          "Path to the constant file");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(absl::GetFlag(FLAGS_team));

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  // open logfiles
  aos::logger::LogReader reader(logfiles, absl::GetFlag(FLAGS_override_config)
                                              ? &config.message()
                                              : nullptr);

  reader.RemapLoggedChannel("/imu/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/roborio/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/orin1/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Status");
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Output");
  reader.RemapLoggedChannel(
      "/drivetrain", "frc971.control_loops.drivetrain.RioLocalizerInputs");

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  reader.RegisterWithoutStarting(factory.get());

  y2024::SendSimulationConstants(
      reader.event_loop_factory(), absl::GetFlag(FLAGS_team),
      absl::GetFlag(FLAGS_constants_path), {"roborio"});

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "roborio");
  }
  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop;
  std::unique_ptr<::frc971::control_loops::drivetrain::PuppetLocalizer>
      localizer;
  std::unique_ptr<frc971::control_loops::drivetrain::DrivetrainLoop> drivetrain;
  reader.OnStart(node, [&factory, node, &loggers, &drivetrain_event_loop,
                        &localizer, &drivetrain]() {
    aos::NodeEventLoopFactory *node_factory =
        factory->GetNodeEventLoopFactory(node);
    drivetrain_event_loop = node_factory->MakeEventLoop("drivetrain");
    const auto drivetrain_config =
        ::y2024::control_loops::drivetrain::GetDrivetrainConfig(
            drivetrain_event_loop.get());
    localizer =
        std::make_unique<::frc971::control_loops::drivetrain::PuppetLocalizer>(
            drivetrain_event_loop.get(), drivetrain_config);
    drivetrain =
        std::make_unique<frc971::control_loops::drivetrain::DrivetrainLoop>(
            drivetrain_config, drivetrain_event_loop.get(), localizer.get());
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, absl::GetFlag(FLAGS_output_folder)));
    // The Trajectory information is NOT_LOGGED, so we need to rerun it against
    // the SplineGoals that we have in the log.
    node_factory
        ->AlwaysStart<frc971::control_loops::drivetrain::TrajectoryGenerator>(
            "trajectory_generator", drivetrain_config);
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
