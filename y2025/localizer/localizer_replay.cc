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
#include "frc971/imu_fdcan/dual_imu_blender_lib.h"
#include "y2025/constants/simulated_constants_sender.h"
#include "y2025/localizer/localizer.h"

ABSL_FLAG(std::string, config, "y2025/aos_config.json",
          "Name of the config file to replay using.");
ABSL_FLAG(bool, override_config, false,
          "If set, override the logged config with --config.");
ABSL_FLAG(int32_t, team, 971, "Team number to use for logfile replay.");
ABSL_FLAG(std::string, output_folder, "/tmp/replayed",
          "Name of the folder to write replayed logs to.");
ABSL_FLAG(std::string, constants_path, "y2025/constants/constants.json",
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

  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/localizer" : "/imu/localizer",
      "frc971.control_loops.swerve.LocalizerState");
  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/localizer" : "/imu/localizer",
      "y2025.localizer.Status");
  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/constants" : "/imu/constants",
      "y2025.Constants");
  reader.RemapLoggedChannel("/roborio/constants", "y2025.Constants");
  reader.RemapLoggedChannel("/orin1/constants", "y2025.Constants");

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  reader.RegisterWithoutStarting(factory.get());

  y2025::SendSimulationConstants(reader.event_loop_factory(),
                                 absl::GetFlag(FLAGS_team),
                                 absl::GetFlag(FLAGS_constants_path));

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "imu");
  }
  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;

  reader.OnStart(node, [&factory, node, &loggers]() {
    aos::NodeEventLoopFactory *node_factory =
        factory->GetNodeEventLoopFactory(node);
    node_factory->AlwaysStart<y2025::localizer::WeightedAverageLocalizer>(
        "localizer");
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, absl::GetFlag(FLAGS_output_folder)));
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
