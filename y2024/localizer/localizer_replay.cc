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
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"
#include "y2024/localizer/localizer.h"

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

  reader.RemapLoggedChannel("/localizer", "y2024.localizer.Status");
  for (const auto orin : {"orin1", "imu"}) {
    for (const auto camera : {"camera0", "camera1"}) {
      reader.RemapLoggedChannel(absl::StrCat("/", orin, "/", camera),
                                "y2024.localizer.Visualization");
    }
  }
  reader.RemapLoggedChannel("/localizer", "frc971.controls.LocalizerOutput");
  reader.RemapLoggedChannel("/localizer", "frc971.IMUValuesBatch");
  reader.RemapLoggedChannel("/imu", "frc971.imu.DualImuBlenderStatus");
  reader.RemapLoggedChannel("/imu/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/roborio/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/orin1/constants", "y2024.Constants");

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  reader.RegisterWithoutStarting(factory.get());

  y2024::SendSimulationConstants(reader.event_loop_factory(),
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
    node_factory->AlwaysStart<y2024::localizer::Localizer>("localizer");
    node_factory->AlwaysStart<frc971::imu_fdcan::DualImuBlender>(
        "dual_imu_blender");
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, absl::GetFlag(FLAGS_output_folder)));
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
