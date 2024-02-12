#include "gflags/gflags.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/util/simulation_logger.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"
#include "y2024/localizer/localizer.h"

DEFINE_string(config, "y2024/aos_config.json",
              "Name of the config file to replay using.");
DEFINE_int32(team, 9971, "Team number to use for logfile replay.");
DEFINE_string(output_folder, "/tmp/replayed",
              "Name of the folder to write replayed logs to.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(FLAGS_team);

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  // open logfiles
  aos::logger::LogReader reader(logfiles, &config.message());

  reader.RemapLoggedChannel("/localizer", "y2024.localizer.Status");
  for (const auto orin : {"orin1", "orin2"}) {
    for (const auto camera : {"camera0", "camera1"}) {
      reader.RemapLoggedChannel(absl::StrCat("/", orin, "/", camera),
                                "y2024.localizer.Visualization");
    }
  }
  reader.RemapLoggedChannel("/localizer", "frc971.controls.LocalizerOutput");

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  reader.RegisterWithoutStarting(factory.get());

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "imu");
  }
  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;

  reader.OnStart(node, [&factory, node, &loggers]() {
    aos::NodeEventLoopFactory *node_factory =
        factory->GetNodeEventLoopFactory(node);
    node_factory->AlwaysStart<y2024::localizer::Localizer>("localizer");
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, FLAGS_output_folder));
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
