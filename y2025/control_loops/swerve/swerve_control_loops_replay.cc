// This binary allows us to replay the swerve constrol loop code over existing
// logfile. When you run this code, it generates a new logfile with the data all
// replayed, so that it can then be run through the plotting tool or analyzed
// in some other way. The original swerve status data will be on the
// /original/swerve channel.

#include <iostream>

#include "absl/flags/flag.h"
#include "absl/log/check.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/log_message_generated.h"
#include "aos/network/team_number.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/swerve/swerve_control_loops.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/constants/simulated_constants_sender.h"
#include "y2025/control_loops/swerve/parameters.h"

using y2025::control_loops::CreateWeights;

ABSL_FLAG(int32_t, team, 971, "Team number to use for logfile replay.");
ABSL_FLAG(std::string, config, "y2025/aos_config.json",
          "Name of the config file to replay using.");
ABSL_FLAG(std::string, constants_path, "y2025/constants/constants.json",
          "Path to the constant file");
ABSL_FLAG(std::string, output_folder, "",
          "Name of the folder to write replayed logs to.");
ABSL_FLAG(bool, use_orin1, true,
          "Use only the orin1 node instead of the imu node.");

std::string generate_filepath(char **argv) {
  std::string full_filename = argv[1];

  if (!full_filename.empty() && full_filename.back() == '/') {
    full_filename.pop_back();
  }

  size_t last_slash_pos = full_filename.rfind('/');
  std::string filename = (last_slash_pos == std::string::npos)
                             ? full_filename
                             : full_filename.substr(last_slash_pos + 1);

  return "/tmp/" + filename + "_replayed/";
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  std::string output_folder = (!absl::GetFlag(FLAGS_output_folder).empty())
                                  ? absl::GetFlag(FLAGS_output_folder)
                                  : generate_filepath(argv);

  LOG(INFO) << "Saving replay to " << output_folder;

  if (std::filesystem::is_directory(output_folder)) {
    LOG(INFO) << "The directory " << output_folder
              << " already exists. Deleting...";

    std::filesystem::remove_all(output_folder);
  }

  aos::network::OverrideTeamNumber(absl::GetFlag(FLAGS_team));

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  CHECK(!logfiles.empty());

  // open logfiles
  aos::logger::LogReader reader(logfiles, &config.message());

  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/constants" : "/imu/constants",
      "y2025.Constants");
  reader.RemapLoggedChannel("/roborio/constants", "y2025.Constants");
  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/drivetrain" : "/imu/drivetrain",
      "frc971.control_loops.swerve.Output");
  reader.RemapLoggedChannel(
      absl::GetFlag(FLAGS_use_orin1) ? "/orin1/drivetrain" : "/imu/drivetrain",
      "frc971.control_loops.swerve.Status");
  reader.RemapLoggedChannel(absl::GetFlag(FLAGS_use_orin1)
                                ? "/orin1/autonomous_auto_align"
                                : "/imu/autonomous_auto_align",
                            "frc971.control_loops.swerve.Goal");

  aos::SimulatedEventLoopFactory factory(reader.configuration());

  reader.RegisterWithoutStarting(&factory);

  aos::NodeEventLoopFactory *imu = factory.GetNodeEventLoopFactory("imu");

  imu->OnStartup([imu, &reader, &output_folder]() {
    y2025::SendSimulationConstants(
        reader.event_loop_factory(), absl::GetFlag(FLAGS_team),
        absl::GetFlag(FLAGS_constants_path), {"imu"});
    auto logger = imu->AlwaysStart<aos::logger::Logger>("logger");
    logger->StartLoggingOnRun(output_folder);

    auto constants =
        imu->AlwaysStart<frc971::constants::ConstantsFetcher<y2025::Constants>>(
            "constants_fetcher");
    imu->AlwaysStart<frc971::control_loops::swerve::SwerveControlLoops>(
        "drivetrain", constants->constants().common()->rotation(),
        constants->constants().robot()->swerve_zeroing(),
        y2025::control_loops::MakeSwerveParameters<float>(),
        CreateWeights(constants->constants().common()->weights()),
        "/drivetrain");
  });

  factory.Run();

  reader.Deregister();

  return 0;
}
