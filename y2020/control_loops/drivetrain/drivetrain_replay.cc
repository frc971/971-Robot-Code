// This binary allows us to replay the drivetrain code over existing logfile,
// primarily for use in testing changes to the localizer code.
// When you run this code, it generates a new logfile with the data all
// replayed, so that it can then be run through the plotting tool or analyzed
// in some other way. The original drivetrain status data will be on the
// /original/drivetrain channel.
#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "gflags/gflags.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/drivetrain/localizer.h"

DEFINE_string(logfile, "/tmp/logfile.bfbs",
              "Name of the logfile to read from.");
DEFINE_string(config, "y2020/config.json",
              "Name of the config file to replay using.");
DEFINE_string(output_file, "/tmp/replayed.bfbs",
              "Name of the logfile to write replayed data to.");
DEFINE_int32(team, 971, "Team number to use for logfile replay.");
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(FLAGS_team);

  aos::InitCreate();

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::logger::LogReader reader(FLAGS_logfile, &config.message());
  // TODO(james): Actually enforce not sending on the same buses as the logfile
  // spews out.
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Status");
  reader.RemapLoggedChannel("/drivetrain",
                            "frc971.control_loops.drivetrain.Output");
  reader.Register();

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "roborio");
  }

  aos::logger::DetachedBufferWriter file_writer(FLAGS_output_file);
  std::unique_ptr<aos::EventLoop> log_writer_event_loop =
      reader.event_loop_factory()->MakeEventLoop("log_writer", node);
  log_writer_event_loop->SkipTimingReport();
  aos::logger::Logger writer(&file_writer, log_writer_event_loop.get());

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop =
      reader.event_loop_factory()->MakeEventLoop("drivetrain", node);
  drivetrain_event_loop->SkipTimingReport();

  y2020::control_loops::drivetrain::Localizer localizer(
      drivetrain_event_loop.get(),
      y2020::control_loops::drivetrain::GetDrivetrainConfig());
  frc971::control_loops::drivetrain::DrivetrainLoop drivetrain(
      y2020::control_loops::drivetrain::GetDrivetrainConfig(),
      drivetrain_event_loop.get(), &localizer);

  reader.event_loop_factory()->Run();

  return 0;
}
