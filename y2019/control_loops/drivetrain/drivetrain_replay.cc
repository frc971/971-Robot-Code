#include <iostream>

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "gflags/gflags.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/event_loop_localizer.h"

DEFINE_string(logfile, "/tmp/logfile.bfbs",
              "Name of the logfile to read from.");
DEFINE_string(config, "y2019/config.json",
              "Name of the config file to replay using.");
DEFINE_string(output_file, "/tmp/replayed",
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

  std::unique_ptr<aos::EventLoop> log_writer_event_loop =
      reader.event_loop_factory()->MakeEventLoop("log_writer");
  log_writer_event_loop->SkipTimingReport();
  log_writer_event_loop->SkipAosLog();
  CHECK(nullptr == log_writer_event_loop->node());
  aos::logger::Logger writer(log_writer_event_loop.get());
  writer.StartLoggingLocalNamerOnRun(FLAGS_output_file);

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop =
      reader.event_loop_factory()->MakeEventLoop("drivetrain");
  drivetrain_event_loop->SkipTimingReport();

  y2019::control_loops::drivetrain::EventLoopLocalizer localizer(
      drivetrain_event_loop.get(),
      y2019::control_loops::drivetrain::GetDrivetrainConfig());
  frc971::control_loops::drivetrain::DrivetrainLoop drivetrain(
      y2019::control_loops::drivetrain::GetDrivetrainConfig(),
      drivetrain_event_loop.get(), &localizer);

  reader.event_loop_factory()->Run();

  return 0;
}
