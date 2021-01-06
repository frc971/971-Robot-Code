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

DEFINE_string(config, "y2020/config.json",
              "Name of the config file to replay using.");
DEFINE_string(output_file, "/tmp/replayed",
              "Name of the folder to write replayed logs to.");
DEFINE_int32(team, 971, "Team number to use for logfile replay.");

class LoggerState {
 public:
  LoggerState(aos::logger::LogReader *reader, const aos::Node *node)
      : event_loop_(
            reader->event_loop_factory()->MakeEventLoop("logger", node)),
        namer_(std::make_unique<aos::logger::MultiNodeLogNamer>(
            absl::StrCat(FLAGS_output_file, "/", node->name()->string_view(),
                         "/"),
            event_loop_->configuration(), node)),
        logger_(std::make_unique<aos::logger::Logger>(event_loop_.get())) {
    event_loop_->SkipTimingReport();
    event_loop_->OnRun([this]() {
      logger_->StartLogging(std::move(namer_), aos::UUID::Zero().string_view());
    });
  }

 private:
  std::unique_ptr<aos::EventLoop> event_loop_;
  std::unique_ptr<aos::logger::LogNamer> namer_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

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
  reader.Register();

  // List of nodes to create loggers for (note: currently just roborio; this
  // code was refactored to allow easily adding new loggers to accommodate
  // debugging and potential future changes).
  const std::vector<std::string> nodes_to_log = {"roborio"};
  std::vector<std::unique_ptr<LoggerState>> loggers;
  for (const std::string& node : nodes_to_log) {
    loggers.emplace_back(std::make_unique<LoggerState>(
        &reader,
        aos::configuration::GetNode(reader.configuration(), node)));
  }

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "roborio");
  }

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
