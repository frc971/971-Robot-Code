#include <iostream>

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_string(logfile, "/tmp/logfile.bfbs",
              "Name of the logfile to read from.");
DEFINE_string(
    name, "",
    "Name to match for printing out channels. Empty means no name filter.");
DEFINE_string(type, "",
              "Channel type to match for printing out channels. Empty means no "
              "type filter.");
int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "This program provides a basic interface to dump data from a logfile to "
      "stdout. Given a logfile, channel name filter, and type filter, it will "
      "print all the messages in the logfile matching the filters. The message "
      "filters work by taking the values of --name and --type and printing any "
      "channel whose name contains --name as a substr and whose type contains "
      "--type as a substr. Not specifying --name or --type leaves them free. "
      "Calling this program without --name or --type specified prints out all "
      "the logged data.");
  aos::InitGoogle(&argc, &argv);

  aos::logger::LogReader reader(FLAGS_logfile);
  aos::SimulatedEventLoopFactory log_reader_factory(reader.configuration(),
                                                    reader.node());
  reader.Register(&log_reader_factory);

  std::unique_ptr<aos::EventLoop> printer_event_loop =
      log_reader_factory.MakeEventLoop("printer");
  printer_event_loop->SkipTimingReport();

  bool found_channel = false;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      reader.configuration()->channels();
  for (flatbuffers::uoffset_t i = 0; i < channels->size(); i++) {
    const aos::Channel *channel = channels->Get(i);
    const flatbuffers::string_view name = channel->name()->string_view();
    const flatbuffers::string_view type = channel->type()->string_view();
    if (name.find(FLAGS_name) != std::string::npos &&
        type.find(FLAGS_type) != std::string::npos) {
      if (!aos::configuration::ChannelIsReadableOnNode(
              channel, printer_event_loop->node())) {
        continue;
      }
      LOG(INFO) << "Listening on " << name << " " << type;

      CHECK_NOTNULL(channel->schema());
      printer_event_loop->MakeRawWatcher(
          channel, [channel](const aos::Context &context, const void *message) {
            // Print the flatbuffer out to stdout, both to remove the
            // unnecessary cruft from glog and to allow the user to readily
            // redirect just the logged output independent of any debugging
            // information on stderr.
            if (context.monotonic_remote_time != context.monotonic_event_time) {
              std::cout << context.realtime_remote_time << " ("
                        << context.monotonic_remote_time << ") delivered "
                        << context.realtime_event_time << " ("
                        << context.monotonic_event_time << ") "
                        << channel->name()->c_str() << ' '
                        << channel->type()->c_str() << ": "
                        << aos::FlatbufferToJson(
                               channel->schema(),
                               static_cast<const uint8_t *>(message))
                        << '\n';
            } else {
              std::cout << context.realtime_event_time << " ("
                        << context.monotonic_event_time << ") "
                        << channel->name()->c_str() << ' '
                        << channel->type()->c_str() << ": "
                        << aos::FlatbufferToJson(
                               channel->schema(),
                               static_cast<const uint8_t *>(message))
                        << '\n';
            }
          });
      found_channel = true;
    }
  }

  if (!found_channel) {
    LOG(FATAL) << "Could not find any channels";
  }

  log_reader_factory.Run();

  reader.Deregister();

  aos::Cleanup();
  return 0;
}
