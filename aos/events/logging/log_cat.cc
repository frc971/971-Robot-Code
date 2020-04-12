#include <iostream>

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_string(
    name, "",
    "Name to match for printing out channels. Empty means no name filter.");
DEFINE_string(type, "",
              "Channel type to match for printing out channels. Empty means no "
              "type filter.");
DEFINE_bool(raw, false,
            "If true, just print the data out unsorted and unparsed");
DEFINE_int32(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");

int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "Usage:\n"
      "  log_cat [args] logfile1 logfile2 ...\n"
      "\n"
      "This program provides a basic interface to dump data from a logfile to "
      "stdout. Given a logfile, channel name filter, and type filter, it will "
      "print all the messages in the logfile matching the filters. The message "
      "filters work by taking the values of --name and --type and printing any "
      "channel whose name contains --name as a substr and whose type contains "
      "--type as a substr. Not specifying --name or --type leaves them free. "
      "Calling this program without --name or --type specified prints out all "
      "the logged data.");
  aos::InitGoogle(&argc, &argv);

  if (FLAGS_raw) {
    if (argc != 2) {
      LOG(FATAL) << "Expected 1 logfile as an argument.";
    }
    aos::logger::MessageReader reader(argv[1]);

    while (true) {
      std::optional<aos::FlatbufferVector<aos::logger::MessageHeader>> message =
          reader.ReadMessage();
      if (!message) {
        break;
      }

      std::cout << aos::FlatbufferToJson(message.value(), FLAGS_max_vector_size)
                << std::endl;
    }
    return 0;
  }

  if (argc < 2) {
    LOG(FATAL) << "Expected at least 1 logfile as an argument.";
  }

  std::vector<std::vector<std::string>> logfiles;

  for (int i = 1; i < argc; ++i) {
    logfiles.emplace_back(std::vector<std::string>{std::string(argv[i])});
  }

  aos::logger::LogReader reader(logfiles);
  reader.Register();

  std::vector<std::unique_ptr<aos::EventLoop>> printer_event_loops;

  for (const aos::Node *node : reader.Nodes()) {
    std::unique_ptr<aos::EventLoop> printer_event_loop =
        reader.event_loop_factory()->MakeEventLoop("printer", node);
    printer_event_loop->SkipTimingReport();
    printer_event_loop->SkipAosLog();

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
        VLOG(1) << "Listening on " << name << " " << type;

        std::string node_name =
            node == nullptr ? ""
                            : std::string(node->name()->string_view()) + " ";

        CHECK_NOTNULL(channel->schema());
        printer_event_loop->MakeRawWatcher(
            channel, [channel, node_name](const aos::Context &context,
                                          const void *message) {
              // Print the flatbuffer out to stdout, both to remove the
              // unnecessary cruft from glog and to allow the user to readily
              // redirect just the logged output independent of any debugging
              // information on stderr.
              if (context.monotonic_remote_time !=
                  context.monotonic_event_time) {
                std::cout << node_name << context.realtime_event_time << " ("
                          << context.monotonic_event_time << ") sent "
                          << context.realtime_remote_time << " ("
                          << context.monotonic_remote_time << ") "
                          << channel->name()->c_str() << ' '
                          << channel->type()->c_str() << ": "
                          << aos::FlatbufferToJson(
                                 channel->schema(),
                                 static_cast<const uint8_t *>(message),
                                 FLAGS_max_vector_size)
                          << std::endl;
              } else {
                std::cout << node_name << context.realtime_event_time << " ("
                          << context.monotonic_event_time << ") "
                          << channel->name()->c_str() << ' '
                          << channel->type()->c_str() << ": "
                          << aos::FlatbufferToJson(
                                 channel->schema(),
                                 static_cast<const uint8_t *>(message),
                                 FLAGS_max_vector_size)
                          << std::endl;
              }
            });
        found_channel = true;
      }
    }

    if (!found_channel) {
      LOG(FATAL) << "Could not find any channels";
    }
    printer_event_loops.emplace_back(std::move(printer_event_loop));
  }

  reader.event_loop_factory()->Run();

  aos::Cleanup();
  return 0;
}
