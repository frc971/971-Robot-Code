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
DEFINE_bool(fetch, false,
            "If true, also print out the messages from before the start of the "
            "log file");
DEFINE_bool(raw, false,
            "If true, just print the data out unsorted and unparsed");
DEFINE_bool(format_raw, true,
            "If true and --raw is specified, print out raw data, but use the "
            "schema to format the data.");
DEFINE_int32(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");

void LogContext(const aos::Channel *channel, std::string node_name,
                const aos::Context &context) {
  // Print the flatbuffer out to stdout, both to remove the
  // unnecessary cruft from glog and to allow the user to readily
  // redirect just the logged output independent of any debugging
  // information on stderr.
  if (context.monotonic_remote_time != context.monotonic_event_time) {
    std::cout << node_name << context.realtime_event_time << " ("
              << context.monotonic_event_time << ") sent "
              << context.realtime_remote_time << " ("
              << context.monotonic_remote_time << ") "
              << channel->name()->c_str() << ' ' << channel->type()->c_str()
              << ": "
              << aos::FlatbufferToJson(
                     channel->schema(),
                     static_cast<const uint8_t *>(context.data),
                     FLAGS_max_vector_size)
              << std::endl;
  } else {
    std::cout << node_name << context.realtime_event_time << " ("
              << context.monotonic_event_time << ") "
              << channel->name()->c_str() << ' ' << channel->type()->c_str()
              << ": "
              << aos::FlatbufferToJson(
                     channel->schema(),
                     static_cast<const uint8_t *>(context.data),
                     FLAGS_max_vector_size)
              << std::endl;
  }
}

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
      const aos::Channel *channel =
          reader.log_file_header()->configuration()->channels()->Get(
              message.value().message().channel_index());

      if (FLAGS_format_raw && message.value().message().data() != nullptr) {
        std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                  << aos::FlatbufferToJson(message.value(), false, 4) << ": "
                  << aos::FlatbufferToJson(
                         channel->schema(),
                         message.value().message().data()->data(),
                         FLAGS_max_vector_size)
                  << std::endl;
      } else {
        std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                  << aos::FlatbufferToJson(message.value(), false,
                                           FLAGS_max_vector_size)
                  << std::endl;
      }
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

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());
  reader.Register(&event_loop_factory);

  std::vector<std::unique_ptr<aos::EventLoop>> printer_event_loops;

  for (const aos::Node *node : reader.Nodes()) {
    std::unique_ptr<aos::EventLoop> printer_event_loop =
        event_loop_factory.MakeEventLoop("printer", node);
    printer_event_loop->SkipTimingReport();
    printer_event_loop->SkipAosLog();

    std::vector<std::tuple<aos::monotonic_clock::time_point, std::string,
                           std::unique_ptr<aos::RawFetcher>>>
        messages;

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

        if (FLAGS_fetch) {
          // Grab the last message on each channel.
          std::unique_ptr<aos::RawFetcher> fetcher =
              printer_event_loop->MakeRawFetcher(channel);
          if (fetcher->Fetch()) {
            auto message =
                std::make_tuple(fetcher->context().monotonic_event_time,
                                node_name, std::move(fetcher));

            // Insert it sorted into the vector so we can print in time order
            // instead of channel order at the start.
            auto it = std::lower_bound(
                messages.begin(), messages.end(), message,
                [](const std::tuple<aos::monotonic_clock::time_point,
                                    std::string,
                                    std::unique_ptr<aos::RawFetcher>> &a,
                   const std::tuple<aos::monotonic_clock::time_point,
                                    std::string,
                                    std::unique_ptr<aos::RawFetcher>> &b) {
                  if (std::get<0>(a) < std::get<0>(b)) {
                    return true;
                  }
                  if (std::get<0>(a) > std::get<0>(b)) {
                    return false;
                  }

                  return std::get<2>(a)->channel() < std::get<2>(b)->channel();
                });
            messages.insert(it, std::move(message));
          }
        }

        printer_event_loop->MakeRawWatcher(
            channel, [channel, node_name](const aos::Context &context,
                                          const void * /*message*/) {
              LogContext(channel, node_name, context);
            });
        found_channel = true;
      }
    }

    if (!found_channel) {
      LOG(FATAL) << "Could not find any channels";
    }
    // TODO(austin): Sort between nodes too when it becomes annoying enough.
    for (const std::tuple<aos::monotonic_clock::time_point, std::string,
                          std::unique_ptr<aos::RawFetcher>> &message :
         messages) {
      LogContext(std::get<2>(message)->channel(), std::get<1>(message),
                 std::get<2>(message)->context());
    }
    printer_event_loops.emplace_back(std::move(printer_event_loop));
  }

  if (FLAGS_fetch) {
    // New line to separate fetched messages from non-fetched messages.
    std::cout << std::endl;
  }

  event_loop_factory.Run();

  aos::Cleanup();
  return 0;
}
