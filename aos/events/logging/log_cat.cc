#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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

// Print the flatbuffer out to stdout, both to remove the unnecessary cruft from
// glog and to allow the user to readily redirect just the logged output
// independent of any debugging information on stderr.
void PrintMessage(const std::string_view node_name, const aos::Channel *channel,
                  const aos::Context &context,
                  aos::FastStringBuilder *builder) {
  builder->Reset();
  aos::FlatbufferToJson(builder, channel->schema(),
                        static_cast<const uint8_t *>(context.data),
                        {false, static_cast<size_t>(FLAGS_max_vector_size)});

  if (context.monotonic_remote_time != context.monotonic_event_time) {
    std::cout << node_name << context.realtime_event_time << " ("
              << context.monotonic_event_time << ") sent "
              << context.realtime_remote_time << " ("
              << context.monotonic_remote_time << ") "
              << channel->name()->c_str() << ' ' << channel->type()->c_str()
              << ": " << *builder << std::endl;
  } else {
    std::cout << node_name << context.realtime_event_time << " ("
              << context.monotonic_event_time << ") "
              << channel->name()->c_str() << ' ' << channel->type()->c_str()
              << ": " << *builder << std::endl;
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
                  << aos::FlatbufferToJson(
                         message.value(),
                         {.multi_line = false, .max_vector_size = 4})
                  << ": "
                  << aos::FlatbufferToJson(
                         channel->schema(),
                         message.value().message().data()->data(),
                         {false, static_cast<size_t>(FLAGS_max_vector_size)})
                  << std::endl;
      } else {
        std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                  << aos::FlatbufferToJson(
                         message.value(),
                         {false, static_cast<size_t>(FLAGS_max_vector_size)})
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

  aos::FastStringBuilder builder;

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());
  reader.Register(&event_loop_factory);

  std::vector<std::unique_ptr<aos::EventLoop>> printer_event_loops;

  for (const aos::Node *node : reader.Nodes()) {
    std::unique_ptr<aos::EventLoop> printer_event_loop =
        event_loop_factory.MakeEventLoop("printer", node);
    printer_event_loop->SkipTimingReport();
    printer_event_loop->SkipAosLog();

    struct MessageInfo {
      std::string node_name;
      std::unique_ptr<aos::RawFetcher> fetcher;
    };
    std::vector<MessageInfo> messages_before_start;

    bool found_channel = false;
    const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
        printer_event_loop->configuration()->channels();

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

        // Fetch the last message on this channel from before the log start
        // time.
        if (FLAGS_fetch) {
          std::unique_ptr<aos::RawFetcher> fetcher =
              printer_event_loop->MakeRawFetcher(channel);
          if (fetcher->Fetch()) {
            MessageInfo message{.node_name = node_name,
                                .fetcher = std::move(fetcher)};
            // Insert it sorted into the vector so we can print in time order
            // instead of channel order at the start.
            auto it = std::lower_bound(
                messages_before_start.begin(), messages_before_start.end(),
                message, [](const MessageInfo &lhs, const MessageInfo &rhs) {
                  if (lhs.fetcher->context().monotonic_event_time <
                      rhs.fetcher->context().monotonic_event_time) {
                    return true;
                  }
                  if (lhs.fetcher->context().monotonic_event_time >
                      rhs.fetcher->context().monotonic_event_time) {
                    return false;
                  }
                  return lhs.fetcher->channel() < rhs.fetcher->channel();
                });
            messages_before_start.insert(it, std::move(message));
          }
        }

        printer_event_loop->MakeRawWatcher(
            channel,
            [channel, node_name, &builder](const aos::Context &context,
                                               const void * /*message*/) {
              PrintMessage(node_name, channel, context, &builder);
            });
        found_channel = true;
      }
    }

    if (!found_channel) {
      LOG(FATAL) << "Could not find any channels";
    }

    // Print the messages from before the log start time.
    // TODO(austin): Sort between nodes too when it becomes annoying enough.
    for (const MessageInfo &message : messages_before_start) {
      PrintMessage(message.node_name, message.fetcher->channel(),
                   message.fetcher->context(), &builder);
    }
    printer_event_loops.emplace_back(std::move(printer_event_loop));

    std::cout << std::endl;
    std::cout << "Log starting at " << reader.realtime_start_time() << " ("
              << reader.monotonic_start_time() << ")";
    std::cout << std::endl << std::endl;
  }

  if (FLAGS_fetch) {
    // New line to separate fetched messages from non-fetched messages.
    std::cout << std::endl;
  }

  event_loop_factory.Run();

  return 0;
}
