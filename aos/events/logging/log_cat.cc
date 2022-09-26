#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/strings/escaping.h"
#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
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
DEFINE_bool(json, false, "If true, print fully valid JSON");
DEFINE_bool(fetch, false,
            "If true, also print out the messages from before the start of the "
            "log file");
DEFINE_bool(raw, false,
            "If true, just print the data out unsorted and unparsed");
DEFINE_string(raw_header, "",
              "If set, the file to read the header from in raw mode");
DEFINE_bool(distributed_clock, false,
            "If true, print out the distributed time");
DEFINE_bool(format_raw, true,
            "If true and --raw is specified, print out raw data, but use the "
            "schema to format the data.");
DEFINE_int64(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");
DEFINE_bool(pretty, false,
            "If true, pretty print the messages on multiple lines");
DEFINE_bool(
    pretty_max, false,
    "If true, expand every field to its own line (expands more than -pretty)");
DEFINE_bool(print_timestamps, true, "If true, timestamps are printed.");
DEFINE_bool(print, true,
            "If true, actually print the messages.  If false, discard them, "
            "confirming they can be parsed.");
DEFINE_uint64(
    count, 0,
    "If >0, log_cat will exit after printing this many messages.  This "
    "includes messages from before the start of the log if --fetch is set.");
DEFINE_bool(print_parts_only, false,
            "If true, only print out the results of logfile sorting.");
DEFINE_bool(channels, false,
            "If true, print out all the configured channels for this log.");
DEFINE_double(monotonic_start_time, 0.0,
              "If set, only print messages sent at or after this many seconds "
              "after epoch.");
DEFINE_double(monotonic_end_time, 0.0,
              "If set, only print messages sent at or before this many seconds "
              "after epoch.");
DEFINE_bool(use_hex, false, "Are integers in the messages printed in hex notation.");

using aos::monotonic_clock;
namespace chrono = std::chrono;

// Prints out raw log parts to stdout.
int PrintRaw(int argc, char **argv) {
  if (argc == 1) {
    CHECK(!FLAGS_raw_header.empty());
    aos::logger::MessageReader raw_header_reader(FLAGS_raw_header);
    std::cout << aos::FlatbufferToJson(raw_header_reader.raw_log_file_header(),
                                       {.multi_line = FLAGS_pretty,
                                        .max_vector_size = static_cast<size_t>(
                                            FLAGS_max_vector_size)})
              << std::endl;
    return 0;
  }
  if (argc != 2 && argc != 1) {
    LOG(FATAL) << "Expected 1 logfile as an argument.";
  }
  aos::logger::SpanReader reader(argv[1]);
  absl::Span<const uint8_t> raw_log_file_header_span = reader.ReadMessage();

  if (raw_log_file_header_span == absl::Span<const uint8_t>()) {
    LOG(WARNING) << "Empty log file on " << reader.filename();
    return 0;
  }

  // Now, reproduce the log file header deduplication logic inline so we can
  // print out all the headers we find.
  aos::SizePrefixedFlatbufferVector<aos::logger::LogFileHeader> log_file_header(
      raw_log_file_header_span);
  if (!log_file_header.Verify()) {
    LOG(ERROR) << "Header corrupted on " << reader.filename();
    return 1;
  }
  while (true) {
    absl::Span<const uint8_t> maybe_header_data = reader.PeekMessage();
    if (maybe_header_data == absl::Span<const uint8_t>()) {
      break;
    }

    aos::SizePrefixedFlatbufferSpan<aos::logger::LogFileHeader> maybe_header(
        maybe_header_data);
    if (maybe_header.Verify()) {
      std::cout << aos::FlatbufferToJson(
                       log_file_header, {.multi_line = FLAGS_pretty,
                                         .max_vector_size = static_cast<size_t>(
                                             FLAGS_max_vector_size)})
                << std::endl;
      LOG(WARNING) << "Found duplicate LogFileHeader in " << reader.filename();
      log_file_header =
          aos::SizePrefixedFlatbufferVector<aos::logger::LogFileHeader>(
              maybe_header_data);

      reader.ConsumeMessage();
    } else {
      break;
    }
  }

  // And now use the final sha256 to match the raw_header.
  std::optional<aos::logger::MessageReader> raw_header_reader;
  const aos::logger::LogFileHeader *full_header = &log_file_header.message();
  if (!FLAGS_raw_header.empty()) {
    raw_header_reader.emplace(FLAGS_raw_header);
    std::cout << aos::FlatbufferToJson(full_header,
                                       {.multi_line = FLAGS_pretty,
                                        .max_vector_size = static_cast<size_t>(
                                            FLAGS_max_vector_size)})
              << std::endl;
    CHECK_EQ(
        full_header->configuration_sha256()->string_view(),
        aos::logger::Sha256(raw_header_reader->raw_log_file_header().span()));
    full_header = raw_header_reader->log_file_header();
  }

  if (!FLAGS_print) {
    return 0;
  }

  std::cout << aos::FlatbufferToJson(full_header,
                                     {.multi_line = FLAGS_pretty,
                                      .max_vector_size = static_cast<size_t>(
                                          FLAGS_max_vector_size)})
            << std::endl;
  CHECK(full_header->has_configuration())
      << ": Missing configuration! You may want to provide the path to the "
         "logged configuration file using the --raw_header flag.";

  while (true) {
    const aos::SizePrefixedFlatbufferSpan<aos::logger::MessageHeader> message(
        reader.ReadMessage());
    if (message.span() == absl::Span<const uint8_t>()) {
      break;
    }
    CHECK(message.Verify());

    const auto *const channels = full_header->configuration()->channels();
    const size_t channel_index = message.message().channel_index();
    CHECK_LT(channel_index, channels->size());
    const aos::Channel *const channel = channels->Get(channel_index);

    CHECK(message.Verify()) << absl::BytesToHexString(
        std::string_view(reinterpret_cast<const char *>(message.span().data()),
                         message.span().size()));

    if (message.message().data() != nullptr) {
      CHECK(channel->has_schema());

      CHECK(flatbuffers::Verify(
          *channel->schema(), *channel->schema()->root_table(),
          message.message().data()->data(), message.message().data()->size()))
          << ": Corrupted flatbuffer on " << channel->name()->c_str() << " "
          << channel->type()->c_str();
    }

    if (FLAGS_format_raw && message.message().data() != nullptr) {
      std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                << aos::FlatbufferToJson(message, {.multi_line = FLAGS_pretty,
                                                   .max_vector_size = 4})
                << ": "
                << aos::FlatbufferToJson(
                       channel->schema(), message.message().data()->data(),
                       {FLAGS_pretty,
                        static_cast<size_t>(FLAGS_max_vector_size)})
                << std::endl;
    } else {
      std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                << aos::FlatbufferToJson(
                       message, {FLAGS_pretty,
                                 static_cast<size_t>(FLAGS_max_vector_size)})
                << std::endl;
    }
  }
  return 0;
}

// This class prints out all data from a node on a boot.
class NodePrinter {
 public:
  NodePrinter(aos::EventLoop *event_loop, uint64_t *message_print_counter,
              aos::SimulatedEventLoopFactory *factory,
              aos::FastStringBuilder *builder)
      : factory_(factory),
        node_factory_(factory->GetNodeEventLoopFactory(event_loop->node())),
        event_loop_(event_loop),
        message_print_counter_(message_print_counter),
        node_name_(
            event_loop_->node() == nullptr
                ? ""
                : std::string(event_loop->node()->name()->string_view())),
        builder_(builder) {
    event_loop_->SkipTimingReport();
    event_loop_->SkipAosLog();

    const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
        event_loop_->configuration()->channels();

    const monotonic_clock::time_point start_time =
        (FLAGS_monotonic_start_time == 0.0
             ? monotonic_clock::min_time
             : monotonic_clock::time_point(
                   std::chrono::duration_cast<monotonic_clock::duration>(
                       std::chrono::duration<double>(
                           FLAGS_monotonic_start_time))));
    const monotonic_clock::time_point end_time =
        (FLAGS_monotonic_end_time == 0.0
             ? monotonic_clock::max_time
             : monotonic_clock::time_point(
                   std::chrono::duration_cast<monotonic_clock::duration>(
                       std::chrono::duration<double>(
                           FLAGS_monotonic_end_time))));

    for (flatbuffers::uoffset_t i = 0; i < channels->size(); i++) {
      const aos::Channel *channel = channels->Get(i);
      const flatbuffers::string_view name = channel->name()->string_view();
      const flatbuffers::string_view type = channel->type()->string_view();
      if (name.find(FLAGS_name) != std::string::npos &&
          type.find(FLAGS_type) != std::string::npos) {
        if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                         event_loop_->node())) {
          continue;
        }
        VLOG(1) << "Listening on " << name << " " << type;

        CHECK_NOTNULL(channel->schema());
        event_loop_->MakeRawWatcher(channel, [this, channel, start_time,
                                              end_time](
                                                 const aos::Context &context,
                                                 const void * /*message*/) {
          if (!FLAGS_print) {
            return;
          }

          if (!FLAGS_fetch && !started_) {
            return;
          }

          if (context.monotonic_event_time < start_time ||
              context.monotonic_event_time > end_time) {
            return;
          }

          PrintMessage(
              node_name_, node_factory_, channel, context, builder_,
              {
                  .pretty = FLAGS_pretty,
                  .max_vector_size = static_cast<size_t>(FLAGS_max_vector_size),
                  .pretty_max = FLAGS_pretty_max,
                  .print_timestamps = FLAGS_print_timestamps,
                  .json = FLAGS_json,
                  .distributed_clock = FLAGS_distributed_clock,
                  .use_hex = FLAGS_use_hex,
              });
          ++(*message_print_counter_);
          if (FLAGS_count > 0 && *message_print_counter_ >= FLAGS_count) {
            factory_->Exit();
          }
        });
      }
    }
  }

  void SetStarted(bool started, aos::monotonic_clock::time_point monotonic_now,
                  aos::realtime_clock::time_point realtime_now) {
    started_ = started;
    if (FLAGS_json) {
      return;
    }
    if (started_) {
      std::cout << std::endl;
      std::cout << (event_loop_->node() != nullptr
                        ? (event_loop_->node()->name()->str() + " ")
                        : "")
                << "Log starting at " << realtime_now << " (" << monotonic_now
                << ")";
      std::cout << std::endl << std::endl;
    } else {
      std::cout << std::endl;
      std::cout << (event_loop_->node() != nullptr
                        ? (event_loop_->node()->name()->str() + " ")
                        : "")
                << "Log shutting down at " << realtime_now << " ("
                << monotonic_now << ")";
      std::cout << std::endl << std::endl;
    }
  }

 private:
  struct MessageInfo {
    std::string node_name;
    std::unique_ptr<aos::RawFetcher> fetcher;
  };

  aos::SimulatedEventLoopFactory *factory_;
  aos::NodeEventLoopFactory *node_factory_;
  aos::EventLoop *event_loop_;

  uint64_t *message_print_counter_ = nullptr;

  std::string node_name_;

  bool started_ = false;

  aos::FastStringBuilder *builder_;
};

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
    return PrintRaw(argc, argv);
  }

  if (argc < 2) {
    LOG(FATAL) << "Expected at least 1 logfile as an argument.";
  }

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  for (auto &it : logfiles) {
    VLOG(1) << it;
    if (FLAGS_print_parts_only) {
      std::cout << it << std::endl;
    }
  }
  if (FLAGS_print_parts_only) {
    return 0;
  }

  aos::logger::LogReader reader(logfiles);

  if (FLAGS_channels) {
    const aos::Configuration *config = reader.configuration();
    for (const aos::Channel *channel : *config->channels()) {
      std::cout << channel->name()->c_str() << " " << channel->type()->c_str()
                << '\n';
    }
    return 0;
  }

  {
    bool found_channel = false;
    const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
        reader.configuration()->channels();

    for (flatbuffers::uoffset_t i = 0; i < channels->size(); i++) {
      const aos::Channel *channel = channels->Get(i);
      const flatbuffers::string_view name = channel->name()->string_view();
      const flatbuffers::string_view type = channel->type()->string_view();
      if (name.find(FLAGS_name) != std::string::npos &&
          type.find(FLAGS_type) != std::string::npos) {
        found_channel = true;
      }
    }
    if (!found_channel) {
      LOG(FATAL) << "Could not find any channels";
    }
  }

  aos::FastStringBuilder builder;

  uint64_t message_print_counter = 0;

  std::vector<NodePrinter *> printers;
  printers.resize(aos::configuration::NodesCount(reader.configuration()),
                  nullptr);

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());

  reader.RegisterWithoutStarting(&event_loop_factory);

  for (const aos::Node *node :
       aos::configuration::GetNodes(event_loop_factory.configuration())) {
    size_t node_index = aos::configuration::GetNodeIndex(
        event_loop_factory.configuration(), node);
    // Spin up the printer, and hook up the SetStarted method so that it gets
    // notified when the log starts and stops.
    aos::NodeEventLoopFactory *node_factory =
        event_loop_factory.GetNodeEventLoopFactory(node);
    node_factory->OnStartup([&event_loop_factory, node_factory,
                             &message_print_counter, &builder, &printers,
                             node_index]() {
      printers[node_index] = node_factory->AlwaysStart<NodePrinter>(
          "printer", &message_print_counter, &event_loop_factory, &builder);
    });
    node_factory->OnShutdown(
        [&printers, node_index]() { printers[node_index] = nullptr; });

    reader.OnStart(node, [&printers, node_index, node_factory]() {
      CHECK(printers[node_index]);
      printers[node_index]->SetStarted(true, node_factory->monotonic_now(),
                                       node_factory->realtime_now());
    });
    reader.OnEnd(node, [&printers, node_index, node_factory]() {
      CHECK(printers[node_index]);
      printers[node_index]->SetStarted(false, node_factory->monotonic_now(),
                                       node_factory->realtime_now());
    });
  }

  event_loop_factory.Run();

  reader.Deregister();

  return 0;
}
