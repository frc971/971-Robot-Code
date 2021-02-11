#include <unistd.h>

#include <iostream>

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_int32(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");
DEFINE_bool(fetch, false,
            "If true, fetch the current message on the channel first");
DEFINE_bool(pretty, false,
            "If true, pretty print the messages on multiple lines");
DEFINE_bool(print_timestamps, true, "If true, timestamps are printed.");
DEFINE_uint64(count, 0,
              "If >0, aos_dump will exit after printing this many messages.");
DEFINE_int32(rate_limit, 0,
             "The minimum amount of time to wait in milliseconds before "
             "sending another message");

namespace {

void PrintMessage(const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder) {
  // Print the flatbuffer out to stdout, both to remove the
  // unnecessary cruft from glog and to allow the user to readily
  // redirect just the logged output independent of any debugging
  // information on stderr.

  builder->Reset();
  aos::FlatbufferToJson(
      builder, channel->schema(), static_cast<const uint8_t *>(context.data),
      {FLAGS_pretty, static_cast<size_t>(FLAGS_max_vector_size)});

  if (FLAGS_print_timestamps) {
    if (context.monotonic_remote_time != context.monotonic_event_time) {
      std::cout << context.realtime_remote_time << " ("
                << context.monotonic_remote_time << ") delivered "
                << context.realtime_event_time << " ("
                << context.monotonic_event_time << "): " << *builder << '\n';
    } else {
      std::cout << context.realtime_event_time << " ("
                << context.monotonic_event_time << "): " << *builder << '\n';
    }
  } else {
    std::cout << *builder << '\n';
  }
}

}  // namespace

int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "Prints messages from arbitrary channels as they are received given a "
      "configuration file describing the channels to listen on.\nTypical "
      "Usage: aos_dump [--config path_to_config.json] channel_name "
      "message_type\nExample Usage: aos_dump --config pingpong_config.json "
      "/test aos.examples.Ping");
  aos::InitGoogle(&argc, &argv);

  aos::CliUtilInfo cli_info;
  if (cli_info.Initialize(&argc, &argv,
                          [&cli_info](const aos::Channel *channel) {
                            return aos::configuration::ChannelIsReadableOnNode(
                                channel, cli_info.event_loop->node());
                          })) {
    return 0;
  }

  uint64_t message_count = 0;

  aos::FastStringBuilder str_builder;

  aos::monotonic_clock::time_point next_send_time =
      aos::monotonic_clock::min_time;
  for (const aos::Channel *channel : cli_info.found_channels) {
    if (FLAGS_fetch) {
      const std::unique_ptr<aos::RawFetcher> fetcher =
          cli_info.event_loop->MakeRawFetcher(channel);
      if (fetcher->Fetch()) {
        PrintMessage(channel, fetcher->context(), &str_builder);
        ++message_count;
      }
    }

    if (FLAGS_count > 0 && message_count >= FLAGS_count) {
      return 0;
    }

    cli_info.event_loop->MakeRawWatcher(
        channel,
        [channel, &str_builder, &cli_info, &message_count, &next_send_time](
            const aos::Context &context, const void * /*message*/) {
          if (context.monotonic_event_time > next_send_time) {
            PrintMessage(channel, context, &str_builder);
            ++message_count;
            next_send_time = context.monotonic_event_time +
                             std::chrono::milliseconds(FLAGS_rate_limit);
            if (FLAGS_count > 0 && message_count >= FLAGS_count) {
              cli_info.event_loop->Exit();
            }
          }
        });
  }

  cli_info.event_loop->Run();

  return 0;
}
