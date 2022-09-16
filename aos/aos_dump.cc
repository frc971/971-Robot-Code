#include <unistd.h>

#include <iostream>

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_int64(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");
DEFINE_bool(json, false, "If true, print fully valid JSON");
DEFINE_bool(fetch, false,
            "If true, fetch the current message on the channel first");
DEFINE_bool(pretty, false,
            "If true, pretty print the messages on multiple lines");
DEFINE_bool(
    pretty_max, false,
    "If true, expand every field to its own line (expands more than -pretty)");
DEFINE_bool(print_timestamps, true, "If true, timestamps are printed.");
DEFINE_uint64(count, 0,
              "If >0, aos_dump will exit after printing this many messages.");
DEFINE_int32(rate_limit, 0,
             "The minimum amount of time to wait in milliseconds before "
             "sending another message");
DEFINE_int32(timeout, -1,
             "The max time in milliseconds to wait for messages before "
             "exiting.  -1 means forever, 0 means don't wait.");
DEFINE_bool(use_hex, false, "Are integers in the messages printed in hex notation.");

int main(int argc, char **argv) {
  gflags::SetUsageMessage(
      "Prints messages from arbitrary channels as they are received given a "
      "configuration file describing the channels to listen on.\nTypical "
      "Usage: aos_dump [--config path_to_config.json] channel_name "
      "message_type\nExample Usage: aos_dump --config pingpong_config.json "
      "/test aos.examples.Ping");
  aos::InitGoogle(&argc, &argv);

  aos::CliUtilInfo cli_info;
  if (cli_info.Initialize(
          &argc, &argv,
          [&cli_info](const aos::Channel *channel) {
            return aos::configuration::ChannelIsReadableOnNode(
                channel, cli_info.event_loop->node());
          },
          "channel is readeable on node", true)) {
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
        PrintMessage(
            channel, fetcher->context(), &str_builder,
            {
                .pretty = FLAGS_pretty,
                .max_vector_size = static_cast<size_t>(FLAGS_max_vector_size),
                .pretty_max = FLAGS_pretty_max,
                .print_timestamps = FLAGS_print_timestamps,
                .json = FLAGS_json,
                .distributed_clock = false,
                .use_hex = FLAGS_use_hex,
            });
        ++message_count;
      }
    }

    if (FLAGS_count > 0 && message_count >= FLAGS_count) {
      return 0;
    }

    if (FLAGS_timeout == 0) {
      continue;
    }

    cli_info.event_loop->MakeRawWatcher(
        channel,
        [channel, &str_builder, &cli_info, &message_count, &next_send_time](
            const aos::Context &context, const void * /*message*/) {
          if (context.monotonic_event_time > next_send_time) {
            if (FLAGS_count > 0 && message_count >= FLAGS_count) {
              return;
            }
            PrintMessage(channel, context, &str_builder,
                         {
                             .pretty = FLAGS_pretty,
                             .max_vector_size =
                                 static_cast<size_t>(FLAGS_max_vector_size),
                             .pretty_max = FLAGS_pretty_max,
                             .print_timestamps = FLAGS_print_timestamps,
                             .json = FLAGS_json,
                             .distributed_clock = false,
                             .use_hex = FLAGS_use_hex,
                         });
            ++message_count;
            next_send_time = context.monotonic_event_time +
                             std::chrono::milliseconds(FLAGS_rate_limit);
            if (FLAGS_count > 0 && message_count >= FLAGS_count) {
              cli_info.event_loop->Exit();
            }
          }
        });
  }

  if (FLAGS_timeout == 0) {
    return 0;
  }

  if (FLAGS_timeout > 0) {
    aos::TimerHandler *handle = cli_info.event_loop->AddTimer(
        [event_loop = &cli_info.event_loop.value()]() { event_loop->Exit(); });

    cli_info.event_loop->OnRun(
        [handle, event_loop = &cli_info.event_loop.value()]() {
          handle->Setup(event_loop->monotonic_now() +
                        std::chrono::milliseconds(FLAGS_timeout));
        });
  }

  cli_info.event_loop->Run();

  return 0;
}
