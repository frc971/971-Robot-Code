#include <unistd.h>

#include <iostream>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"

ABSL_FLAG(int64_t, max_vector_size, 100,
          "If positive, vectors longer than this will not be printed");
ABSL_FLAG(bool, json, false, "If true, print fully valid JSON");
ABSL_FLAG(bool, fetch, false,
          "If true, fetch the current message on the channel first");
ABSL_FLAG(bool, pretty, false,
          "If true, pretty print the messages on multiple lines");
ABSL_FLAG(
    bool, pretty_max, false,
    "If true, expand every field to its own line (expands more than -pretty)");
ABSL_FLAG(bool, print_timestamps, true, "If true, timestamps are printed.");
ABSL_FLAG(uint64_t, count, 0,
          "If >0, aos_dump will exit after printing this many messages.");
ABSL_FLAG(int32_t, rate_limit, 0,
          "The minimum amount of time to wait in milliseconds before "
          "sending another message");
ABSL_FLAG(int32_t, timeout, -1,
          "The max time in milliseconds to wait for messages before "
          "exiting.  -1 means forever, 0 means don't wait.");
ABSL_FLAG(bool, hex, false,
          "Are integers in the messages printed in hex notation.");

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
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

  aos::monotonic_clock::time_point next_send_time =
      aos::monotonic_clock::min_time;

  aos::Printer printer(
      {
          .pretty = absl::GetFlag(FLAGS_pretty),
          .max_vector_size =
              static_cast<size_t>(absl::GetFlag(FLAGS_max_vector_size)),
          .pretty_max = absl::GetFlag(FLAGS_pretty_max),
          .print_timestamps = absl::GetFlag(FLAGS_print_timestamps),
          .json = absl::GetFlag(FLAGS_json),
          .distributed_clock = false,
          .hex = absl::GetFlag(FLAGS_hex),
      },
      /*flush*/ true);

  for (const aos::Channel *channel : cli_info.found_channels) {
    if (absl::GetFlag(FLAGS_fetch)) {
      const std::unique_ptr<aos::RawFetcher> fetcher =
          cli_info.event_loop->MakeRawFetcher(channel);
      if (fetcher->Fetch()) {
        printer.PrintMessage(channel, fetcher->context());
      }
    }

    if (absl::GetFlag(FLAGS_count) > 0 &&
        printer.message_count() >= absl::GetFlag(FLAGS_count)) {
      return 0;
    }

    if (absl::GetFlag(FLAGS_timeout) == 0) {
      continue;
    }

    cli_info.event_loop->MakeRawWatcher(
        channel, [channel, &printer, &cli_info, &next_send_time](
                     const aos::Context &context, const void * /*message*/) {
          if (context.monotonic_event_time > next_send_time) {
            if (absl::GetFlag(FLAGS_count) > 0 &&
                printer.message_count() >= absl::GetFlag(FLAGS_count)) {
              return;
            }

            printer.PrintMessage(channel, context);
            next_send_time =
                context.monotonic_event_time +
                std::chrono::milliseconds(absl::GetFlag(FLAGS_rate_limit));
            if (absl::GetFlag(FLAGS_count) > 0 &&
                printer.message_count() >= absl::GetFlag(FLAGS_count)) {
              cli_info.event_loop->Exit();
            }
          }
        });
  }

  if (absl::GetFlag(FLAGS_timeout) == 0) {
    return 0;
  }

  if (absl::GetFlag(FLAGS_timeout) > 0) {
    aos::TimerHandler *handle = cli_info.event_loop->AddTimer(
        [event_loop = &cli_info.event_loop.value()]() { event_loop->Exit(); });

    cli_info.event_loop->OnRun([handle,
                                event_loop = &cli_info.event_loop.value()]() {
      handle->Schedule(event_loop->monotonic_now() +
                       std::chrono::milliseconds(absl::GetFlag(FLAGS_timeout)));
    });
  }

  cli_info.event_loop->Run();

  return 0;
}
