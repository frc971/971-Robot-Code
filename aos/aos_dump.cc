#include <unistd.h>

#include <iostream>
#include <map>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_int32(max_vector_size, 100,
             "If positive, vectors longer than this will not be printed");
DEFINE_bool(fetch, false,
            "If true, fetch the current message on the channel first");
DEFINE_bool(pretty, false,
            "If true, pretty print the messages on multiple lines");
DEFINE_bool(all, false,
            "If true, print out the channels for all nodes, not just the "
            "channels which are visible on this node.");
DEFINE_bool(print_timestamps, true, "If true, timestamps are printed.");
DEFINE_uint64(count, 0,
              "If >0, aos_dump will exit after printing this many messages.");
DEFINE_int32(rate_limit, 0,
             "The minimum amount of time to wait in milliseconds before "
             "sending another message");
DEFINE_bool(
    _bash_autocomplete, false,
    "Internal use: Outputs channel list for use with autocomplete script.");
DEFINE_string(_bash_autocomplete_word, "",
              "Intenal use: Index of current word being autocompleted");

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

// Generate eval command to populate autocomplete responses. Eval escapes spaces
// so channels are paired with their types. If a complete channel name is found,
// only autocompletes the type to avoid repeating arguments. Returns no
// autocomplete suggestions if a channel and type is found with the current
// arguments.
void Autocomplete(const aos::Configuration *config_msg,
                  const aos::ShmEventLoop &event_loop,
                  std::string_view channel_name,
                  std::string_view message_type) {
  const bool unique_match =
      std::count_if(
          config_msg->channels()->begin(), config_msg->channels()->end(),
          [channel_name, message_type](const aos::Channel *channel) {
            return channel->name()->string_view() == channel_name &&
                   channel->type()->string_view() == message_type;
          }) == 1;

  const bool editing_message = !channel_name.empty() && FLAGS__bash_autocomplete_word == message_type;
  const bool editing_channel = !editing_message && FLAGS__bash_autocomplete_word == channel_name;

  std::cout << "COMPREPLY=(";

  // If we have a unique match, don't provide any suggestions. Otherwise, check
  // that were're editing one of the two positional arguments.
  if (!unique_match && (editing_message || editing_channel)) {
    for (const aos::Channel *channel : *config_msg->channels()) {
      if (FLAGS_all || aos::configuration::ChannelIsReadableOnNode(
                           channel, event_loop.node())) {
        // Suggest only message types if the message type argument is being
        // entered.
        if (editing_message) {
          // Then, filter for only channel names that match exactly and types
          // that begin with message_type.
          if (channel->name()->string_view() == channel_name &&
              channel->type()->string_view().find(message_type) == 0) {
            std::cout << '\'' << channel->type()->c_str() << "' ";
          }
        } else if (channel->name()->string_view().find(channel_name) == 0) {
          // If the message type empty, then return full autocomplete.
          // Otherwise, since the message type is poulated yet not being edited,
          // the user must be editing the channel name alone, in which case only
          // suggest channel names, not pairs.
          if (message_type.empty()) {
            std::cout << '\'' << channel->name()->c_str() << ' '
                      << channel->type()->c_str() << "' ";
          } else {
            std::cout << '\'' << channel->name()->c_str() << "' ";
          }
        }
      }
    }
  }
  std::cout << ')';
}

bool EndsWith(std::string_view str, std::string_view ending) {
  const std::size_t offset = str.size() - ending.size();
  return str.size() >= ending.size() && std::equal(str.begin() + offset, str.end(),
                                   ending.begin(), ending.end());
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

  // Don't generate failure output if the config doesn't exist while attempting
  // to autocomplete.
  if (struct stat file_stat;
      FLAGS__bash_autocomplete &&
      (!(EndsWith(FLAGS_config, ".json") || EndsWith(FLAGS_config, ".bfbs")) ||
       stat(FLAGS_config.c_str(), &file_stat) != 0 ||
       (file_stat.st_mode & S_IFMT) != S_IFREG)) {
    std::cout << "COMPREPLY=()";
    return 0;
  }

  std::string channel_name;
  std::string message_type;
  if (argc > 1) {
    channel_name = argv[1];
  }
  if (argc > 2) {
    message_type = argv[2];
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  const aos::Configuration *config_msg = &config.message();
  aos::ShmEventLoop event_loop(config_msg);
  event_loop.SkipTimingReport();
  event_loop.SkipAosLog();

  if (FLAGS__bash_autocomplete) {
    Autocomplete(config_msg, event_loop, channel_name, message_type);
    return 0;
  }

  if (argc == 1) {
    std::cout << "Channels:\n";
    for (const aos::Channel *channel : *config_msg->channels()) {
      if (FLAGS_all || aos::configuration::ChannelIsReadableOnNode(
                           channel, event_loop.node())) {
        std::cout << channel->name()->c_str() << ' ' << channel->type()->c_str()
                  << '\n';
      }
    }
    return 0;
  }

  std::vector<const aos::Channel *> found_channels;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config_msg->channels();
  bool found_exact = false;
  for (const aos::Channel *channel : *channels) {
    if (!FLAGS_all && !aos::configuration::ChannelIsReadableOnNode(
                          channel, event_loop.node())) {
      continue;
    }
    if (channel->name()->c_str() != channel_name) {
      continue;
    }
    if (channel->type()->string_view() == message_type) {
      if (!found_exact) {
        found_channels.clear();
        found_exact = true;
      }
    } else if (!found_exact && channel->type()->string_view().find(
                                   message_type) != std::string_view::npos) {
    } else {
      continue;
    }
    found_channels.push_back(channel);
  }

  if (found_channels.empty()) {
    LOG(FATAL) << "Could not find any channels with the given name and type.";
  } else if (found_channels.size() > 1 && !message_type.empty()) {
    LOG(FATAL) << "Multiple channels found with same type";
  }

  uint64_t message_count = 0;

  aos::FastStringBuilder str_builder;

  aos::monotonic_clock::time_point next_send_time =
      aos::monotonic_clock::min_time;
  for (const aos::Channel *channel : found_channels) {
    if (FLAGS_fetch) {
      const std::unique_ptr<aos::RawFetcher> fetcher =
          event_loop.MakeRawFetcher(channel);
      if (fetcher->Fetch()) {
        PrintMessage(channel, fetcher->context(), &str_builder);
        ++message_count;
      }
    }

    if (FLAGS_count > 0 && message_count >= FLAGS_count) {
      return 0;
    }

    event_loop.MakeRawWatcher(
        channel,
        [channel, &str_builder, &event_loop, &message_count, &next_send_time](
            const aos::Context &context, const void * /*message*/) {
          if (context.monotonic_event_time > next_send_time) {
            PrintMessage(channel, context, &str_builder);
            next_send_time = context.monotonic_event_time +
                             std::chrono::milliseconds(FLAGS_rate_limit);
            if (FLAGS_count > 0 && message_count >= FLAGS_count) {
              event_loop.Exit();
            }
          }
        });
  }

  event_loop.Run();

  return 0;
}
