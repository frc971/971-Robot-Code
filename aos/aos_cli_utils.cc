#include "aos/aos_cli_utils.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

DEFINE_string(config, "./config.json", "File path of aos configuration");

DEFINE_bool(
    _bash_autocomplete, false,
    "Internal use: Outputs channel list for use with autocomplete script.");
DEFINE_string(_bash_autocomplete_word, "",
              "Internal use: Current word being autocompleted");

DEFINE_bool(all, false,
            "If true, print out the channels for all nodes, not just the "
            "channels which are visible on this node.");

namespace aos {
namespace {

bool EndsWith(std::string_view str, std::string_view ending) {
  const std::size_t offset = str.size() - ending.size();
  return str.size() >= ending.size() &&
         std::equal(str.begin() + offset, str.end(), ending.begin(),
                    ending.end());
}

}  // namespace

bool CliUtilInfo::Initialize(
    int *argc, char ***argv,
    std::function<bool(const aos::Channel *)> channel_filter) {
  // Don't generate failure output if the config doesn't exist while attempting
  // to autocomplete.
  if (struct stat file_stat;
      FLAGS__bash_autocomplete &&
      (!(EndsWith(FLAGS_config, ".json") || EndsWith(FLAGS_config, ".bfbs")) ||
       stat(FLAGS_config.c_str(), &file_stat) != 0 ||
       (file_stat.st_mode & S_IFMT) != S_IFREG)) {
    std::cout << "COMPREPLY=()";
    return true;
  }

  config.emplace(aos::configuration::ReadConfig(FLAGS_config));
  event_loop.emplace(&config->message());
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      event_loop->configuration()->channels();

  do {
    std::string channel_name;
    std::string message_type;
    if (*argc > 1) {
      channel_name = (*argv)[1];
      ShiftArgs(argc, argv);
    }
    if (*argc > 1) {
      message_type = (*argv)[1];
      ShiftArgs(argc, argv);
    }

    if (FLAGS__bash_autocomplete) {
      Autocomplete(channel_name, message_type, channel_filter);
      return true;
    }

    if (channel_name.empty() && message_type.empty()) {
      std::cout << "Channels:\n";
      for (const aos::Channel *channel : *channels) {
        if (FLAGS_all || channel_filter(channel)) {
          std::cout << channel->name()->c_str() << ' '
                    << channel->type()->c_str() << '\n';
        }
      }
      return true;
    }

    std::vector<const aos::Channel *> found_channels_now;
    bool found_exact = false;
    for (const aos::Channel *channel : *channels) {
      if (!FLAGS_all && !channel_filter(channel)) {
        continue;
      }
      if (channel->name()->c_str() != channel_name) {
        continue;
      }
      if (channel->type()->string_view() == message_type) {
        if (!found_exact) {
          found_channels_now.clear();
          found_exact = true;
        }
      } else if (!found_exact && channel->type()->string_view().find(
                                     message_type) != std::string_view::npos) {
      } else {
        continue;
      }
      found_channels_now.push_back(channel);
    }

    if (found_channels_now.empty()) {
      LOG(FATAL)
          << "Could not find any channels with the given name and type for "
          << channel_name << " " << message_type;
    } else if (found_channels_now.size() > 1 && !message_type.empty()) {
      LOG(FATAL) << "Multiple channels found with same type for "
                 << channel_name << " " << message_type;
    }
    for (const aos::Channel *channel : found_channels_now) {
      found_channels.push_back(channel);
    }
  } while (*argc > 1);

  return false;
}

void CliUtilInfo::Autocomplete(
    std::string_view channel_name, std::string_view message_type,
    std::function<bool(const aos::Channel *)> channel_filter) {
  const aos::Configuration *const config_msg = event_loop->configuration();
  const bool unique_match =
      std::count_if(config_msg->channels()->begin(),
                    config_msg->channels()->end(),
                    [channel_name, message_type](const aos::Channel *channel) {
                      return channel->name()->string_view() == channel_name &&
                             channel->type()->string_view() == message_type;
                    }) == 1;

  const bool editing_message =
      !channel_name.empty() && FLAGS__bash_autocomplete_word == message_type;
  const bool editing_channel =
      !editing_message && FLAGS__bash_autocomplete_word == channel_name;

  std::cout << "COMPREPLY=(";

  // If we have a unique match, don't provide any suggestions. Otherwise, check
  // that were're editing one of the two positional arguments.
  if (!unique_match && (editing_message || editing_channel)) {
    for (const aos::Channel *channel : *config_msg->channels()) {
      if (FLAGS_all || channel_filter(channel)) {
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

}  // namespace aos
