#include "aos/aos_cli_utils.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iostream>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"

DEFINE_string(config, "aos_config.json", "File path of aos configuration");

DEFINE_bool(
    _bash_autocomplete, false,
    "Internal use: Outputs channel list for use with autocomplete script.");

DEFINE_bool(_zsh_compatability, false,
            "Internal use: Force completion to complete either channels or "
            "message_types, zsh doesn't handle spaces well.");

DEFINE_string(_bash_autocomplete_word, "",
              "Internal use: Current word being autocompleted");

DEFINE_bool(all, false,
            "If true, print out the channels for all nodes, not just the "
            "channels which are visible on this node.");

namespace aos {
namespace {

namespace chrono = std::chrono;

bool EndsWith(std::string_view str, std::string_view ending) {
  const std::size_t offset = str.size() - ending.size();
  return str.size() >= ending.size() &&
         std::equal(str.begin() + offset, str.end(), ending.begin(),
                    ending.end());
}

void StreamSeconds(std::ostream &stream,
                   const aos::monotonic_clock::time_point now) {
  if (now < monotonic_clock::epoch()) {
    chrono::seconds seconds =
        chrono::duration_cast<chrono::seconds>(now.time_since_epoch());

    stream << "-" << -seconds.count() << "." << std::setfill('0')
           << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(seconds -
                                                         now.time_since_epoch())
                  .count();
  } else {
    chrono::seconds seconds =
        chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
    stream << seconds.count() << "." << std::setfill('0') << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(
                  now.time_since_epoch() - seconds)
                  .count();
  }
}

}  // namespace

bool CliUtilInfo::Initialize(
    int *argc, char ***argv,
    std::function<bool(const aos::Channel *)> channel_filter,
    std::string_view channel_filter_description, bool expect_args) {
  // Don't generate failure output if the config doesn't exist while attempting
  // to autocomplete.
  if (FLAGS__bash_autocomplete &&
      (!(EndsWith(FLAGS_config, ".json") || EndsWith(FLAGS_config, ".bfbs")))) {
    std::cout << "COMPREPLY=()";
    return true;
  }

  config = aos::configuration::MaybeReadConfig(FLAGS_config);
  if (FLAGS__bash_autocomplete && !config.has_value()) {
    std::cout << "COMPREPLY=()";
    return true;
  }
  CHECK(config.has_value()) << "Could not read config. See above errors.";

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

      if (!FLAGS_all && !channel_filter(channel)) {
        LOG(FATAL) << "matched channel does not pass the channel filter: \""
                   << channel_filter_description
                   << "\" [matched channel info]: "
                   << configuration::CleanedChannelToString(channel);
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
  } while (expect_args && *argc > 1);

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
          // If _split_complete flag is set then dont return
          // pairs of values
          if (!FLAGS__zsh_compatability && message_type.empty()) {
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

void PrintMessage(const std::string_view node_name, const aos::Channel *channel,
                  const aos::Context &context, aos::FastStringBuilder *builder,
                  PrintOptions options) {
  // Print the flatbuffer out to stdout, both to remove the
  // unnecessary cruft from glog and to allow the user to readily
  // redirect just the logged output independent of any debugging
  // information on stderr.

  builder->Reset();

  CHECK(flatbuffers::Verify(*channel->schema(),
                            *channel->schema()->root_table(),
                            static_cast<const uint8_t *>(context.data),
                            static_cast<size_t>(context.size)))
      << ": Corrupted flatbuffer on " << channel->name()->c_str() << " "
      << channel->type()->c_str();

  aos::FlatbufferToJson(
      builder, channel->schema(), static_cast<const uint8_t *>(context.data),
      {options.pretty, static_cast<size_t>(options.max_vector_size),
       options.pretty_max, options.hex});

  if (options.json) {
    std::cout << "{";
    if (!node_name.empty()) {
      std::cout << "\"node\": \"" << node_name << "\", ";
    }
    std::cout << "\"monotonic_event_time\": ";
    StreamSeconds(std::cout, context.monotonic_event_time);
    std::cout << ", \"realtime_event_time\": \"" << context.realtime_event_time
              << "\", ";

    if (context.monotonic_remote_time != context.monotonic_event_time) {
      std::cout << "\"monotonic_remote_time\": ";
      StreamSeconds(std::cout, context.monotonic_remote_time);
      std::cout << ", \"realtime_remote_time\": \""
                << context.realtime_remote_time << "\", ";
    }

    std::cout << "\"channel\": "
              << aos::configuration::StrippedChannelToString(channel)
              << ", \"data\": " << *builder << "}";
  } else {
    if (!node_name.empty()) {
      std::cout << node_name << " ";
    }

    if (options.print_timestamps) {
      if (context.monotonic_remote_time != context.monotonic_event_time) {
        std::cout << context.realtime_event_time << " ("
                  << context.monotonic_event_time << ") sent "
                  << context.realtime_remote_time << " ("
                  << context.monotonic_remote_time << ") "
                  << channel->name()->c_str() << ' ' << channel->type()->c_str()
                  << ": " << *builder;
      } else {
        std::cout << context.realtime_event_time << " ("
                  << context.monotonic_event_time << ") "
                  << channel->name()->c_str() << ' ' << channel->type()->c_str()
                  << ": " << *builder;
      }
    } else {
      std::cout << *builder;
    }
  }
}

void PrintMessage(const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder, PrintOptions options) {
  PrintMessage("", channel, context, builder, options);
}

void PrintMessage(const std::string_view node_name,
                  aos::NodeEventLoopFactory *node_factory,
                  const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder, PrintOptions options) {
  if (!options.json && options.distributed_clock) {
    std::cout << node_factory->ToDistributedClock(context.monotonic_event_time)
              << " ";
  }
  PrintMessage(node_name, channel, context, builder, options);
}

Printer::Printer(PrintOptions options, bool flush)
    : options_(options), flush_(flush) {
  if (options_.json) {
    std::cout << "[";
  }
}

Printer::~Printer() {
  if (options_.json) {
    if (message_count_ > 0) {
      std::cout << "\n]\n";
    } else {
      std::cout << "]\n";
    }
  }
}

void Printer::PrintMessage(const std::string_view node_name,
                           aos::NodeEventLoopFactory *node_factory,
                           const aos::Channel *channel,
                           const aos::Context &context) {
  if (options_.json) {
    if (message_count_ != 0) {
      std::cout << ",\n  ";
    } else {
      std::cout << "\n  ";
    }
  }

  aos::PrintMessage(node_name, node_factory, channel, context, &str_builder_,
                    options_);

  if (!options_.json) {
    if (flush_) {
      std::cout << std::endl;
    } else {
      std::cout << "\n";
    }
  }
  ++message_count_;
}

void Printer::PrintMessage(const aos::Channel *channel,
                           const aos::Context &context) {
  if (options_.json) {
    if (message_count_ != 0) {
      std::cout << ",\n  ";
    } else {
      std::cout << "\n  ";
    }
  }

  aos::PrintMessage(channel, context, &str_builder_, options_);

  if (!options_.json) {
    if (flush_) {
      std::cout << std::endl;
    } else {
      std::cout << "\n";
    }
  }
  ++message_count_;
}

}  // namespace aos
