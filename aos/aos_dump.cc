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

namespace {

void PrintMessage(const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder) {
  // Print the flatbuffer out to stdout, both to remove the
  // unnecessary cruft from glog and to allow the user to readily
  // redirect just the logged output independent of any debugging
  // information on stderr.

  builder->Reset();
  aos::FlatbufferToJson(builder, channel->schema(),
                        static_cast<const uint8_t *>(context.data),
                        {false, static_cast<size_t>(FLAGS_max_vector_size)});

  if (context.monotonic_remote_time != context.monotonic_event_time) {
    std::cout << context.realtime_remote_time << " ("
              << context.monotonic_remote_time << ") delivered "
              << context.realtime_event_time << " ("
              << context.monotonic_event_time << "): " << *builder << '\n';
  } else {
    std::cout << context.realtime_event_time << " ("
              << context.monotonic_event_time << "): " << *builder << '\n';
  }
}

}  // namespace

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

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
  ::aos::ShmEventLoop event_loop(config_msg);
  event_loop.SkipTimingReport();
  event_loop.SkipAosLog();

  if (argc == 1) {
    std::cout << "Channels:\n";
    for (const aos::Channel *channel : *config_msg->channels()) {
      std::cout << channel->name()->c_str() << ' ' << channel->type()->c_str()
                << '\n';
    }
    return 0;
  }

  std::vector<const aos::Channel *> found_channels;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config_msg->channels();
  bool found_exact = false;
  for (const aos::Channel *channel : *channels) {
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

  aos::FastStringBuilder str_builder;

  for (const aos::Channel *channel : found_channels) {
    if (FLAGS_fetch) {
      const std::unique_ptr<aos::RawFetcher> fetcher =
          event_loop.MakeRawFetcher(channel);
      if (fetcher->Fetch()) {
        PrintMessage(channel, fetcher->context(), &str_builder);
      }
    }

    event_loop.MakeRawWatcher(
        channel, [channel, &str_builder](const aos::Context &context,
                                         const void * /*message*/) {
          PrintMessage(channel, context, &str_builder);
        });
  }

  event_loop.Run();

  return 0;
}
