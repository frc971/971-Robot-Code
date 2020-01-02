#include <iostream>
#include <map>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");

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

  if (argc == 1) {
    std::cout << "Channels:\n";
    for (const aos::Channel *channel : *config_msg->channels()) {
      std::cout << channel->name()->c_str() << ' ' << channel->type()->c_str()
                << '\n';
    }
    return 0;
  }

  int found_channels = 0;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config_msg->channels();
  for (const aos::Channel *channel : *channels) {
    if (channel->name()->c_str() == channel_name &&
        channel->type()->str().find(message_type) != std::string::npos) {
      event_loop.MakeRawWatcher(
          channel, [channel](const aos::Context &context, const void *message) {
            // Print the flatbuffer out to stdout, both to remove the
            // unnecessary cruft from glog and to allow the user to readily
            // redirect just the logged output independent of any debugging
            // information on stderr.
            if (context.monotonic_remote_time != context.monotonic_event_time) {
              std::cout << context.realtime_remote_time << " ("
                        << context.monotonic_remote_time << ") delivered "
                        << context.realtime_event_time << " ("
                        << context.monotonic_event_time << "): "
                        << aos::FlatbufferToJson(
                               channel->schema(),
                               static_cast<const uint8_t *>(message))
                        << '\n';
            } else {
              std::cout << context.realtime_event_time << " ("
                        << context.monotonic_event_time << "): "
                        << aos::FlatbufferToJson(
                               channel->schema(),
                               static_cast<const uint8_t *>(message))
                        << '\n';
            }
          });
      found_channels++;
    }
  }

  if (found_channels == 0) {
    LOG(FATAL) << "Could not find any channels with the given name and type.";
  } else if (found_channels > 1 && message_type.size() != 0) {
    LOG(FATAL) << "Multiple channels found with same type";
  }

  event_loop.Run();
  ::aos::Cleanup();
  return 0;
}
