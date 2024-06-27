#include "aos/ipc_lib/memory_estimation.h"

#include "aos/ipc_lib/memory_mapped_queue.h"
namespace aos::ipc_lib {
size_t TotalSharedMemoryUsage(const aos::Configuration *config,
                              const aos::Node *node) {
  size_t total_size = 0;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config->channels();
  CHECK(channels != nullptr);
  for (const aos::Channel *channel : *channels) {
    if (aos::configuration::ChannelIsReadableOnNode(channel, node)) {
      total_size +=
          LocklessQueueMemorySize(MakeQueueConfiguration(config, channel));
    }
  }
  return total_size;
}
}  // namespace aos::ipc_lib
