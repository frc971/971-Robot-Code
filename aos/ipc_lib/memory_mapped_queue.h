#ifndef AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
#define AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_

#include "absl/types/span.h"

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"

namespace aos {
namespace ipc_lib {

std::string ShmFolder(std::string_view shm_base, const Channel *channel);

std::string ShmPath(std::string_view shm_base, const Channel *channel);

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel);

class MemoryMappedQueue {
 public:
  MemoryMappedQueue(std::string_view shm_base, uint32_t permissions,
                    const Configuration *config, const Channel *channel);
  ~MemoryMappedQueue();

  // This class can't be default or copy constructed.
  MemoryMappedQueue() = delete;
  MemoryMappedQueue(const MemoryMappedQueue &other) = delete;
  MemoryMappedQueue &operator=(const MemoryMappedQueue &rhs) = delete;

  LocklessQueueMemory *memory() const {
    return reinterpret_cast<ipc_lib::LocklessQueueMemory *>(data_);
  }

  const LocklessQueueMemory *const_memory() const {
    return reinterpret_cast<const LocklessQueueMemory *>(const_data_);
  }

  const LocklessQueueConfiguration &config() const { return config_; }

  LocklessQueue queue() const {
    return LocklessQueue(const_memory(), memory(), config());
  }

  absl::Span<char> GetMutableSharedMemory() const {
    return absl::Span<char>(static_cast<char *>(data_), size_);
  }

  absl::Span<const char> GetConstSharedMemory() const {
    return absl::Span<const char>(static_cast<const char *>(const_data_),
                                  size_);
  }

 private:
  const LocklessQueueConfiguration config_;

  size_t size_;
  void *data_;
  const void *const_data_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  //  AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
