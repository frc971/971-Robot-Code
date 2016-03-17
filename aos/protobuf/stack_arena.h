#ifndef AOS_PROTOBUF_STACK_ARENA_H_
#define AOS_PROTOBUF_STACK_ARENA_H_
#include "google/protobuf/arena.h"

namespace aos {
namespace protobuf {

void* FatalArenaBlockAlloc(size_t);

void FatalArenaBlockDealloc(void*, size_t);

// This class manages a protobuf arena which uses an internal buffer
// (allocated as part of the object) of size buffer_size.
//
// Protos allocated from this arena must not use more than buffer_size
// worth of data.
//
// Also worth noting is that sizeof(google::protobuf::Arena::Block)
// is used up out of the buffer for internal protobuf related usage, so
// overallocate accordingly.
template <size_t buffer_size>
class StackProtoArena {
 public:
  StackProtoArena() :
      arena_(GetArenaOptions(&data_[0])) {}

  google::protobuf::Arena* arena() { return &arena_; }

  // For convienence:
  template <typename T>
  T* CreateMessage() {
    return google::protobuf::Arena::CreateMessage<T>(&arena_);
  }
 private:
  static google::protobuf::ArenaOptions GetArenaOptions(char* data) {
    // Expecting RVO to kick in.
    google::protobuf::ArenaOptions options;
    options.initial_block = data;
    options.initial_block_size = buffer_size;
    options.block_alloc = &FatalArenaBlockAlloc;
    options.block_dealloc = FatalArenaBlockDealloc;
    return options;
  }

  char data_[buffer_size];
  google::protobuf::Arena arena_;
};

}  // namespace protobuf
}  // namespace aos

#endif  // AOS_PROTOBUF_STACK_ARENA_H_
