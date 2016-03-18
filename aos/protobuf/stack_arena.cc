#include "aos/common/logging/logging.h"

namespace aos {
namespace protobuf {

void FatalArenaBlockAlloc(size_t) {
  LOG(FATAL, "trying to allocate in arena code");
}

void FatalArenaBlockDealloc(void*, size_t) {
  LOG(FATAL, "trying to deallocate in arena code");
}

}  // namespace protobuf
}  // namespace aos
