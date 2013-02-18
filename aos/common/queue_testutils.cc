#include "aos/common/queue_testutils.h"

#include <string.h>

#include "aos/common/queue.h"

namespace aos {
namespace common {
namespace testing {

GlobalCoreInstance::GlobalCoreInstance() {
  const size_t kCoreSize = 0x100000;
  global_core = &global_core_data_;
  global_core->owner = 1;
  void *memory = malloc(kCoreSize);
  assert(memory != NULL);
  memset(memory, 0, kCoreSize);

  assert(aos_core_use_address_as_shared_mem(memory, kCoreSize) == 0);
}

GlobalCoreInstance::~GlobalCoreInstance() {
  free(global_core->mem_struct);
  global_core = NULL;
}

}  // namespace testing
}  // namespace common
}  // namespace aos
