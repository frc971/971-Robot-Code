#include "aos/testing/test_shm.h"

#include <sys/mman.h>

#include "aos/common/queue.h"
#include "aos/common/logging/logging.h"
#include "aos/testing/test_logging.h"

namespace aos {
namespace testing {
namespace {

const size_t kCoreSize = 0x100000;

}  // namespace

TestSharedMemory::TestSharedMemory() {
  global_core = &global_core_data_;
  global_core->owner = true;
  // Use mmap(2) manually instead of through malloc(3) so that we can pass
  // MAP_SHARED which allows forked processes to communicate using the
  // "shared" memory.
  void *memory = mmap(NULL, kCoreSize, PROT_READ | PROT_WRITE,
                      MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  CHECK_NE(memory, MAP_FAILED);

  aos_core_use_address_as_shared_mem(memory, kCoreSize);

  ::aos::testing::EnableTestLogging();
}

TestSharedMemory::~TestSharedMemory() {
  PCHECK(munmap(global_core->mem_struct, kCoreSize));
  global_core = NULL;
}

}  // namespace testing
}  // namespace aos
