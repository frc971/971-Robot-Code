#include "aos/common/queue.h"

namespace aos {
namespace common {
namespace testing {

class GlobalCoreInstance {
 public:
  GlobalCoreInstance();
  ~GlobalCoreInstance();

 private:
  struct aos_core global_core_data_;
};

// Enables the logging framework for use during a gtest test.
// It will print out all WARNING and above messages all of the time. It will
// also print out all log messages when a test fails.
// This function only needs to be called once in each process (after gtest is
// initialized), however it can be called more than that.
void EnableTestLogging();

}  // namespace testing
}  // namespace common
}  // namespace aos
