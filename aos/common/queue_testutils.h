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

}  // namespace testing
}  // namespace common
}  // namespace aos
