#include "aos/testing/prevent_exit.h"

#include <stdlib.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace testing {
namespace {

void TerminateExitHandler() {
  _exit(EXIT_SUCCESS);
}

}  // namespace

void PreventExit() {
  CHECK_EQ(atexit(TerminateExitHandler), 0);
}

}  // namespace testing
}  // namespace aos
