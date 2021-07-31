#include "aos/testing/prevent_exit.h"

#include <unistd.h>

#include <cstdlib>

#include "glog/logging.h"

namespace aos {
namespace testing {
namespace {

void TerminateExitHandler() { _exit(EXIT_SUCCESS); }

}  // namespace

void PreventExit() { CHECK_EQ(atexit(TerminateExitHandler), 0); }

}  // namespace testing
}  // namespace aos
