#include "aos/testing/prevent_exit.h"

#include <unistd.h>

#include <cstdlib>

#include "absl/log/check.h"
#include "absl/log/log.h"

namespace aos::testing {
namespace {

void TerminateExitHandler() { _exit(EXIT_SUCCESS); }

}  // namespace

void PreventExit() { CHECK_EQ(atexit(TerminateExitHandler), 0); }

}  // namespace aos::testing
