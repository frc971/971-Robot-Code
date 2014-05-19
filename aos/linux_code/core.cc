#include <sys/wait.h>
#include <sys/select.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>

#include <string>

#include "aos/linux_code/init.h"
#include "aos/common/util/run_command.h"

// Initializes shared memory. This is the only file that will create the shared
// memory file if it doesn't already exist (and set everything up).
//
// Will also touch the file given as a first argument.

int main(int argc, char **argv) {
  aos::InitCreate();

  if (argc > 1) {
    const int result = ::aos::util::RunCommand(
        (std::string("touch '") + argv[1] + "'").c_str());
    if (result == -1 || !WIFEXITED(result) || WEXITSTATUS(result) != 0) {
      fprintf(stderr, "`touch '%s'` failed; result = %x\n", argv[1], result);
      exit(EXIT_FAILURE);
    }
  }

  select(0, NULL, NULL, NULL, NULL);  // wait forever
  aos::Cleanup();
}
