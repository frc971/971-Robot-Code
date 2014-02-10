#include <sys/select.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>

#include <string>

#include "aos/linux_code/init.h"

// Initializes shared memory. This is the only file that will create the shared
// memory file if it doesn't already exist (and set everything up).
//
// Will also create the file given as a first argument.

int main(int argc, char **argv) {
  aos::InitCreate();

  if (argc > 1) {
    if (system((std::string("touch '") + argv[1] + "'").c_str()) != 0) {
      fprintf(stderr, "`touch '%s'` failed\n", argv[1]);
      exit(EXIT_FAILURE);
    }
  }

  select(0, NULL, NULL, NULL, NULL);  // wait forever
  aos::Cleanup();
}
