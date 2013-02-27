// This is the core scheduling system
//
// Purposes: All shared memory gets allocated here.
//

#include <sys/select.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

#include <string>

#include "aos/atom_code/init.h"

int main(int argc, char **argv) {
  aos::InitCreate();

  if (argc > 1) {
    if (system((std::string("touch '") + argv[1] + "'").c_str()) != 0) {
      fprintf(stderr, "`touch '%s'` failed", argv[1]);
      exit(EXIT_FAILURE);
    }
  }

  select(0, NULL, NULL, NULL, NULL);  // wait forever
  aos::Cleanup();
}
