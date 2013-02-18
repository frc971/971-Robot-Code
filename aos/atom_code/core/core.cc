// This is the core scheduling system
//
// Purposes: All shared memory gets allocated here.
//

#include <sys/select.h>
#include <stdlib.h>

#include "aos/atom_code/init.h"

int main() {
  aos::InitCreate();
  select(0, NULL, NULL, NULL, NULL);  // wait forever
  aos::Cleanup();
}
