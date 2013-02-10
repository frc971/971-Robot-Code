// This is the core scheduling system
//
// Purposes: All shared memory gets allocated here.
//

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "aos/aos_core.h"

int main() {
  aos::InitCreate();
  select(0, NULL, NULL, NULL, NULL);  // wait forever
  aos::Cleanup();
}
