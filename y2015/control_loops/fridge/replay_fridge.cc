#include "aos/common/controls/replay_control_loop.h"
#include "aos/linux_code/init.h"

#include "y2015/control_loops/fridge/fridge.q.h"

// Reads one or more log files and sends out all the queue messages (in the
// correct order and at the correct time) to feed a "live" fridge process.

int main(int argc, char **argv) {
  if (argc <= 1) {
    fprintf(stderr, "Need at least one file to replay!\n");
    return EXIT_FAILURE;
  }

  ::aos::InitNRT();

  ::aos::controls::ControlLoopReplayer<
      ::y2015::control_loops::fridge::FridgeQueue>
      replayer(&::y2015::control_loops::fridge::fridge_queue, "fridge");
  for (int i = 1; i < argc; ++i) {
    replayer.ProcessFile(argv[i]);
  }

  ::aos::Cleanup();
}
