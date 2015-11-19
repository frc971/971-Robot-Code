#include "aos/common/controls/replay_control_loop.h"
#include "aos/linux_code/init.h"

#include "y2014_bot3/control_loops/drivetrain/drivetrain.q.h"

// Reads one or more log files and sends out all the queue messages (in the
// correct order and at the correct time) to feed a "live" drivetrain process.

int main(int argc, char **argv) {
  if (argc <= 1) {
    fprintf(stderr, "Need at least one file to replay!\n");
    return EXIT_FAILURE;
  }

  ::aos::InitNRT();

  ::aos::controls::ControlLoopReplayer<::y2014_bot3::control_loops::DrivetrainQueue>
      replayer(&::y2014_bot3::control_loops::drivetrain_queue, "drivetrain");
  for (int i = 1; i < argc; ++i) {
    replayer.ProcessFile(argv[i]);
  }

  ::aos::Cleanup();
}
