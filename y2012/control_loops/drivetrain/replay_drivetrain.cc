#include "aos/common/controls/replay_control_loop.h"
#include "aos/linux_code/init.h"

#include "y2012/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro.q.h"

// Reads one or more log files and sends out all the queue messages (in the
// correct order and at the correct time) to feed a "live" drivetrain process.

int main(int argc, char **argv) {
  if (argc <= 1) {
    fprintf(stderr, "Need at least one file to replay!\n");
    return EXIT_FAILURE;
  }

  ::aos::InitNRT();

  {
    ::aos::controls::ControlLoopReplayer<
        ::y2012::control_loops::DrivetrainQueue>
        replayer(&::y2012::control_loops::drivetrain_queue, "drivetrain");

    replayer.AddDirectQueueSender("wpilib_interface.Gyro", "sending",
                                  ::frc971::sensors::gyro_reading);
    for (int i = 1; i < argc; ++i) {
      replayer.ProcessFile(argv[i]);
    }
  }
  ::frc971::sensors::gyro_reading.Clear();
  ::y2012::control_loops::drivetrain_queue.goal.Clear();
  ::y2012::control_loops::drivetrain_queue.status.Clear();
  ::y2012::control_loops::drivetrain_queue.position.Clear();
  ::y2012::control_loops::drivetrain_queue.output.Clear();

  ::aos::Cleanup();
}
