#include "aos/controls/replay_control_loop.h"
#include "aos/init.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro.q.h"

// Reads one or more log files and sends out all the queue messages (in the
// correct order and at the correct time) to feed a "live" drivetrain process.

int main(int argc, char **argv) {
  if (argc <= 1) {
    fprintf(stderr, "Need at least one file to replay!\n");
    return EXIT_FAILURE;
  }

  ::aos::InitNRT();

  ::frc971::control_loops::DrivetrainQueue drivetrain_queue(
      ".frc971.control_loops.drivetrain_queue",
      ".frc971.control_loops.drivetrain_queue.goal",
      ".frc971.control_loops.drivetrain_queue.position",
      ".frc971.control_loops.drivetrain_queue.output",
      ".frc971.control_loops.drivetrain_queue.status");

  {
    ::aos::controls::ControlLoopReplayer<
        ::frc971::control_loops::DrivetrainQueue>
        replayer(&drivetrain_queue, "drivetrain");

    replayer.AddDirectQueueSender<::frc971::sensors::GyroReading>(
        "wpilib_interface.Gyro", "sending", ".frc971.sensors.gyro_reading");
    for (int i = 1; i < argc; ++i) {
      replayer.ProcessFile(argv[i]);
    }
  }

  ::aos::Cleanup();
}
