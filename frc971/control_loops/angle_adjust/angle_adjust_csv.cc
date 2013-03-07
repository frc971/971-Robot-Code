#include "stdio.h"

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"

using ::frc971::control_loops::angle_adjust;
using ::aos::time::Time;

// Records data from the queue and stores it in a .csv file which can then
// be plotted/processed with relative ease.
int main(int argc, char * argv[]) {
  FILE *data_file = NULL;
  FILE *output_file = NULL;

  if (argc == 2) {
    data_file = fopen(argv[1], "w");
    output_file = data_file;
  } else {
    printf("Not saving to a CSV file.\n");
    output_file = stdout;
  }

  fprintf(data_file, "time, power, position");

  ::aos::Init();

  Time start_time = Time::Now();

  while (true) {
    ::aos::time::PhasedLoop10MS(2000);
    angle_adjust.goal.FetchLatest();
    angle_adjust.status.FetchLatest();
    angle_adjust.position.FetchLatest();
    angle_adjust.output.FetchLatest();
    if (angle_adjust.output.get() &&
        angle_adjust.position.get()) {
      fprintf(output_file, "\n%f, %f, %f",
              (angle_adjust.position->sent_time - start_time).ToSeconds(), 
              angle_adjust.output->voltage,
              angle_adjust.position->bottom_angle);
    }
  }

  if (data_file) {
    fclose(data_file);
  }

  ::aos::Cleanup();
  return 0;
}

