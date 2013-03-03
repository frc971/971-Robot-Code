#include "stdio.h"

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"

using ::frc971::control_loops::shooter;
using ::aos::time::Time;

int main(int argc, char * argv[]) {
  FILE *data_file = NULL;
  FILE *output_file = NULL;
  fprintf(data_file, "time, power, position");

  if (argc != 2) {
    printf("Not saving to a CSV file.\n");
    data_file = fopen(argv[1], "w");
    fprintf(data_file, "time, power, position");
    output_file = data_file;
  } else {
    output_file = stdout;
  }

  ::aos::Init();

  Time start_time = Time::Now();

  while (true) {
    ::aos::time::PhasedLoop10MS(2000);
    shooter.goal.FetchLatest();
    shooter.status.FetchLatest();
    shooter.position.FetchLatest();
    shooter.output.FetchLatest();
    if (shooter.output.get() &&
        shooter.position.get()) {
      fprintf(output_file, "\n%f, %f, %f",
              (shooter.position->sent_time - start_time).ToSeconds(), 
              shooter.output->voltage,
              shooter.position->position);
    }
  }

  if (data_file) {
    fclose(data_file);
  }

  ::aos::Cleanup();
  return 0;
}

