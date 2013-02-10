#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "aos/aos_core.h"

#include "frc971/queues/GyroAngle.q.h"

#define M_PI 3.14159265358979323846264338327

using frc971::sensors::gyro;

int main(){
  aos::Init();
  int fd = open("/dev/aschuh0", O_RDONLY);
  int rate_limit = 0;
  if (fd < 0) {
    LOG(ERROR, "No Gyro found.\n");
  } else {
    LOG(INFO, "Gyro now connected\n");
  }

  while (true) {
    int64_t gyro_value;
    if (read(fd, (void *)&gyro_value, sizeof(gyro_value)) != sizeof(gyro_value)) {
      LOG(ERROR, "Could not read gyro errno: %d\n", errno);
      if (errno == ENODEV || errno == EBADF) {
        close(fd);
        while (1) {
          usleep(1000);
          fd = open("/dev/aschuh0", O_RDONLY);
          if (fd > 0) {
            LOG(INFO, "Found gyro again\n");
            break;
          }
        }
      }
      continue;
    }
    rate_limit ++;
    if (rate_limit > 10) {
      LOG(DEBUG, "Gyro is %d\n", (int)(gyro_value / 16));
      rate_limit = 0;
    }
    gyro.MakeWithBuilder().angle(gyro_value / 16.0 / 1000.0 / 180.0 * M_PI).Send();
  }

  aos::Cleanup();
}
