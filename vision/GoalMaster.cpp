#include "math.h"

#include "aos/common/time.h"
#include "aos/atom_code/init.h"

#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/CameraTarget.q.h"

#include "vision/RingBuffer.h"

using frc971::vision::RingBuffer;
using frc971::sensors::gyro;
using frc971::vision::targets;
using frc971::vision::target_angle;

int main() {
  RingBuffer< ::aos::time::Time, double> buff;
  ::aos::InitNRT();
  while (true) {
    gyro.FetchNextBlocking();
    buff.Sample(gyro->sent_time, gyro->angle);
    if (targets.FetchNext()) {
      ::aos::time::Time stamp = ::aos::time::Time::InNS(targets->timestamp);
      double angle_goal =
          buff.ValueAt(stamp) -
					M_PI / 2.0 * targets->percent_azimuth_off_center / 2.0;
      printf("%g ",angle_goal);
      printf("%g\n",gyro->angle);

      target_angle.MakeWithBuilder()
          .target_angle(angle_goal).Send();
    }
  }
  ::aos::Cleanup();
}
