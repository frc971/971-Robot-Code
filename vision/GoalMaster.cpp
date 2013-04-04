#include "math.h"

#include "aos/common/time.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"

#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/CameraTarget.q.h"

#include "vision/RingBuffer.h"
#include "vision/SensorProcessor.h"

using ::frc971::vision::RingBuffer;
using ::frc971::sensors::gyro;
using ::frc971::vision::targets;
using ::frc971::vision::target_angle;
using ::frc971::kPixelsToMeters;
using ::frc971::kMetersToShooterSpeeds;
using ::frc971::kMetersToShooterAngles;
using ::frc971::interpolate;

int main() {
  //RingBuffer< ::aos::time::Time, double> buff;
  ::aos::InitNRT();
  while (true) {
    //gyro.FetchNextBlocking();
    //buff.Sample(gyro->sent_time, gyro->angle);
    if (targets.FetchNext()) {
      /*::aos::time::Time stamp = ::aos::time::Time::InNS(targets->timestamp);
      double angle_goal =
          buff.ValueAt(stamp) -
					M_PI / 2.0 * targets->percent_azimuth_off_center / 2.0;
      printf("%g ",angle_goal);
      printf("%g\n",gyro->angle);*/

      double meters = interpolate(
          sizeof(kPixelsToMeters) / sizeof(kPixelsToMeters[0]),
          kPixelsToMeters,
          targets->percent_elevation_off_center);
      const double shooter_speed = interpolate(
          sizeof(kMetersToShooterSpeeds) / sizeof(kMetersToShooterSpeeds[0]),
          kMetersToShooterSpeeds,
          meters);
      const double shooter_angle = interpolate(
           sizeof(kMetersToShooterAngles) / sizeof(kMetersToShooterAngles[0]),
           kMetersToShooterAngles,
           meters);

      LOG(DEBUG, "think target is %f meters away Speed %f Angle %f\n",
          meters, shooter_speed, shooter_angle);

      target_angle.MakeWithBuilder()
          /*.target_angle(angle_goal)*/
          .shooter_speed(shooter_speed)
          .shooter_angle(shooter_angle)
          .Send();
    }
  }
  ::aos::Cleanup();
}
