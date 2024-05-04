#ifndef Y2024_SWERVE_SWERVE_PUBLISHER_H_
#define Y2024_SWERVE_SWERVE_PUBLISHER_H_

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/drivetrain/swerve/swerve_drivetrain_output_generated.h"

namespace y2024_swerve {

namespace drivetrain = frc971::control_loops::drivetrain;

class SwervePublisher {
 public:
  SwervePublisher(aos::EventLoop *event_loop, aos::ExitHandle *exit_handle,
                  const std::string &filename, double duration);

 private:
  aos::Sender<frc971::control_loops::drivetrain::swerve::Output>
      drivetrain_output_sender_;
};

}  // namespace y2024_swerve

#endif  // Y2024_SWERVE_SWERVE_PUBLISHER_H_
