#include "y2024/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2024/constants/constants_generated.h"

namespace y2024::control_loops::drivetrain {

const frc971::control_loops::drivetrain::DrivetrainConfig<double>
GetDrivetrainConfig(aos::EventLoop *event_loop) {
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher(event_loop);
  return frc971::control_loops::drivetrain::DrivetrainConfig<double>::
      FromFlatbuffer(
          *CHECK_NOTNULL(constants_fetcher.constants().common()->drivetrain()));
};

}  // namespace y2024::control_loops::drivetrain
