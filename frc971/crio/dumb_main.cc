#include "WPILib/NetworkRobot/NetworkRobot.h"
#include "WPILib/RobotBase.h"
#include "aos/common/Configuration.h"

namespace frc971 {

class MyRobot : public NetworkRobot {
 public:
  MyRobot() : NetworkRobot(static_cast<uint16_t>(::aos::NetworkPort::kMotors),
                           ::aos::configuration::GetIPAddress(
                               ::aos::configuration::NetworkDevice::kAtom)) {}
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
