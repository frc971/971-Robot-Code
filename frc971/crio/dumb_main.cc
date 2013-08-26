#include "WPILib/NetworkRobot/NetworkRobot.h"
#include "WPILib/RobotBase.h"

#include "aos/common/network_port.h"
#include "aos/crio/ip.h"
#include "aos/common/util.h"

using ::aos::util::MakeIPAddress;
using ::aos::util::GetOwnIPAddress;

namespace frc971 {

class MyRobot : public NetworkRobot {
 public:
  MyRobot() : NetworkRobot(static_cast<uint16_t>(::aos::NetworkPort::kMotors),
                           ::MakeIPAddress(::GetOwnIPAddress(),
                               ::aos::NetworkAddress::kAtom),
                           static_cast<uint16_t>(::aos::NetworkPort::kDS),
                           ::MakeIPAddress(::GetOwnIPAddress(),
                               ::aos::NetworkAddress::kAtom)) {}
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
