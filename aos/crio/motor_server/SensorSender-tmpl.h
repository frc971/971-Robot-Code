#include "WPILib/Task.h"
#include "WPILib/Timer.h"

#include "aos/crio/motor_server/SensorOutput.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/Configuration.h"

namespace aos {

template<class Values> void SensorSender<Values>::Run() {
  SendSocket sock(NetworkPort::kSensors,
                  configuration::GetIPAddress(configuration::NetworkDevice::kAtom));
  Values vals;
  while (true) {
    Wait(0.0015);
    SensorOutput<Values>::RunIterationAll(vals);
    sock.Send(&vals, sizeof(vals));
  }
}

} // namespace aos

