#include "aos/atom_code/output/MotorOutput.h"

#include <math.h>

#include "aos/common/Configuration.h"
#include "aos/aos_core.h"
#include "aos/common/byteorder.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/atom_code/messages/DriverStationDisplay.h"

namespace aos {

MotorOutput::MotorOutput() : sock(NetworkPort::kMotors,
          configuration::GetIPAddress(configuration::NetworkDevice::kCRIO)) {}

void MotorOutput::Run() {
  while (true) {
    time::PhasedLoopXMS(5, 1000);
    RunIteration();

    // doesn't matter if multiple instances end up running this loop at the same
    // time because the queue handles the race conditions
    while (true) {
      const aos::DriverStationDisplay *line = aos::DriverStationDisplay::GetNext();
      if (line != NULL) {
        AddDSLine(line->line, line->data);
        LOG(DEBUG, "got a line %hhd that said '%s'\n", line->line, line->data);
        aos::DriverStationDisplay::Free(line);
      } else {
        break;
      }
    }

    if (sock.SendHoldMsg() == -1) {
      LOG(WARNING, "sending outputs failed\n");
      continue;
    } else {
      LOG(DEBUG, "sent outputs\n");
    }
  }
}

int MotorOutput::AddMotor(char type, uint8_t num, float value) {
  if (sock.hold_msg_len_ + 4 > sock.MAX_MSG) {
    return -1;
  }
  sock.hold_msg_[sock.hold_msg_len_ ++] = type;
  sock.hold_msg_[sock.hold_msg_len_ ++] = num;
  to_network(&value, &sock.hold_msg_[sock.hold_msg_len_]);
  sock.hold_msg_len_ += 4;
  return 0;
}
int MotorOutput::AddSolenoid(uint8_t port, bool on) {
  if (sock.hold_msg_len_ + 3 > sock.MAX_MSG) {
    return -1;
  }
  sock.hold_msg_[sock.hold_msg_len_ ++] = 's';
  sock.hold_msg_[sock.hold_msg_len_ ++] = port;
  sock.hold_msg_[sock.hold_msg_len_ ++] = on ? 1 : 0;
  return 0;
}

int MotorOutput::AddDSLine(uint8_t line, const char *data) {
  size_t data_len = strlen(data); // doesn't include terminating NULL
  if (sock.hold_msg_len_ + 3 + data_len > sock.MAX_MSG) {
    return -1;
  }

  sock.hold_msg_[sock.hold_msg_len_ ++] = 'd';
  sock.hold_msg_[sock.hold_msg_len_ ++] = line;
  sock.hold_msg_[sock.hold_msg_len_ ++] = data_len;
  memcpy(&sock.hold_msg_[sock.hold_msg_len_], data, data_len);
  sock.hold_msg_len_ += data_len;
  return 0;
}

} // namespace aos

