#include "aos/common/messages/robot_state.q.h"
#include "aos/linux_code/init.h"

using ::aos::time::Time;
using ::aos::joystick_state;

int main() {
  ::aos::InitNRT();

  const Time start = Time::Now();

  while (true) {
    auto message = joystick_state.MakeMessage();
    message->Zero();
    if ((Time::Now() - start) < Time::InSeconds(0.5)) {
      message->joysticks[3].pov = -1;
    } else if ((Time::Now() - start) < Time::InSeconds(0.65)) {
      message->joysticks[3].pov = 180;
    } else if ((Time::Now() - start) < Time::InSeconds(1.0)) {
      message->joysticks[3].pov = -1;
    } else {
      break;
    }
    message->fake = true;
    CHECK(message.Send());
    ::aos::time::SleepFor(Time::InSeconds(0.02));
  }
}
