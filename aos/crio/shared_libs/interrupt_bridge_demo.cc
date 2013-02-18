#include "aos/crio/shared_libs/interrupt_bridge.h"

#include <unistd.h>

#include "WPILib/Timer.h"

#include "aos/common/time.h"

using aos::time::Time;

namespace aos {
namespace crio {

class InterruptBridgeTest {
 public:
  void Run(double seconds) {
    int interruptVal = 1;
    WDInterruptNotifier<int> interrupt(Notify, &interruptVal);
    interrupt.StartPeriodic(0.01);
    int timerVal = 2;
    TimerNotifier<int> timer(Notify, &timerVal);
    time::SleepFor(Time::InSeconds(0.005));  // get them offset by ~180 degrees
    timer.StartPeriodic(0.01);
    time::SleepFor(Time::InSeconds(seconds));
  }
 private:
  static void Notify(int *value) {
    printf("notified %f,%d\n", Timer::GetFPGATimestamp(), *value);
  }
};

}  // namespace crio
}  // namespace aos

// Designed to be called from the vxworks shell if somebody wants to run this.
extern "C" int interrupt_bridge_demo(int seconds) {
  if (seconds == 0) {
    printf("arguments: number of seconds\n");
    return -1;
  }
  aos::crio::InterruptBridgeTest runner;
  runner.Run(seconds);
  return 0;
}
