#ifndef AOS_CRIO_MOTOR_SERVER_CRIO_CONTROL_LOOP_RUNNER_H_
#define AOS_CRIO_MOTOR_SERVER_CRIO_CONTROL_LOOP_RUNNER_H_

#include <vector>
#include <semLib.h>

#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/mutex.h"

namespace aos {
namespace crio {

// Runs crio-side control loops. Completely static because there is no reason
// for multiple ones and it gets rid of the problem of passing an instance
// around.
class CRIOControlLoopRunner {
 public:
  // Spawns a new Task that loops forever.
  // No other functions should be called before this one returns.
  static void Start();

  // Adds a control loop to run.
  // This class takes control of the instance.
  static void AddControlLoop(control_loops::SerializableControlLoop *loop);

 private:
  static bool started_;

  static std::vector<control_loops::SerializableControlLoop *> loops_;
  static Mutex loops_lock;

  // Gets called by a WDInterruptNotifier on 0.01 second intervals.
  static void Notify(void *);
};


}  // namespace crio
}  // namespace aos

#endif
