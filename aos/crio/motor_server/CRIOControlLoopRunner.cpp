#include "CRIOControlLoopRunner.h"

#include "aos/aos_core.h"
#include "aos/crio/shared_libs/interrupt_bridge.h"
#include "aos/crio/motor_server/MotorOutput.h"

using ::aos::control_loops::SerializableControlLoop;

namespace aos {
namespace crio {

bool CRIOControlLoopRunner::started_ = false;
std::vector<SerializableControlLoop *> CRIOControlLoopRunner::loops_;
Mutex CRIOControlLoopRunner::loops_lock;

void CRIOControlLoopRunner::Start() {
  if (started_) {
    LOG(WARNING, "not going to Start twice!!\n");
    return;
  }
  started_ = true;

  // TODO(aschuh): Hold on to a handle to this...
  (new WDInterruptNotifier<void>(Notify))->StartPeriodic(0.01);
}

void CRIOControlLoopRunner::AddControlLoop(SerializableControlLoop *loop) {
  MutexLocker control_loop_goals_locker(&loops_lock);
  loops_.push_back(loop);
  MotorServer::RegisterControlLoopGoal(loop);
}

void CRIOControlLoopRunner::Notify(void *) {
  // TODO(aschuh): Too many singletons/static classes!
  SensorOutputs::UpdateAll();
  // sensors get read first so it doesn't really matter if this takes a little bit
  {
    MutexLocker control_loop_goals_locker(
        &MotorServer::control_loop_goals_lock);
    for (auto it = loops_.begin(); it != loops_.end(); ++it) {
      (*it)->Iterate();
    }
  }
  MotorOutput::RunIterationAll();
  MotorServer::WriteOutputs();
}

}  // namespace crio
}  // namespace aos
