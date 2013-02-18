#ifndef AOS_CRIO_SHARED_LIBS_INTERRUPT_BRIDGE_H_
#define AOS_CRIO_SHARED_LIBS_INTERRUPT_BRIDGE_H_

#include <wdLib.h>
#include <semLib.h>
#include <timers.h>
#include <signal.h>

#include "aos/common/scoped_ptr.h"

#include "aos/common/macros.h"

class Task;

namespace aos {
namespace crio {

// Handles creating a separate Task at a high priority with a semaphore to
// link it to something else that provides timing information (likely in an ISR
// context).
template<typename T>
class InterruptBridge {
 public:
  // The default priority. Here as a constant for subclasses to use as a default
  // too.
  static const int kDefaultPriority = 96;
  typedef void (*Handler)(T *param);

  // Stops calling the handler at all until a Start function is called again.
  void Stop();

 protected:
  // < 95 priority does not work, and < 100 isn't guaranteed to work
  InterruptBridge(Handler handler, T *param, int priority);
  // Subclasses should call StopNotifications.
  virtual ~InterruptBridge();

  // Subclasses should call this whenever the Notifier triggers.
  // It is safe to call from an ISR.
  void Notify();
  // Starts the actual Task.
  void StartTask();

 private:
  const Handler handler_;
  T *const param_;

  // Subclasses should do whatever they have to to stop calling Notify().
  virtual void StopNotifications() = 0;

  // The function that the Task runs.
  // Loops forever, waiting for sem_ and then calling handler_.
  static void HandlerLoop(void *self_in);
  const scoped_ptr<Task> task_;
  // For synchronizing between the Task and Notify().
  SEM_ID sem_;
  DISALLOW_COPY_AND_ASSIGN(InterruptBridge<T>);
};

// An accurate version of WPILib's Notifier class.
template<typename T>
class PeriodicNotifier : public InterruptBridge<T> {
 public:
  // Period is how much (in seconds) to wait between running the handler.
  void StartPeriodic(double period);

 protected:
  PeriodicNotifier(typename InterruptBridge<T>::Handler handler, T *param,
                   int priority);
  virtual ~PeriodicNotifier() {}

 private:
  virtual void StopNotifications() = 0;
  // Subclasses should do whatever they have to to start calling Notify() every
  // period seconds.
  virtual void StartNotifications(double period) = 0;
};

// This one works accurately, but it has the potential to drift over time.
// It has only sysClockRateGet() resolution.
template<typename T>
class WDInterruptNotifier : public PeriodicNotifier<T> {
 public:
  WDInterruptNotifier(typename InterruptBridge<T>::Handler handler,
                      T *param = NULL,
                      int priority = InterruptBridge<T>::kDefaultPriority);
  virtual ~WDInterruptNotifier();

 private:
  // The argument is the general callback parameter which will be a pointer to
  // an instance. This function calls Notify() on that instance.
  static void StaticNotify(void *self_in);
  virtual void StopNotifications();
  virtual void StartNotifications(double period);

  WDOG_ID wd_;
  int delay_;  // what to pass to wdStart
  DISALLOW_COPY_AND_ASSIGN(WDInterruptNotifier<T>);
};

// Vxworks (really really) should take care of making sure that this one
// doesn't drift over time, but IT DOESN'T!! Brian found the implementation
// online (file timerLib.c), and it's just doing the same thing as
// WDInterruptNotifier, except with extra conversions and overhead to implement
// the POSIX standard. There really is no reason to use this.
// It has only sysClockRateGet() resolution.
template<typename T>
class TimerNotifier : public PeriodicNotifier<T> {
 public:
  // unique should be different for all instances created in the same Task. It
  // can range from 0 to (SIGRTMAX - SIGRTMIN). This instance will use the
  // signal (SIGRTMIN + unique), so nothing else should use that signal.
  TimerNotifier(typename InterruptBridge<T>::Handler handler,
                T *param = NULL,
                int unique = 0,
                int priority = InterruptBridge<T>::kDefaultPriority);
  virtual ~TimerNotifier();

 private:
  // The first argument is the signal number that is being triggered. This
  // function looks up a pointer to an instance in timer_notifiers using that
  // and calls Notify() on that instance.
  static void StaticNotify(int signum);
  virtual void StopNotifications();
  virtual void StartNotifications(double period);

  timer_t timer_;
  // Which signal timer_ will notify on.
  const int signal_;
  // What the action for signal_ was before we started messing with it.
  struct sigaction old_sa_;
  DISALLOW_COPY_AND_ASSIGN(TimerNotifier<T>);
};

}  // namespace crio
}  // namespace aos

#include "aos/crio/shared_libs/interrupt_bridge-tmpl.h"

#endif  // AOS_CRIO_SHARED_LIBS_INTERRUPT_BRIDGE_H_
