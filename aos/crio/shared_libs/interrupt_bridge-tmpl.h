#include <sysLib.h>
#include <usrLib.h>
#include <sigevent.h>

#include "WPILib/Task.h"

#include "aos/aos_core.h"
#include "aos/crio/motor_server/MotorServer.h"
#include "aos/common/time.h"

extern "C" {
// A really simple function implemented in a .c file because the header that
// declares the variable doesn't work in C++.
// Returns the value of the global variable excExtendedVectors declared by
// <usrConfig.h>.
int aos_getExcExtendedVectors();
}

namespace aos {
namespace crio {

// Conceptually a TimerNotifier*[]. Used by the signal handler so that it can
// figure out which instance it's supposed to be using.
// Doesn't need a lock because a value is put in here before the signal handler
// is set up, so everything that might have concurrency issues will be reads
// which will happen after the set to each index.
// Declared like this instead of as an actual array because its size might be
// determined dynamically (SIGRTMAX and SIGRTMIN can be function calls).
extern void **const timer_notifiers;

template<typename T>
InterruptBridge<T>::InterruptBridge(Handler handler,
                              T *param, int priority)
    : handler_(handler),
      param_(param),
      task_(new Task("_NotifierLooper", reinterpret_cast<FUNCPTR>(HandlerLoop),
                     priority)),
      sem_(semBCreate(SEM_Q_PRIORITY, SEM_EMPTY)) {
  // Basically, makes sure that it does -mlongcall for calling interrupt handler
  // functions.
  // See <http://www.vxdev.com/docs/vx55man/vxworks/ppc/powerpc.html#91321> for
  // details about why this is important.
  if (!aos_getExcExtendedVectors()) {
    LOG(FATAL, "extended-call interrupts are not enabled\n");
  }
}

template<typename T>
InterruptBridge<T>::~InterruptBridge() {
  // Can't call Stop() because that calls StopNotifications(), which is virtual,
  // so you can't call it after the subclass's destructor has run.
  semDelete(sem_);
}

template<typename T>
void InterruptBridge<T>::StartTask() {
  // The inner cast to InterruptBridge<T>* is so that Loop knows what to
  // cast the void* to (the implementation of polymorphism means that
  // casting the pointer might actually change its value).
  task_->Start(reinterpret_cast<UINT32>(
          static_cast<InterruptBridge<T> *>(this)));
}

template<typename T>
void InterruptBridge<T>::Stop() {
  StopNotifications();
  task_->Stop();
}

template<typename T>
void InterruptBridge<T>::HandlerLoop(void *self_in) {
  InterruptBridge<T> *const self = static_cast<InterruptBridge<T> *>(self_in);
  while (true) {
    semTake(self->sem_, WAIT_FOREVER);
    self->handler_(self->param_);
  }
}

template<typename T>
PeriodicNotifier<T>::PeriodicNotifier(
    typename InterruptBridge<T>::Handler handler,
    T *param, int priority)
    : InterruptBridge<T>(handler, param, priority) {}

template<typename T>
void PeriodicNotifier<T>::StartPeriodic(double period) {
  this->StartTask();
  StartNotifications(period);
}

template<typename T>
TimerNotifier<T>::TimerNotifier(typename InterruptBridge<T>::Handler handler,
                                T *param, int unique, int priority)
    : PeriodicNotifier<T>(handler, param, priority),
      signal_(SIGRTMIN + unique) {
  timer_notifiers[signal_] = static_cast<void *>(this);

  struct sigaction sa;
  sa.sa_flags = 0;
  sa.sa_handler = StaticNotify;
  sigemptyset(&sa.sa_mask);
  if (sigaction(signal_, &sa, &old_sa_) != OK) {
    LOG(FATAL, "sigaction(%d, %p, %p) failed with %d: %s\n",
        signal_, &sa, &old_sa_, errno, strerror(errno));
  }

  sigevent event;
  event.sigev_notify = SIGEV_TASK_SIGNAL;
  event.sigev_signo = signal_;
  event.sigev_value.sival_ptr = this;
  if (timer_create(CLOCK_REALTIME, &event, &timer_) != OK) {
    LOG(FATAL, "timer_create(CLOCK_REALTIME, %p, %p) failed with %d: %s\n",
        &event, &timer_, errno, strerror(errno));
  }
}

template<typename T>
TimerNotifier<T>::~TimerNotifier() {
  if (timer_delete(timer_) != OK) {
    LOG(FATAL, "timer_delete(%p) failed with %d: %s\n", timer_,
        errno, strerror(errno));
  }
  if (sigaction(signal_, &old_sa_, NULL) != OK) {
    LOG(FATAL, "sigaction(%d, %p, NULL) failed with %d: %s\n",
        signal_, &old_sa_, errno, strerror(errno));
  }
  StopNotifications();
}

template<typename T>
void TimerNotifier<T>::StartNotifications(double period) {
  itimerspec timer_spec;
  timer_spec.it_value.tv_sec = 0;
  timer_spec.it_value.tv_nsec = 1;  // 0 would mean to disarm the timer
  timer_spec.it_interval = time::Time::InSeconds(period).ToTimespec();
  if (timer_settime(timer_, 0, &timer_spec, NULL) != OK) {
    LOG(FATAL, "timer_settime(%p, 0, %p, NULL) failed with %d: %s\n",
        timer_, &timer_spec, errno, strerror(errno));
  }
}

template<typename T>
void TimerNotifier<T>::StopNotifications() {
  timer_cancel(timer_);
}

template<typename T>
WDInterruptNotifier<T>::WDInterruptNotifier(
    typename InterruptBridge<T>::Handler handler, T *param, int priority)
    : PeriodicNotifier<T>(handler, param, priority), wd_(wdCreate()) {
  if (wd_ == NULL) {
    LOG(FATAL, "wdCreate() failed with %d: %s\n", errno, strerror(errno));
  }
}

template<typename T>
WDInterruptNotifier<T>::~WDInterruptNotifier() {
  if (wdDelete(wd_) != OK) {
    LOG(FATAL, "wdDelete(%p) failed with %d: %s\n",
        wd_, errno, strerror(errno));
  }
  StopNotifications();
}

template<typename T>
void WDInterruptNotifier<T>::StartNotifications(double period) {
  delay_ = time::Time::InSeconds(period).ToTicks();

  if (wdStart(wd_,
              1,  // run it really soon
              (FUNCPTR)StaticNotify,
              (UINT32)this) != OK) {
    LOG(FATAL, "wdStart(%p) failed with %d: %s", wd_, errno, strerror(errno));
  }
}

template<typename T>
void WDInterruptNotifier<T>::StopNotifications() {
  wdCancel(wd_);
}

// THESE GET CALLED AT INTERRUPT LEVEL. BE CAREFUL CHANGING THEM!!
//
// It might be tempting to use floating point math here, but DON'T!
//
// Also, do NOT use 64-bit integers (at least not without checking the assembly
// code again). GCC optimizes those using the floating point registers (see
// <http://compgroups.net/comp.os.vxworks/int64-where-gcc-ppc-and-vxworks-don-t-play-we/1110156>
// for an example of that being a problem).
//
// The assembly code comments are in there to make looking at assembly code
// dumps to verify that there aren't any floating point instructions easier.
// brians did that on 10/14/12 to his then-uncommited code and didn't find any.
// For future reference, here is a list of powerpc instruction mnemonics (what
// g++ -S gives) that do not involve any floating point:
//   lwz
//   lis
//   mflr
//   la
//   stwu
//   stw
//   mr
//   mtctr
//   bctrl
//   addi
//   mtlr
//   blr
//   cmpwi
//   bne
//   li
// NOTE: This macro is used in interrupt_notifier-tmpl.h too.
#define ASM_COMMENT(str) __asm__("# " str)
template<typename T>
void WDInterruptNotifier<T>::StaticNotify(void *self_in) {
  ASM_COMMENT("beginning of WDInterruptNotifier::StaticNotify");
  WDInterruptNotifier<T> *const self =
      static_cast<WDInterruptNotifier<T> *>(self_in);
  wdStart(self->wd_,
          self->delay_,
          (FUNCPTR)StaticNotify,
          (UINT32)self);  // call the same thing again
  self->Notify();
  ASM_COMMENT("end of WDInterruptNotifier::StaticNotify");
}

template<typename T>
void TimerNotifier<T>::StaticNotify(int signum) {
  ASM_COMMENT("beginning of TimerNotifier::StaticNotify");
  TimerNotifier<T> *const self =
      static_cast<TimerNotifier<T> *>(timer_notifiers[signum]);
  self->Notify();
  ASM_COMMENT("end of TimerNotifier::StaticNotify");
}

template<typename T>
void InterruptBridge<T>::Notify() {
  ASM_COMMENT("beginning of InterruptBridge::Notify");
  semGive(sem_);
  ASM_COMMENT("end of InterruptBridge::Notify");
}

}  // namespace crio
}  // namespace aos
