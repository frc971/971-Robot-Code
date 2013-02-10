#include <intLib.h>
#include <logLib.h>

namespace aos {
namespace crio {

template<typename T>
InterruptNotifier<T>::InterruptNotifier(
    typename InterruptBridge<T>::Handler handler,
    InterruptableSensorBase *sensor, T *param, int priority)
    : InterruptBridge<T>(handler, param, priority), sensor_(sensor) {
  sensor_->RequestInterrupts(StaticNotify, this);
}

template<typename T>
InterruptNotifier<T>::~InterruptNotifier() {
  sensor_->CancelInterrupts();
}

template<typename T>
void InterruptNotifier<T>::Start() {
  this->StartTask();
  sensor_->EnableInterrupts();
}

template<typename T>
void InterruptNotifier<T>::StopNotifications() {
  sensor_->DisableInterrupts();
}

// WARNING: This IS called from an ISR. Don't use floating point. Look in
// interrupt_bridge-tmpl.h for details.
template<typename T>
void InterruptNotifier<T>::StaticNotify(uint32_t, void *self_in) {
  ASM_COMMENT("beginning of InterruptNotifier::StaticNotify");
  if (!intContext()) {  // if we're not in an actual ISR
    logMsg(const_cast<char *>("WPILib is not calling callbacks"
                              " in actual ISRs any more!!\n"),
           0, 0, 0, 0, 0, 0);
  }
  InterruptNotifier<T> *const self =
      static_cast<InterruptNotifier<T> *>(self_in);
  self->Notify();
  ASM_COMMENT("end of InterruptNotifier::StaticNotify");
}

}  // namespace crio
}  // namespace aos
