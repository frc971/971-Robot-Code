#include <intLib.h>
#include <logLib.h>

namespace aos {
namespace crio {

template<typename T>
InterruptNotifier<T>::InterruptNotifier(
    Handler handler,
    InterruptableSensorBase *sensor, T *param)
    : handler_(handler), param_(param), sensor_(sensor) {
  sensor_->RequestInterrupts(StaticNotify, this);
}

template<typename T>
InterruptNotifier<T>::~InterruptNotifier() {
  sensor_->CancelInterrupts();
}

template<typename T>
void InterruptNotifier<T>::Start() {
  sensor_->EnableInterrupts();
}

template<typename T>
void InterruptNotifier<T>::StopNotifications() {
  sensor_->DisableInterrupts();
}

template<typename T>
void InterruptNotifier<T>::StaticNotify(uint32_t, void *self_in) {
  if (intContext()) {  // if we are in an actual ISR
    logMsg(const_cast<char *>("WPILib is calling callbacks"
                              " in actual ISRs now!!\n"),
           0, 0, 0, 0, 0, 0);
    return;
  }
  InterruptNotifier<T> *const self =
      static_cast<InterruptNotifier<T> *>(self_in);
  self->handler_(self->param_);
}

}  // namespace crio
}  // namespace aos
