#ifndef AOS_CRIO_SHARED_LIBS_INTERRUPT_NOTIFIER_H_
#define AOS_CRIO_SHARED_LIBS_INTERRUPT_NOTIFIER_H_

#include "WPILib/InterruptableSensorBase.h"

#include "aos/crio/shared_libs/interrupt_bridge.h"

namespace aos {
namespace crio {

// An InterruptBridge that notifies based on interrupts from a WPILib
// InterruptableSensorBase object (which DigitalInput is an example of).
template<typename T>
class InterruptNotifier : public InterruptBridge<T> {
 public:
  // This object will hold a reference to sensor, but will not free it. This
  // object will take ownership of everything related to interrupts for sensor
  // (ie this constructor will call sensor->RequestInterrupts).
  // Interrupts should be cancelled (the state InterruptableSensorBases are
  // constructed in) when this constructor is called.
  // Any setup of sensor that is required should happen before Start() is
  // called, but after this constructor (ie SetUpSourceEdge).
  InterruptNotifier(typename InterruptBridge<T>::Handler handler,
                    InterruptableSensorBase *sensor,
                    T *param = NULL,
                    int priority = InterruptBridge<T>::kDefaultPriority);
  virtual ~InterruptNotifier();

  // Starts calling the handler whenever the interrupt triggers.
  void Start();

 private:
  // The only docs that seem to exist on the first arg is that it's named
  // interruptAssertedMask in WPILib/ChipObject/tInterruptManager.h
  // The second arg is the general callback parameter which will be a pointer to
  // an instance. This function calls Notify() on that instance.
  static void StaticNotify(uint32_t, void *self_in);
  virtual void StopNotifications();

  InterruptableSensorBase *const sensor_;
  DISALLOW_COPY_AND_ASSIGN(InterruptNotifier<T>);
};

}  // namespace crio
}  // namespace aos

#include "aos/crio/shared_libs/interrupt_notifier-tmpl.h"

#endif  // AOS_CRIO_SHARED_LIBS_INTERRUPT_NOTIFIER_H_
