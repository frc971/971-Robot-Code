#ifndef AOS_CRIO_MOTOR_SERVER_SENSOR_SENDER_H_
#define AOS_CRIO_MOTOR_SERVER_SENSOR_SENDER_H_

namespace aos {

// A class that handles sending all of the sensor values to the atom.
// Designed for an instantiation (aos::SensorSender<X>) to be AOS_RUN_FORKed,
// NOT a subclass.
// Values is the type of the struct that will get sent out over the network.
// Note: it should the same as the instance of TODO(brians) on the atom and any
// SensorOutput instances that you want to feed into an instance of this.
template<class Values> class SensorSender {
	public:
   // Loops forever.
   void Run();
};

}  // namespace aos

#include "SensorSender-tmpl.h"

#endif  // AOS_CRIO_MOTOR_SERVER_SENSOR_SENDER_H_
