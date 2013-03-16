#include "aos/common/sensors/sensor_receiver.h"
#include "aos/atom_code/init.h"

#include "frc971/queues/sensor_values.h"
#include "frc971/input/sensor_unpacker.h"

int main() {
  ::aos::Init();
  ::frc971::SensorUnpacker unpacker;
  ::aos::sensors::NetworkSensorReceiver< ::frc971::sensor_values>
      receiver(&unpacker);
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
