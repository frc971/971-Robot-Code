#include "aos/atom_code/output/motor_output.h"

#include <math.h>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network_port.h"

namespace aos {

// 255 = 2.5ms, 0 = 0.5ms (or something close to that)
// got 211->210 as first transition that made a speed difference and 53->54 on
//   the other side
// going 2 to each side to make sure we get the full range
const MotorOutput::MotorControllerBounds MotorOutput::kTalonBounds
    {213, 135, 132, 129, 50};

uint8_t MotorOutput::MotorControllerBounds::Map(double value) const {
  if (value == 0.0) return kCenter;
  if (value > 12.0) return Map(12.0);
  if (value < -12.0) return Map(-12.0);
  uint8_t r;
  if (value > 0.0) {
    r = static_cast<uint8_t>(kDeadbandMax + (value * (kMax - kDeadbandMax)) +
                             0.5);
  } else {
    r = static_cast<uint8_t>(kDeadbandMin + (value * (kDeadbandMin - kMin)) +
                             0.5);
  }
  if (r < kMin) return kMin;
  if (r > kMax) return kMax;
  return r;
}

MotorOutput::MotorOutput()
  : socket_(NetworkPort::kMotors, ::aos::NetworkAddress::kCRIO) {
}

void MotorOutput::Run() {
  while (true) {
    time::PhasedLoopXMS(5, 1000);

    values_.digital_module = -1;
    // 0 means output disabled.
    memset(&values_.pwm_outputs, 0x00, sizeof(values_.pwm_outputs));
    values_.digital_output_enables = 0;
    values_.digital_output_values = 0;
    values_.pressure_switch_channel = 0;
    values_.compressor_channel = 0;
    values_.solenoid_module = -1;
    values_.solenoid_values = 0;

    RunIteration();

    char buffer[sizeof(values_) + ::buffers::kOverhead];
    ssize_t size = values_.SerializeTo(buffer, sizeof(buffer));
    if (size <= 0) {
      LOG(WARNING, "serializing outputs failed\n");
      continue;
    }
    if (socket_.Send(buffer, size) != size) {
      LOG(WARNING, "sending outputs failed\n");
      continue;
    } else {
      LOG(DEBUG, "sent outputs\n");
    }
  }
}

void MotorOutput::SetSolenoid(uint8_t channel, bool set) {
  if (set) {
    values_.solenoid_values |= 1 << (channel - 1);
  } else {
    values_.solenoid_values &= ~(1 << (channel - 1));
  }
}

void MotorOutput::SetPWMOutput(uint8_t channel, double value,
                               const MotorControllerBounds &bounds) {
  values_.pwm_outputs[channel - 1] = bounds.Map(value);
}

void MotorOutput::DisablePWMOutput(uint8_t channel) {
  values_.pwm_outputs[channel - 1] = 0;
}

void MotorOutput::SetDigitalOutput(uint8_t channel, bool value) {
  const uint8_t shift_amount = 15 - channel;
  values_.digital_output_enables |= 1 << shift_amount;
  if (value) {
    values_.digital_output_values |= 1 << shift_amount;
  } else {
    values_.digital_output_values &= ~(1 << shift_amount);
  }
}

}  // namespace aos
