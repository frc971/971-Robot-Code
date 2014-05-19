#include "aos/prime/output/motor_output.h"

#include <math.h>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network_port.h"
#include "aos/common/messages/robot_state.q.h"

namespace aos {

// 255 = 2.5ms, 0 = 0.5ms (or something close to that)
// got 211->210 as first transition that made a speed difference and 53->54 on
//   the other side
// going 2 to each side to make sure we get the full range
const MotorOutput::MotorControllerBounds MotorOutput::kTalonBounds
    {213, 135, 132, 129, 50};
const MotorOutput::MotorControllerBounds MotorOutput::kVictorBounds
    {210, 138, 132, 126, 56};

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
  if (socket_.LastStatus() != 0) {
    LOG(FATAL, "opening output socket failed (returned %d)\n",
        socket_.LastStatus());
  }
  values_.solenoid_values = 0;
}

void MotorOutput::Run() {
  ::aos::time::Time::EnableMockTime();
  while (true) {
    ::aos::time::Time::UpdateMockTime();
    time::PhasedLoopXMS(5, 1000);
    ::aos::time::Time::UpdateMockTime();

    no_robot_state_.Print();
    fake_robot_state_.Print();
    sending_failed_.Print();

    values_.digital_module = -1;
    // 0 means output disabled.
    memset(&values_.pwm_outputs, 0x00, sizeof(values_.pwm_outputs));
    values_.digital_output_enables = 0;
    values_.digital_output_values = 0;
    values_.pressure_switch_channel = 0;
    values_.compressor_channel = 0;
    values_.solenoid_module = -1;

    RunIteration();

    ::aos::robot_state.FetchLatest();
    if (!::aos::robot_state.get()) {
      LOG_INTERVAL(no_robot_state_);
      continue;
    }
    if (::aos::robot_state->fake) {
      LOG_INTERVAL(fake_robot_state_);
      continue;
    }

    char buffer[sizeof(values_) + ::buffers::kOverhead];
    ssize_t size = values_.SerializeTo(buffer, sizeof(buffer));
    if (size <= 0) {
      LOG(WARNING, "serializing outputs failed\n");
      continue;
    }
    if (socket_.Send(buffer, size) != size) {
      LOG_INTERVAL(sending_failed_);
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

void MotorOutput::SetRawPWMOutput(uint8_t channel, uint8_t value) {
  values_.pwm_outputs[channel - 1] = value;
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
