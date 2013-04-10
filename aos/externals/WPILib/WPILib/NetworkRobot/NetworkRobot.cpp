/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "WPILib/NetworkRobot/NetworkRobot.h"

#include <sockLib.h>
#include <stdint.h>
#include <selectLib.h>
#include <assert.h>

#include "WPILib/Utility.h"
#include "WPILib/WPIErrors.h"
#include "WPILib/SensorBase.h"
#include "WPILib/DriverStation.h"
#include "WPILib/Timer.h"

const double NetworkRobot::kDisableTime = 0.15;

NetworkRobot::NetworkRobot(UINT16 port, const char *sender_address)
    : port_(port), sender_address_(sender_address), socket_(-1),
      last_received_timestamp_(0.0),
      digital_modules_(), solenoid_bases_(),
      allocated_digital_outputs_() {
}
NetworkRobot::~NetworkRobot() {
  if (socket_ != -1) {
    // Nothing we can do about any errors.
    close(socket_);
  }
  for (size_t i = 0;
       i < sizeof(solenoid_bases_) / sizeof(solenoid_bases_[0]);
       ++i) {
    delete solenoid_bases_[i];
  }
  for (size_t module = 0;
       module < sizeof(digital_modules_) / sizeof(digital_modules_[0]);
       ++module) {
    for (int i = 0; i < 16; ++i) {
      if (allocated_digital_outputs_[module] & (1 << i)) {
        digital_modules_[module]->FreeDIO(15 - i);
      }
    }
  }
}

void NetworkRobot::StartCompetition() {
  // Give ourself plenty of time to get around to feeding it before it
  // completely cuts out.
  m_watchdog.SetExpiration(kDisableTime * 2);

  CreateSocket();
  if (StatusIsFatal()) return;

  errno = 0;
  if (inet_aton(const_cast<char *>(sender_address_),
                &expected_sender_address_) == ERROR) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Converting IP Address %s", sender_address_);
    wpi_setErrnoErrorWithContext(buf);
    return;
  }

  while (!StatusIsFatal()) {
    if ((Timer::GetPPCTimestamp() - last_received_timestamp_) > kDisableTime) {
      StopOutputs();
    }
    ReceivePacket();
  }
  StopOutputs();
}

void NetworkRobot::StopOutputs() {
  for (size_t module = 0;
       module < sizeof(digital_modules_) / sizeof(digital_modules_[0]);
       ++module) {
    DigitalModule *digital_module = digital_modules_[module];

    if (digital_module != NULL) {
      for (int i = 0; i < 10; ++i) {
        // 0 means stop sending anything.
        digital_module->SetPWM(i + 1, 0);
      }

      // Turn off all of the ones that we're responsible for.
      digital_module->SetDIOs(allocated_digital_outputs_[module], 0);

      // Turn off all of the relays (both directions).
      digital_module->SetRelaysForward(0xFF, 0);
      digital_module->SetRelaysReverse(0xFF, 0);
    }
  }

  // Can't do anything intelligent with solenoids. Turning them off can be just
  // as dangerous as leaving them on.

  // We took care of it, so we don't want the watchdog to permanently disable
  // us.
  m_watchdog.Feed();
}

bool NetworkRobot::WaitForData() {
  assert(kDisableTime < 1.0);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = kDisableTime *
      1000.0 /*seconds to mseconds*/ *
      1000.0 /*mseconds to useconds*/;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(socket_, &fds);

  int ret = select(socket_ + 1,
                   &fds, // read fds
                   NULL, // write fds
                   NULL, // exception fds (not supported)
                   &timeout);
  if (ret == 0) {
    // timeout
    return false;
  } else if (ret == 1) {
    return true;
  }
  wpi_setErrnoErrorWithContext("waiting until the socket has data");
  return false;
}

void NetworkRobot::ReceivePacket() {
  if (!WaitForData()) return;

  union {
    sockaddr addr;
    sockaddr_in in;
  } sender_address;
  int sender_address_length = sizeof(sender_address);
  int received = recvfrom(socket_,
                          reinterpret_cast<char *>(&values_),
                          sizeof(values_),
                          0,
                          &sender_address.addr,
                          &sender_address_length);
  if (received == -1) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
        errno == ETIMEDOUT) {
      // All of these are various kinds of timing out.
      return;
    }
    wpi_setErrnoErrorWithContext("recvfrom on motor value socket");
    return;
  }
  wpi_assert(static_cast<size_t>(sender_address_length) >=
             sizeof(sender_address.in));
  if (sender_address.in.sin_addr.s_addr !=
      expected_sender_address_.s_addr) {
    char address[INET_ADDR_LEN];
    inet_ntoa_b(sender_address.in.sin_addr, address);
    char buf[64];
    snprintf(buf, sizeof(buf), "Received packet from wrong IP %s", address);
    wpi_setErrorWithContext(1, buf);
    return;
  }

  if (received != sizeof(values_)) {
    char buf[64];
    snprintf(buf, sizeof(buf),
             "Got packet with %zd bytes but expected %zd\n",
             received, sizeof(values_));
    wpi_setErrorWithContext(1, buf);
    return;
  }
  if (values_.CheckHashValue()) {
    ProcessPacket();
  } else {
    char buf[64];
    snprintf(buf, sizeof(buf), "Hash Value Is %x", values_.hash_value);
    wpi_setErrorWithContext(1, buf);
    return;
  }
}

void NetworkRobot::ProcessPacket() {
  int8_t digital_number = values_.digital_module;
  if (digital_number != -1) {
    if (digital_number == 0) {
      digital_number = SensorBase::GetDefaultDigitalModule();
    }
    if (digital_number < 1 || digital_number > 2) {
      char buf[64];
      snprintf(buf, sizeof(buf), "Got Digital Module %d",
               static_cast<int>(digital_number));
      wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf);
      return;
    }
    DigitalModule *digital_module = digital_modules_[digital_number - 1];
    if (digital_module == NULL) {
      digital_module = digital_modules_[digital_number - 1] =
          DigitalModule::GetInstance(digital_number);
      for (int i = 0; i < 10; ++i) {
        digital_module->SetPWMPeriodScale(i + 1, 0);
      }
    }

    for (int i = 0; i < 10; ++i) {
      digital_module->SetPWM(i + 1, values_.pwm_outputs[i]);
    }

    uint16_t old_allocated = allocated_digital_outputs_[digital_number - 1];
    // Have to keep track of which ones we've (de)allocated as we go through in
    // case we have trouble allocating one of them in the middle.
    for (int i = 0; i < 16; ++i) {
      // If we have it allocated and this packet says we shouldn't.
      if ((old_allocated & (1 << i)) &&
          !(values_.digital_output_enables & (1 << i))) {
        digital_module->FreeDIO(15 - i);
        allocated_digital_outputs_[digital_number - 1] &= ~(1 << i);
      // If this packet says we should have it allocated and we don't.
      } else if ((values_.digital_output_enables & (1 << i)) &&
                 !(old_allocated & (1 << i))) {
        if (!digital_module->AllocateDIO(15 - i, false /*input*/)) return;
        allocated_digital_outputs_[digital_number - 1] |= 1 << i;
      }
    }
    wpi_assertEqual(allocated_digital_outputs_[digital_number - 1],
                    values_.digital_output_enables);
    digital_module->SetDIOs(values_.digital_output_enables,
                            values_.digital_output_values);

    if (values_.pressure_switch_channel != 0 &&
        values_.compressor_channel != 0) {
      digital_module->SetRelayForward(values_.compressor_channel,
                                      !digital_module->GetDIO(
                                          values_.pressure_switch_channel));
    }
  }

  int8_t solenoid_number = values_.solenoid_module;
  if (solenoid_number != -1) {
    if (solenoid_number == 0) {
      solenoid_number = SensorBase::GetDefaultSolenoidModule();
    }
    if (solenoid_number < 1 || solenoid_number > 2) {
      char buf[64];
      snprintf(buf, sizeof(buf), "Got Solenoid Module %d",
               static_cast<int>(solenoid_number));
      wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf);
      return;
    }
    SolenoidBase *solenoid_base = solenoid_bases_[solenoid_number - 1];
    if (solenoid_base == NULL) {
      solenoid_base = solenoid_bases_[solenoid_number - 1] =
          new SolenoidBase(solenoid_number);
    }

    solenoid_base->SetAll(values_.solenoid_values);
  }

  // This code can only assume that whatever is sending it values knows what
  // state it should be in.
  DriverStation::FMSState state = m_ds->GetCurrentState();
  m_ds->InDisabled(state == DriverStation::FMSState::kDisabled);
  m_ds->InAutonomous(state == DriverStation::FMSState::kAutonomous);
  m_ds->InOperatorControl(state == DriverStation::FMSState::kTeleop);
  m_ds->InTest(state == DriverStation::FMSState::kTestMode);

  m_watchdog.Feed();
  last_received_timestamp_ = Timer::GetPPCTimestamp();
}

void NetworkRobot::CreateSocket() {
  wpi_assertEqual(socket_, -1);

  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    wpi_setErrnoErrorWithContext("Creating UDP Socket");
    socket_ = -1;
    return;
  }

  union {
    sockaddr_in in;
    sockaddr addr;
  } address;
  memset(&address, 0, sizeof(address));
  address.in.sin_family = AF_INET;
  address.in.sin_port = port_;
  address.in.sin_addr.s_addr = INADDR_ANY;
  if (bind(socket_, &address.addr, sizeof(address.addr)) == ERROR) {
    wpi_setErrnoErrorWithContext("Binding Socket");
    return;
  }

  int on = 1;
  errno = 0;
  if (ioctl(socket_, FIONBIO, reinterpret_cast<int>(&on)) < 0) {
    wpi_setErrnoErrorWithContext("Setting Socket to Non-Blocking Mode");
    return;
  }
}
