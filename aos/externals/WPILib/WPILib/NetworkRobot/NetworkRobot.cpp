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

NetworkRobot::NetworkRobot(UINT16 receive_port, const char *sender_address,
                           UINT16 send_port, const char *receiver_address)
    : receive_port_(receive_port), sender_address_(sender_address),
      send_port_(send_port), receiver_address_(receiver_address),
      receive_socket_(-1), send_socket_(-1),
      joystick_values_(),
      send_task_("DS_Send", reinterpret_cast<FUNCPTR>(StaticSendLoop)),
      last_received_timestamp_(0.0),
      digital_modules_(), solenoid_bases_(),
      allocated_digital_outputs_() {
}

NetworkRobot::~NetworkRobot() {
  // Nothing we can really do about any errors for either of these.
  if (receive_socket_ != -1) {
    close(receive_socket_);
  }
  if (send_socket_ != -1) {
    close(send_socket_);
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

bool NetworkRobot::FillinInAddr(const char *const_ip, in_addr *inet_address) {
  // A copy of the passed in address string because vxworks has the function
  // signature without the const and I don't really trust it not to do something
  // weird and change it.
  // The size is the maximum length of an IP address (including the terminating
  // NUL) (ie "123.456.789.123").
  char non_const_ip[3 + 1 + 3 + 1 + 3 + 1 + 3 + 1];
  size_t ip_length = strlen(const_ip);
  if (ip_length >= sizeof(non_const_ip)) {
    char buf[128];
    snprintf(buf, sizeof(buf), "IP address '%s' is %zd bytes long"
             " but should only be %zd", const_ip,
             ip_length, sizeof(non_const_ip) - 1);
    wpi_setErrorWithContext(-1, buf);
    return false;
  }
  memcpy(non_const_ip, const_ip, ip_length + 1);
  errno = 0;
  if (inet_aton(non_const_ip, inet_address) != 0) {
    char buf[64];
    snprintf(buf, sizeof(buf), "inet_aton(%s)", const_ip);
    wpi_setErrnoErrorWithContext(buf);
    return false;
  }
  return true;
}

void NetworkRobot::StartCompetition() {
  // Multiplied by 2 to give ourselves plenty of time to get around to feeding
  // it before it completely cuts out everything.
  m_watchdog.SetExpiration(kDisableTime * 2);

  CreateReceiveSocket();
  if (StatusIsFatal()) return;
  CreateSendSocket();
  if (StatusIsFatal()) return;

  send_task_.Start(reinterpret_cast<uintptr_t>(this));

  if (!FillinInAddr(sender_address_, &expected_sender_address_)) return;

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
  FD_SET(receive_socket_, &fds);

  int ret = select(receive_socket_ + 1,
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
  int received = recvfrom(receive_socket_,
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
  {
    // Don't have to synchronize around getting the current state too because
    // it's ok if we're 1 cycle off. It would just be bad if we reported not
    // being in any state or in 2 states at once.
    RWLock::Locker state_locker(m_ds->GetUserStateLock(), true);
    m_ds->InDisabled(state == DriverStation::FMSState::kDisabled);
    m_ds->InAutonomous(state == DriverStation::FMSState::kAutonomous);
    m_ds->InOperatorControl(state == DriverStation::FMSState::kTeleop);
    m_ds->InTest(state == DriverStation::FMSState::kTestMode);
  }

  m_watchdog.Feed();
  last_received_timestamp_ = Timer::GetPPCTimestamp();
}

void NetworkRobot::SendLoop() {
  while (!StatusIsFatal()) {
    m_ds->WaitForData();

    {
      RWLock::Locker data_locker(m_ds->GetDataReadLock());
      // Get a pointer to the data and then cast away the volatile because it's
      // annoying to propagate it all over and it's unnecessary here because
      // we have a lock on the data so it's not going to change.
      const FRCCommonControlData *data =
          const_cast<const FRCCommonControlData *>(m_ds->GetControlData());
      CopyStickValues(0, data->stick0Axes, data->stick0Buttons);
      CopyStickValues(1, data->stick1Axes, data->stick1Buttons);
      CopyStickValues(2, data->stick2Axes, data->stick2Buttons);
      CopyStickValues(3, data->stick3Axes, data->stick3Buttons);

      joystick_values_.control_packet_index = data->packetIndex;

      joystick_values_.team_number = data->teamID;
      joystick_values_.alliance = data->dsID_Alliance;
      joystick_values_.position = data->dsID_Position;

      joystick_values_.control.set_test_mode(data->test);
      joystick_values_.control.set_fms_attached(data->fmsAttached);
      joystick_values_.control.set_autonomous(data->autonomous);
      joystick_values_.control.set_enabled(data->enabled);
    }

    ++joystick_values_.index;

    joystick_values_.FillInHashValue();
  }
}

void NetworkRobot::CopyStickValues(int number,
                                   const INT8 (&axes)[6],
                                   UINT16 buttons) {
  for (int i = 0; i < 6; ++i) {
    joystick_values_.joysticks[number].axes[i] = axes[i];
  }
  joystick_values_.joysticks[number].buttons = buttons;
}

void NetworkRobot::CreateReceiveSocket() {
  wpi_assertEqual(receive_socket_, -1);

  receive_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (receive_socket_ < 0) {
    wpi_setErrnoErrorWithContext("Creating UDP Socket");
    receive_socket_ = -1;
    return;
  }

  union {
    sockaddr_in in;
    sockaddr addr;
  } address;
  memset(&address, 0, sizeof(address));
  address.in.sin_family = AF_INET;
  address.in.sin_port = receive_port_;
  address.in.sin_addr.s_addr = INADDR_ANY;
  if (bind(receive_socket_, &address.addr, sizeof(address.addr)) == ERROR) {
    wpi_setErrnoErrorWithContext("Binding Socket");
    return;
  }

  int on = 1;
  errno = 0;
  if (ioctl(receive_socket_, FIONBIO, reinterpret_cast<int>(&on)) < 0) {
    wpi_setErrnoErrorWithContext("Setting Socket to Non-Blocking Mode");
    return;
  }
}

void NetworkRobot::CreateSendSocket() {
  wpi_assertEqual(send_socket_, -1);

  send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_socket_ < 0) {
    wpi_setErrnoErrorWithContext("Creating UDP Socket");
    send_socket_ = -1;
    return;
  }

  union {
    sockaddr_in in;
    sockaddr addr;
  } address;
  memset(&address, 0, sizeof(address));
  address.in.sin_family = AF_INET;
  address.in.sin_port = send_port_;
  if (!FillinInAddr(receiver_address_, &address.in.sin_addr)) return;

  if (bind(send_socket_, &address.addr, sizeof(address.addr)) == ERROR) {
    wpi_setErrnoErrorWithContext("Binding Socket");
    return;
  }
}
