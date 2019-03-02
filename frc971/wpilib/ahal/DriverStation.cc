/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/DriverStation.h"

#include <chrono>

#include "FRC_NetworkCommunication/FRCComm.h"
#include "aos/make_unique.h"
#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Utility.h"
#include "frc971/wpilib/ahal/WPIErrors.h"
#include "hal/HAL.h"
#include "hal/Power.h"
#include "wpi/SmallString.h"

using namespace frc;

#define WPI_LIB_FATAL(error_tag) \
  do {                           \
  } while (false)

const int DriverStation::kJoystickPorts;

DriverStation::~DriverStation() {}

/**
 * Return a pointer to the singleton DriverStation.
 *
 * @return Pointer to the DS instance
 */
DriverStation &DriverStation::GetInstance() {
  static DriverStation instance;
  return instance;
}

/**
 * Report an error to the DriverStation messages window.
 *
 * The error is also printed to the program console.
 */
void DriverStation::ReportError(const wpi::Twine &error) {
  wpi::SmallString<128> temp;
  HAL_SendError(1, 1, 0, error.toNullTerminatedStringRef(temp).data(), "", "",
                1);
}

/**
 * Report a warning to the DriverStation messages window.
 *
 * The warning is also printed to the program console.
 */
void DriverStation::ReportWarning(const wpi::Twine &error) {
  wpi::SmallString<128> temp;
  HAL_SendError(0, 1, 0, error.toNullTerminatedStringRef(temp).data(), "", "",
                1);
}

/**
 * Report an error to the DriverStation messages window.
 *
 * The error is also printed to the program console.
 */
void DriverStation::ReportError(bool is_error, int32_t code,
                                const wpi::Twine &error,
                                const wpi::Twine &location,
                                const wpi::Twine &stack) {
  wpi::SmallString<128> errorTemp;
  wpi::SmallString<128> locationTemp;
  wpi::SmallString<128> stackTemp;
  HAL_SendError(is_error, code, 0,
                error.toNullTerminatedStringRef(errorTemp).data(),
                location.toNullTerminatedStringRef(locationTemp).data(),
                stack.toNullTerminatedStringRef(stackTemp).data(), 1);
}

/**
 * Get the value of the axis on a joystick.
 *
 * This depends on the mapping of the joystick connected to the specified port.
 *
 * @param stick The joystick to read.
 * @param axis  The analog axis value to read from the joystick.
 * @return The value of the axis on the joystick.
 */
double DriverStation::GetStickAxis(int stick, int axis) {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return 0;
  }
  if (axis >= m_joystickAxes[stick].count) {
    if (axis >= HAL_kMaxJoystickAxes) WPI_LIB_FATAL(BadJoystickAxis);
    return 0.0;
  }

  return m_joystickAxes[stick].axes[axis];
}

/**
 * Get the state of a POV on the joystick.
 *
 * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
 */
int DriverStation::GetStickPOV(int stick, int pov) {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return -1;
  }
  if (pov >= m_joystickPOVs[stick].count) {
    if (pov >= HAL_kMaxJoystickPOVs) WPI_LIB_FATAL(BadJoystickAxis);
    return -1;
  }

  return m_joystickPOVs[stick].povs[pov];
}

/**
 * The state of the buttons on the joystick.
 *
 * @param stick The joystick to read.
 * @return The state of the buttons on the joystick.
 */
int DriverStation::GetStickButtons(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return 0;
  }
  return m_joystickButtons[stick].buttons;
}

/**
 * The state of one joystick button. Button indexes begin at 1.
 *
 * @param stick  The joystick to read.
 * @param button The button index, beginning at 1.
 * @return The state of the joystick button.
 */
bool DriverStation::GetStickButton(int stick, int button) {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return false;
  }
  if (button == 0) {
    return false;
  }
  if (button > m_joystickButtons[stick].count) {
    return false;
  }

  return ((0x1 << (button - 1)) & m_joystickButtons[stick].buttons) != 0;
}

/**
 * Returns the number of axes on a given joystick port.
 *
 * @param stick The joystick port number
 * @return The number of axes on the indicated joystick
 */
int DriverStation::GetStickAxisCount(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return 0;
  }
  return m_joystickAxes[stick].count;
}

/**
 * Returns the number of POVs on a given joystick port.
 *
 * @param stick The joystick port number
 * @return The number of POVs on the indicated joystick
 */
int DriverStation::GetStickPOVCount(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return 0;
  }
  return m_joystickPOVs[stick].count;
}

/**
 * Returns the number of buttons on a given joystick port.
 *
 * @param stick The joystick port number
 * @return The number of buttons on the indicated joystick
 */
int DriverStation::GetStickButtonCount(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return 0;
  }
  return m_joystickButtons[stick].count;
}

/**
 * Returns a boolean indicating if the controller is an xbox controller.
 *
 * @param stick The joystick port number
 * @return A boolean that is true if the controller is an xbox controller.
 */
bool DriverStation::GetJoystickIsXbox(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return false;
  }
  return static_cast<bool>(m_joystickDescriptor[stick].isXbox);
}

/**
 * Returns the type of joystick at a given port.
 *
 * @param stick The joystick port number
 * @return The HID type of joystick at the given port
 */
int DriverStation::GetJoystickType(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return -1;
  }
  return static_cast<int>(m_joystickDescriptor[stick].type);
}

/**
 * Returns the name of the joystick at the given port.
 *
 * @param stick The joystick port number
 * @return The name of the joystick at the given port
 */
std::string DriverStation::GetJoystickName(int stick) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
  }
  std::string retVal(m_joystickDescriptor[stick].name);
  return retVal;
}

/**
 * Returns the types of Axes on a given joystick port.
 *
 * @param stick The joystick port number and the target axis
 * @return What type of axis the axis is reporting to be
 */
int DriverStation::GetJoystickAxisType(int stick, int axis) const {
  if (stick >= kJoystickPorts) {
    WPI_LIB_FATAL(BadJoystickIndex);
    return -1;
  }
  return m_joystickDescriptor[stick].axisTypes[axis];
}

/**
 * Check if the FPGA outputs are enabled.
 *
 * The outputs may be disabled if the robot is disabled or e-stopped, the
 * watchdog has expired, or if the roboRIO browns out.
 *
 * @return True if the FPGA outputs are enabled.
 */
bool DriverStation::IsSysActive() const {
  int32_t status = 0;
  bool retVal = HAL_GetSystemActive(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return retVal;
}

/**
 * Check if the system is browned out.
 *
 * @return True if the system is browned out
 */
bool DriverStation::IsBrownedOut() const {
  int32_t status = 0;
  bool retVal = HAL_GetBrownedOut(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  return retVal;
}

/**
 * Return the alliance that the driver station says it is on.
 *
 * This could return kRed or kBlue.
 *
 * @return The Alliance enum (kRed, kBlue or kInvalid)
 */
DriverStation::Alliance DriverStation::GetAlliance() const {
  int32_t status = 0;
  auto allianceStationID = HAL_GetAllianceStation(&status);
  switch (allianceStationID) {
    case HAL_AllianceStationID_kRed1:
    case HAL_AllianceStationID_kRed2:
    case HAL_AllianceStationID_kRed3:
      return kRed;
    case HAL_AllianceStationID_kBlue1:
    case HAL_AllianceStationID_kBlue2:
    case HAL_AllianceStationID_kBlue3:
      return kBlue;
    default:
      return kInvalid;
  }
}

/**
 * Return the driver station location on the field.
 *
 * This could return 1, 2, or 3.
 *
 * @return The location of the driver station (1-3, 0 for invalid)
 */
int DriverStation::GetLocation() const {
  int32_t status = 0;
  auto allianceStationID = HAL_GetAllianceStation(&status);
  switch (allianceStationID) {
    case HAL_AllianceStationID_kRed1:
    case HAL_AllianceStationID_kBlue1:
      return 1;
    case HAL_AllianceStationID_kRed2:
    case HAL_AllianceStationID_kBlue2:
      return 2;
    case HAL_AllianceStationID_kRed3:
    case HAL_AllianceStationID_kBlue3:
      return 3;
    default:
      return 0;
  }
}

/**
 * Return the approximate match time.
 *
 * The FMS does not send an official match time to the robots, but does send an
 * approximate match time.  The value will count down the time remaining in the
 * current period (auto or teleop).
 *
 * Warning: This is not an official time (so it cannot be used to dispute ref
 * calls or guarantee that a function will trigger before the match ends).
 *
 * The Practice Match function of the DS approximates the behaviour seen on the
 * field.
 *
 * @return Time remaining in current match period (auto or teleop)
 */
double DriverStation::GetMatchTime() const {
  int32_t status;
  return HAL_GetMatchTime(&status);
}

/**
 * Read the battery voltage.
 *
 * @return The battery voltage in Volts.
 */
double DriverStation::GetBatteryVoltage() const {
  int32_t status = 0;
  double voltage = HAL_GetVinVoltage(&status);
  wpi_setErrorWithContext(status, "getVinVoltage");

  return voltage;
}

/**
 * Copy data from the DS task for the user.
 *
 * If no new data exists, it will just be returned, otherwise
 * the data will be copied from the DS polling loop.
 */
void DriverStation::GetData() {
  // Get the status of all of the joysticks, and save to the cache
  for (uint8_t stick = 0; stick < kJoystickPorts; stick++) {
    HAL_GetJoystickAxes(stick, &m_joystickAxesCache[stick]);
    HAL_GetJoystickPOVs(stick, &m_joystickPOVsCache[stick]);
    HAL_GetJoystickButtons(stick, &m_joystickButtonsCache[stick]);
    HAL_GetJoystickDescriptor(stick, &m_joystickDescriptorCache[stick]);
  }
  m_joystickAxes.swap(m_joystickAxesCache);
  m_joystickPOVs.swap(m_joystickPOVsCache);
  m_joystickButtons.swap(m_joystickButtonsCache);
  m_joystickDescriptor.swap(m_joystickDescriptorCache);

  HAL_ControlWord control_word;
  HAL_GetControlWord(&control_word);
  is_enabled_ = control_word.enabled;
  is_autonomous_ = control_word.autonomous;
  is_test_mode_ = control_word.test;
  is_fms_attached_ = control_word.fmsAttached;
}

/**
 * DriverStation constructor.
 *
 * This is only called once the first time GetInstance() is called
 */
DriverStation::DriverStation() {
  // The HAL_Observe* functions let the Driver Station know that the
  // robot code is alive (this influences the Robot Code status light on the
  // DS, and if the DS thinks you don't have robot code, then you can't enable).
  HAL_ObserveUserProgramStarting();

  m_joystickAxes = aos::make_unique<HAL_JoystickAxes[]>(kJoystickPorts);
  m_joystickPOVs = aos::make_unique<HAL_JoystickPOVs[]>(kJoystickPorts);
  m_joystickButtons = aos::make_unique<HAL_JoystickButtons[]>(kJoystickPorts);
  m_joystickDescriptor =
      aos::make_unique<HAL_JoystickDescriptor[]>(kJoystickPorts);
  m_joystickAxesCache = aos::make_unique<HAL_JoystickAxes[]>(kJoystickPorts);
  m_joystickPOVsCache = aos::make_unique<HAL_JoystickPOVs[]>(kJoystickPorts);
  m_joystickButtonsCache =
      aos::make_unique<HAL_JoystickButtons[]>(kJoystickPorts);
  m_joystickDescriptorCache =
      aos::make_unique<HAL_JoystickDescriptor[]>(kJoystickPorts);

  // All joysticks should default to having zero axes, povs and buttons, so
  // uninitialized memory doesn't get sent to speed controllers.
  for (unsigned int i = 0; i < kJoystickPorts; i++) {
    m_joystickAxes[i].count = 0;
    m_joystickPOVs[i].count = 0;
    m_joystickButtons[i].count = 0;
    m_joystickDescriptor[i].isXbox = 0;
    m_joystickDescriptor[i].type = -1;
    m_joystickDescriptor[i].name[0] = '\0';

    m_joystickAxesCache[i].count = 0;
    m_joystickPOVsCache[i].count = 0;
    m_joystickButtonsCache[i].count = 0;
    m_joystickDescriptorCache[i].isXbox = 0;
    m_joystickDescriptorCache[i].type = -1;
    m_joystickDescriptorCache[i].name[0] = '\0';
  }
}

void DriverStation::RunIteration(std::function<void()> on_data) {
  HAL_WaitForDSData();
  GetData();

  // We have to feed some sort of watchdog so that the driver's station knows
  // that the robot code is still alive. HAL_ObserveUserProgramStarting must be
  // called during initialization (currently called in the constructor of this
  // class).
  if (!IsEnabled()) {
    HAL_ObserveUserProgramDisabled();
  } else if (IsAutonomous()) {
    HAL_ObserveUserProgramAutonomous();
  } else if (IsTestMode()) {
    HAL_ObserveUserProgramTest();
  } else {
    HAL_ObserveUserProgramTeleop();
  }

  on_data();
}
