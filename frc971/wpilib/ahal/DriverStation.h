/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
// #include <condition_variable>
#include <memory>
#include <string>
#include <thread>

#include "hal/DriverStation.h"
#include "frc971/wpilib/ahal/SensorBase.h"
#include "wpi/Twine.h"

namespace frc {

/**
 * Provide access to the network communication data to / from the Driver
 * Station.
 */
class DriverStation {
 public:
  enum Alliance { kRed, kBlue, kInvalid };

  virtual ~DriverStation();
  static DriverStation &GetInstance();
  static void ReportError(const wpi::Twine &error);
  static void ReportWarning(const wpi::Twine &error);
  static void ReportError(bool is_error, int code, const wpi::Twine &error,
                          const wpi::Twine &location, const wpi::Twine &stack);

  static const int kJoystickPorts = 6;

  double GetStickAxis(int stick, int axis);
  int GetStickPOV(int stick, int pov);
  int GetStickButtons(int stick) const;
  bool GetStickButton(int stick, int button);

  int GetStickAxisCount(int stick) const;
  int GetStickPOVCount(int stick) const;
  int GetStickButtonCount(int stick) const;

  bool GetJoystickIsXbox(int stick) const;
  int GetJoystickType(int stick) const;
  std::string GetJoystickName(int stick) const;
  int GetJoystickAxisType(int stick, int axis) const;

  bool IsEnabled() const { return is_enabled_; }
  bool IsTestMode() const { return is_test_mode_; }
  bool IsFmsAttached() const { return is_fms_attached_; }
  bool IsAutonomous() const { return is_autonomous_; }

  bool IsSysActive() const;
  bool IsBrownedOut() const;

  Alliance GetAlliance() const;
  int GetLocation() const;
  double GetMatchTime() const;
  double GetBatteryVoltage() const;

  void RunIteration(std::function<void()> on_data);

 protected:
  void GetData();

 private:
  DriverStation();

  // Joystick User Data
  std::unique_ptr<HAL_JoystickAxes[]> m_joystickAxes;
  std::unique_ptr<HAL_JoystickPOVs[]> m_joystickPOVs;
  std::unique_ptr<HAL_JoystickButtons[]> m_joystickButtons;
  std::unique_ptr<HAL_JoystickDescriptor[]> m_joystickDescriptor;

  // Joystick Cached Data
  std::unique_ptr<HAL_JoystickAxes[]> m_joystickAxesCache;
  std::unique_ptr<HAL_JoystickPOVs[]> m_joystickPOVsCache;
  std::unique_ptr<HAL_JoystickButtons[]> m_joystickButtonsCache;
  std::unique_ptr<HAL_JoystickDescriptor[]> m_joystickDescriptorCache;

  bool is_enabled_ = false;
  bool is_test_mode_ = false;
  bool is_autonomous_ = false;
  bool is_fms_attached_ = false;
};

}  // namespace frc
