/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include "frc971/wpilib/ahal/ErrorBase.h"

namespace frc {

inline int GetDefaultSolenoidModule() { return 0; }

bool CheckSolenoidModule(int moduleNumber);
bool CheckDigitalChannel(int channel);
bool CheckRelayChannel(int channel);
bool CheckPWMChannel(int channel);
bool CheckAnalogInputChannel(int channel);
bool CheckAnalogOutputChannel(int channel);
bool CheckSolenoidChannel(int channel);
bool CheckPDPChannel(int channel);

extern const int kDigitalChannels;
extern const int kAnalogInputs;
extern const int kAnalogOutputs;
extern const int kSolenoidChannels;
extern const int kSolenoidModules;
extern const int kPwmChannels;
extern const int kRelayChannels;
extern const int kPDPChannels;

}  // namespace frc
