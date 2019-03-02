/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc971/wpilib/ahal/RobotBase.h"

#include <cstdio>

#include "hal/HAL.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Utility.h"
#include "frc971/wpilib/ahal/WPILibVersion.h"

using namespace frc;

/**
 * Constructor for a generic robot program.
 *
 * User code should be placed in the constructor that runs before the Autonomous
 * or Operator Control period starts. The constructor will run to completion
 * before Autonomous is entered.
 *
 * This must be used to ensure that the communications code starts. In the
 * future it would be nice to put this code into it's own task that loads on
 * boot so ensure that it runs.
 */
RobotBase::RobotBase() : m_ds(DriverStation::GetInstance()) {
  std::FILE *file = nullptr;
  file = std::fopen("/tmp/frc_versions/FRC_Lib_Version.ini", "w");

  if (file != nullptr) {
    std::fputs("C++ ", file);
    std::fputs(WPILibVersion, file);
    std::fclose(file);
  }
}
