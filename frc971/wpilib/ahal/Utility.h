/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef FRC971_WPILIB_AHAL_UTILITY_H_
#define FRC971_WPILIB_AHAL_UTILITY_H_

#include <string>

namespace frc {

// Returns the FPGA Version number.  For now, this is the competition year.
int GetFPGAVersion();

// Returns the FPGA Revision number.
// The format of the revision is 3 numbers.
// The 12 most significant bits are the Major Revision.
// the next 8 bits are the Minor Revision.
// The 12 least significant bits are the Build Number.
int64_t GetFPGARevision();

// Reads the microsecond-resolution timer on the FPGA since reset.
uint64_t GetFPGATime();

// Gets the state of the "USER" button on the roboRIO, returning true if
// pressed.
bool GetUserButton();

}  // namespace frc

#endif  // FRC971_WPILIB_AHAL_UTILITY_H_
