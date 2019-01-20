/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc971/wpilib/ahal/Base.h"
#include "llvm/StringRef.h"

#define wpi_setErrnoErrorWithContext(context) \
  this->SetErrnoError((context), __FILE__, __FUNCTION__, __LINE__)
#define wpi_setErrnoError() wpi_setErrnoErrorWithContext("")
#define wpi_setImaqErrorWithContext(code, context) \
  do {                                             \
  } while (0)
#define wpi_setErrorWithContext(code, context) \
  do {                                         \
  } while (0)
#define wpi_setErrorWithContextRange(code, min, max, req, context) \
  do {                                                             \
  } while (0)
#define wpi_setError(code) wpi_setErrorWithContext(code, "")
#define wpi_setStaticErrorWithContext(object, code, context) \
  do {                                                       \
  } while (0)
#define wpi_setStaticError(object, code) \
  wpi_setStaticErrorWithContext(object, code, "")
#define wpi_setGlobalErrorWithContext(code, context) \
  do {                                               \
  } while (0)
#define wpi_setGlobalError(code) wpi_setGlobalErrorWithContext(code, "")
#define wpi_setWPIErrorWithContext(error, context) \
  do {                                             \
  } while (0)
#define wpi_setWPIError(error) wpi_setWPIErrorWithContext(error, "")
#define wpi_setStaticWPIErrorWithContext(object, error, context)  \
  object->SetWPIError((wpi_error_s_##error), (context), __FILE__, \
                      __FUNCTION__, __LINE__)
#define wpi_setStaticWPIError(object, error) \
  wpi_setStaticWPIErrorWithContext(object, error, "")
#define wpi_setGlobalWPIErrorWithContext(error, context)                \
  ::frc::ErrorBase::SetGlobalWPIError((wpi_error_s_##error), (context), \
                                      __FILE__, __FUNCTION__, __LINE__)
#define wpi_setGlobalWPIError(error) wpi_setGlobalWPIErrorWithContext(error, "")

namespace frc {

inline bool StatusIsFatal() { return false; }

}  // namespace frc
