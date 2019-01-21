/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

// A struct to use as a deleter when a std::shared_ptr must wrap a raw pointer
// that is being deleted by someone else.
template <class T>
struct NullDeleter {
  void operator()(T *) const noexcept {};
};

}  // namespace frc
