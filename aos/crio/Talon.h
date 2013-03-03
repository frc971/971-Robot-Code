#ifndef AOS_CRIO_TALON_H_
#define AOS_CRIO_TALON_H_

#include "WPILib/SafePWM.h"
#include "WPILib/SpeedController.h"
#include "WPILib/PIDOutput.h"

// Used for controlling a Talon speed controller. Non-standard API and
// namespace so that the likely WPILib version will be drop-in replaceable.
class Talon : public SafePWM, public SpeedController {
 public:
  explicit Talon(UINT32 channel);

  virtual void Set(float value, UINT8 syncGroup=0);
  virtual float Get();
  virtual void Disable();

  virtual void PIDWrite(float output);
};

#endif  // AOS_CRIO_TALON_H_
