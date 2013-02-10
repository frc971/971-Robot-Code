#include "aos/crio/Talon.h"
#include "WPILib/DigitalModule.h"

Talon::Talon(UINT32 channel) : SafePWM(channel) {
  // 255 = 2.5ms, 0 = 0.5ms (or something close to that)
  // these numbers were determined empirically with real hardware by Brian
  //   on 11/23/12
  //   got 211->210 as first transition that made a speed difference and
  //   53->54 on the other side
  //   going 2 to each side to make sure we get the full range
  SetBounds(213, 137, 132, 127, 50);
  // 1X = every 5.05ms, 2X and 4x are multiples of that
  SetPeriodMultiplier(kPeriodMultiplier_1X);
  SetRaw(m_centerPwm);
}

void Talon::Set(float speed, UINT8 /*syncGroup*/) { SetSpeed(speed); }
float Talon::Get() { return GetSpeed(); }
void Talon::Disable() { SetRaw(kPwmDisabled); }

void Talon::PIDWrite(float output) { Set(output); }
