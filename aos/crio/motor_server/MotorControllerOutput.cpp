#include "aos/crio/motor_server/MotorControllerOutput.h"

#include "aos/aos_core.h"
#include "aos/common/byteorder.h"
#include "aos/common/commonmath.h"

namespace aos {

void LinearizedVictor::Set(float speed, UINT8 syncGroup) {
  speed_ = speed;
  Victor::Set(Linearize(speed), syncGroup);
}

float LinearizedVictor::Get() {
  return speed_;
}

void LinearizedVictor::Disable() {
  Victor::Disable();
  speed_ = 0.0;
}

double LinearizedVictor::Linearize(double goal_speed) {
  // These values were derived by putting the robot up on blocks, and driving it
  // at various speeds.  The power required to drive at these speeds was then
  // recorded and fit with gnuplot.
  const double deadband_value = 0.082;
  // If we are outside the range such that the motor is actually moving,
  // subtract off the constant offset from the deadband.  This makes the
  // function odd and intersect the origin, making the fitting easier to do.
  if (goal_speed > deadband_value) {
    goal_speed -= deadband_value;
  } else if (goal_speed < -deadband_value) {
    goal_speed += deadband_value;
  } else {
    goal_speed = 0.0;
  }
  goal_speed = goal_speed / (1.0 - deadband_value);

  double goal_speed2 = goal_speed * goal_speed;
  double goal_speed3 = goal_speed2 * goal_speed;
  double goal_speed4 = goal_speed3 * goal_speed;
  double goal_speed5 = goal_speed4 * goal_speed;
  double goal_speed6 = goal_speed5 * goal_speed;
  double goal_speed7 = goal_speed6 * goal_speed;

  // Constants for the 5th order polynomial
  double victor_fit_e1  = 0.437239;
  double victor_fit_c1  = -1.56847;
  double victor_fit_a1  = (- (125.0 * victor_fit_e1  + 125.0
                              * victor_fit_c1 - 116.0) / 125.0);
  double answer_5th_order = (victor_fit_a1 * goal_speed5
                             + victor_fit_c1 * goal_speed3
                             + victor_fit_e1 * goal_speed);

  // Constants for the 7th order polynomial
  double victor_fit_c2 = -5.46889;
  double victor_fit_e2 = 2.24214;
  double victor_fit_g2 = -0.042375;
  double victor_fit_a2 = (- (125.0 * (victor_fit_c2 + victor_fit_e2
                                      + victor_fit_g2) - 116.0) / 125.0);
  double answer_7th_order = (victor_fit_a2 * goal_speed7
                             + victor_fit_c2 * goal_speed5
                             + victor_fit_e2 * goal_speed3
                             + victor_fit_g2 * goal_speed);


  // Average the 5th and 7th order polynomials, and add a bit of linear power in
  // as well.  The average turns out to nicely fit the data in gnuplot with nice
  // smooth curves, and the linear power gives it a bit more punch at low speeds
  // again.  Stupid victors.
  double answer =  0.85 * 0.5 * (answer_7th_order + answer_5th_order)
      + .15 * goal_speed * (1.0 - deadband_value);

  // Add the deadband power back in to make it so that the motor starts moving
  // when any power is applied.  This is what the fitting assumes.
  if (answer > 0.001) {
    answer += deadband_value;
  } else if (answer < -0.001) {
    answer -= deadband_value;
  }

  return Clip(answer, -1.0, 1.0);
}

bool MotorControllerOutput::ReadValue(ByteBuffer &buff) {
  const float val = buff.read_float();
  if (val == (1.0 / 0.0)) {
    return false;
  }
  value = val;
  return true;
}
void MotorControllerOutput::SetValue() {
  output.Set(value);
}
void MotorControllerOutput::NoValue() {
  // this is NOT a Set(0.0); it's the same as when the robot is disabled
  output.Disable();
}

} // namespace aos

