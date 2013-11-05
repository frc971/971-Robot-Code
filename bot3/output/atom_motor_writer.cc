#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/atom_code/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/atom_code/init.h"

#include "frc971/queues/Piston.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"

using ::bot3::control_loops::drivetrain;
using ::bot3::control_loops::shooter;
using ::frc971::control_loops::shifters;

namespace bot3 {
namespace output {

class MotorWriter : public ::aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static const int kEnableDrivetrain = true;

  virtual void RunIteration() {
    values_.digital_module = 0;
    values_.pressure_switch_channel = 1;
    values_.compressor_channel = 1; //spike
    values_.solenoid_module = 0;

    drivetrain.output.FetchLatest();
    if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS) && kEnableDrivetrain) {
      SetPWMOutput(4, drivetrain.output->right_voltage / 12.0, kTalonBounds);
      SetPWMOutput(3, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(3);
      DisablePWMOutput(4);
      if (kEnableDrivetrain) {
        LOG(WARNING, "drivetrain not new enough\n");
      }
    }
    shifters.FetchLatest();
    if (shifters.get()) {
      SetSolenoid(7, shifters->set);
      SetSolenoid(8, !shifters->set);
    }

    shooter.output.FetchLatest();
    if (shooter.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(1, LinearizeVictor(shooter.output->voltage / 12.0), kVictorBounds);
      SetPWMOutput(7, shooter.output->intake, kVictorBounds);
      SetSolenoid(4, shooter.output->push);
      LOG(DEBUG, "shooter: %lf, intake: %lf, push: %d", shooter.output->voltage, shooter.output->intake, shooter.output->push);
    } else {
      DisablePWMOutput(2);
      DisablePWMOutput(1);
      LOG(WARNING, "shooter not new enough\n");
    }
    // TODO(danielp): Add stuff for intake and shooter.
  }

  // This linearizes the value from the victor.
  // Copied form the 2012 code.
  double LinearizeVictor(double goal_speed) {
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

    return answer;
  }

};

}  // namespace output
}  // namespace bot3

int main() {
  ::aos::Init();
  ::bot3::output::MotorWriter writer;
  writer.Run();
  ::aos::Cleanup();
}
