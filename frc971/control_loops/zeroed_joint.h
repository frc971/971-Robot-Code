#ifndef FRC971_CONTROL_LOOPS_ZEROED_JOINT_H_
#define FRC971_CONTROL_LOOPS_ZEROED_JOINT_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class WristTest_NoWindupPositive_Test;
class WristTest_NoWindupNegative_Test;
};

// Note: Everything in this file assumes that there is a 1 cycle delay between
// power being requested and it showing up at the motor.  It assumes that
// X_hat(2, 1) is the voltage being applied as well.  It will go unstable if
// that isn't true.

template<int kNumZeroSensors>
class ZeroedJoint;

// This class implements the CapU function correctly given all the extra
// information that we know about from the wrist motor.
template<int kNumZeroSensors>
class ZeroedStateFeedbackLoop : public StateFeedbackLoop<3, 1, 1> {
 public:
  ZeroedStateFeedbackLoop(StateFeedbackLoop<3, 1, 1> loop,
                          ZeroedJoint<kNumZeroSensors> *zeroed_joint)
      : StateFeedbackLoop<3, 1, 1>(loop),
        zeroed_joint_(zeroed_joint),
        voltage_(0.0),
        last_voltage_(0.0) {
  }

  // Caps U, but this time respects the state of the wrist as well.
  virtual void CapU();

  // Returns the accumulated voltage.
  double voltage() const { return voltage_; }

  // Returns the uncapped voltage.
  double uncapped_voltage() const { return uncapped_voltage_; }

  // Zeros the accumulator.
  void ZeroPower() { voltage_ = 0.0; }
 private:
  ZeroedJoint<kNumZeroSensors> *zeroed_joint_;

  // The accumulated voltage to apply to the motor.
  double voltage_;
  double last_voltage_;
  double uncapped_voltage_;
};

template<int kNumZeroSensors>
void ZeroedStateFeedbackLoop<kNumZeroSensors>::CapU() {
  const double old_voltage = voltage_;
  voltage_ += U(0, 0);

  uncapped_voltage_ = voltage_;

  // Do all our computations with the voltage, and then compute what the delta
  // is to make that happen.
  if (zeroed_joint_->state_ == ZeroedJoint<kNumZeroSensors>::READY) {
    if (Y(0, 0) >= zeroed_joint_->config_data_.upper_limit) {
      voltage_ = std::min(0.0, voltage_);
    }
    if (Y(0, 0) <= zeroed_joint_->config_data_.lower_limit) {
      voltage_ = std::max(0.0, voltage_);
    }
  }

  const bool is_ready =
      zeroed_joint_->state_ == ZeroedJoint<kNumZeroSensors>::READY;
  double limit = is_ready ?
      12.0 : zeroed_joint_->config_data_.max_zeroing_voltage;

  // Make sure that reality and the observer can't get too far off.  There is a
  // delay by one cycle between the applied voltage and X_hat(2, 0), so compare
  // against last cycle's voltage.
  if (X_hat(2, 0) > last_voltage_ + 2.0) {
    //X_hat(2, 0) = last_voltage_ + 2.0;
    voltage_ -= X_hat(2, 0) - (last_voltage_ + 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  } else if (X_hat(2, 0) < last_voltage_ -2.0) {
    //X_hat(2, 0) = last_voltage_ - 2.0;
    voltage_ += X_hat(2, 0) - (last_voltage_ - 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  }

  voltage_ = std::min(limit, voltage_);
  voltage_ = std::max(-limit, voltage_);
  U(0, 0) = voltage_ - old_voltage;
  LOG(DEBUG, "abc %f\n", X_hat(2, 0) - voltage_);
  LOG(DEBUG, "error %f\n", X_hat(0, 0) - R(0, 0));

  last_voltage_ = voltage_;
}


// Class to zero and control a joint with any number of zeroing sensors with a
// state feedback controller.
template<int kNumZeroSensors>
class ZeroedJoint {
 public:
  // Sturcture to hold the hardware configuration information.
  struct ConfigurationData {
    // Angle at the lower hardware limit.
    double lower_limit;
    // Angle at the upper hardware limit.
    double upper_limit;
    // Speed (and direction) to move while zeroing.
    double zeroing_speed;
    // Speed (and direction) to move while moving off the sensor.
    double zeroing_off_speed;
    // Maximum voltage to apply when zeroing.
    double max_zeroing_voltage;
    // Deadband voltage.
    double deadband_voltage;
    // Angles where we see a positive edge from the hall effect sensors.
    double hall_effect_start_angle[kNumZeroSensors];
  };

  // Current position data for the encoder and hall effect information.
  struct PositionData {
    // Current encoder position.
    double position;
    // Array of hall effect values.
    bool hall_effects[kNumZeroSensors];
    // Array of the last positive edge position for the sensors.
    double hall_effect_positions[kNumZeroSensors];
  };

  ZeroedJoint(StateFeedbackLoop<3, 1, 1> loop)
      : loop_(new ZeroedStateFeedbackLoop<kNumZeroSensors>(loop, this)),
        last_good_time_(0, 0),
        state_(UNINITIALIZED),
        error_count_(0),
        zero_offset_(0.0),
        capped_goal_(false) {
  }

  // Copies the provided configuration data locally.
  void set_config_data(const ConfigurationData &config_data) {
    config_data_ = config_data;
  }

  // Clips the goal to be inside the limits and returns the clipped goal.
  // Requires the constants to have already been fetched.
  double ClipGoal(double goal) const {
    return ::std::min(config_data_.upper_limit,
                      std::max(config_data_.lower_limit, goal));
  }

  // Updates the loop and state machine.
  // position is null if the position data is stale, output_enabled is true if
  // the output will actually go to the motors, and goal_angle and goal_velocity
  // are the goal position and velocities.
  double Update(const ZeroedJoint<kNumZeroSensors>::PositionData *position,
                bool output_enabled,
                double goal_angle, double goal_velocity);

  // True if the code is zeroing.
  bool is_zeroing() const { return state_ == ZEROING; }

  // True if the code is moving off the hall effect.
  bool is_moving_off() const { return state_ == MOVING_OFF; }

  // True if the state machine is uninitialized.
  bool is_uninitialized() const { return state_ == UNINITIALIZED; }

  // True if the state machine is ready.
  bool is_ready() const { return state_ == READY; }

  // Returns the uncapped voltage.
  double U_uncapped() const { return loop_->U_uncapped(0, 0); }

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return capped_goal_; }

  // Timestamp
  static const double dt;

  double absolute_position() const { return loop_->X_hat(0, 0); }

 private:
  friend class ZeroedStateFeedbackLoop<kNumZeroSensors>;
  // Friend the wrist test cases so that they can simulate windeup.
  friend class testing::WristTest_NoWindupPositive_Test;
  friend class testing::WristTest_NoWindupNegative_Test;

  static const ::aos::time::Time kRezeroTime;

  // The state feedback control loop to talk to.
  ::std::unique_ptr<ZeroedStateFeedbackLoop<kNumZeroSensors>> loop_;

  ConfigurationData config_data_;

  ::aos::time::Time last_good_time_;

  // Returns the index of the first active sensor, or -1 if none are active.
  int ActiveSensorIndex(
      const ZeroedJoint<kNumZeroSensors>::PositionData *position) {
    if (!position) {
      return -1;
    }
    int active_index = -1;
    for (int i = 0; i < kNumZeroSensors; ++i) {
      if (position->hall_effects[i]) {
        if (active_index != -1) {
          LOG(ERROR, "More than one hall effect sensor is active\n");
        } else {
          active_index = i;
        }
      }
    }
    return active_index;
  }
  // Returns true if any of the sensors are active.
  bool AnySensorsActive(
      const ZeroedJoint<kNumZeroSensors>::PositionData *position) {
    return ActiveSensorIndex(position) != -1;
  }

  // Enum to store the state of the internal zeroing state machine.
  enum State {
    UNINITIALIZED,
    MOVING_OFF,
    ZEROING,
    READY,
    ESTOP
  };

  // Internal state for zeroing.
  State state_;

  // Missed position packet count.
  int error_count_;
  // Offset from the raw encoder value to the absolute angle.
  double zero_offset_;
  // Position that gets incremented when zeroing the wrist to slowly move it to
  // the hall effect sensor.
  double zeroing_position_;
  // Last position at which the hall effect sensor was off.
  double last_off_position_;

  // True if the zeroing goal was capped during this cycle.
  bool capped_goal_;

  // Returns true if number is between first and second inclusive.
  bool is_between(double first, double second, double number) {
    if ((number >= first || number >= second) &&
        (number <= first || number <= second)) {
      return true;
    }
    return false;
  }

  DISALLOW_COPY_AND_ASSIGN(ZeroedJoint);
};

template <int kNumZeroSensors>
const ::aos::time::Time ZeroedJoint<kNumZeroSensors>::kRezeroTime =
    ::aos::time::Time::InSeconds(2);

template <int kNumZeroSensors>
/*static*/ const double ZeroedJoint<kNumZeroSensors>::dt = 0.01;

// Updates the zeroed joint controller and state machine.
template <int kNumZeroSensors>
double ZeroedJoint<kNumZeroSensors>::Update(
    const ZeroedJoint<kNumZeroSensors>::PositionData *position,
    bool output_enabled,
    double goal_angle, double goal_velocity) {
  // Uninitialize the bot if too many cycles pass without an encoder.
  if (position == NULL) {
    LOG(WARNING, "no new pos given\n");
    error_count_++;
  }
  if (error_count_ >= 4) {
    output_enabled = false;
    LOG(WARNING, "err_count is %d so disabling\n", error_count_);

    if ((::aos::time::Time::Now() - last_good_time_) > kRezeroTime) {
      LOG(WARNING, "err_count is %d (or 1st time) so forcing a re-zero\n",
          error_count_);
      state_ = UNINITIALIZED;
      loop_->Reset();
    }
  }
  if (position != NULL) {
    last_good_time_ = ::aos::time::Time::Now();
    error_count_ = 0;
  }

  // Compute the absolute position of the wrist.
  double absolute_position;
  if (position) {
    absolute_position = position->position;
    if (state_ == READY) {
      absolute_position -= zero_offset_;
    }
    loop_->Y << absolute_position;
    if (!AnySensorsActive(position)) {
      last_off_position_ = position->position;
    }
  } else {
    // Dead recon for now.
    absolute_position = loop_->X_hat(0, 0);
  }

  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "UNINITIALIZED\n");
      if (position) {
        // Reset the zeroing goal.
        zeroing_position_ = absolute_position;
        // Clear the observer state.
        loop_->X_hat << absolute_position, 0.0, 0.0;
        loop_->ZeroPower();
        // Set the goal to here to make it so it doesn't move when disabled.
        loop_->R = loop_->X_hat;
        // Only progress if we are enabled.
        if (::aos::joystick_state->enabled) {
          if (AnySensorsActive(position)) {
            state_ = MOVING_OFF;
          } else {
            state_ = ZEROING;
          }
        }
      }
      break;
    case MOVING_OFF:
      LOG(DEBUG, "MOVING_OFF\n");
      {
        // Move off the hall effect sensor.
        if (!::aos::joystick_state->enabled) {
          // Start over if disabled.
          state_ = UNINITIALIZED;
        } else if (position && !AnySensorsActive(position)) {
          // We are now off the sensor.  Time to zero now.
          state_ = ZEROING;
        } else {
          // Slowly creep off the sensor.
          zeroing_position_ -= config_data_.zeroing_off_speed * dt;
          loop_->R << zeroing_position_, -config_data_.zeroing_off_speed, 0.0;
          break;
        }
      }
    case ZEROING:
      LOG(DEBUG, "ZEROING\n");
      {
        int active_sensor_index = ActiveSensorIndex(position);
        if (!::aos::joystick_state->enabled) {
          // Start over if disabled.
          state_ = UNINITIALIZED;
        } else if (position && active_sensor_index != -1) {
          state_ = READY;
          // Verify that the calibration number is between the last off position
          // and the current on position.  If this is not true, move off and try
          // again.
          const double calibration =
              position->hall_effect_positions[active_sensor_index];
          if (!is_between(last_off_position_, position->position, 
                          calibration)) {
            LOG(ERROR, "Got a bogus calibration number.  Trying again.\n");
            LOG(ERROR,
                "Last off position was %f, current is %f, calibration is %f\n",
                last_off_position_, position->position,
                position->hall_effect_positions[active_sensor_index]);
            state_ = MOVING_OFF;
          } else {
            // Save the zero, and then offset the observer to deal with the
            // phantom step change.
            const double old_zero_offset = zero_offset_;
            zero_offset_ =
                position->hall_effect_positions[active_sensor_index] -
                config_data_.hall_effect_start_angle[active_sensor_index];
            loop_->X_hat(0, 0) += old_zero_offset - zero_offset_;
            loop_->Y(0, 0) += old_zero_offset - zero_offset_;
          }
        } else {
          // Slowly creep towards the sensor.
          zeroing_position_ += config_data_.zeroing_speed * dt;
          loop_->R << zeroing_position_, config_data_.zeroing_speed, 0.0;
        }
        break;
      }

    case READY:
      LOG(DEBUG, "READY\n");
      {
        const double limited_goal = ClipGoal(goal_angle);
        loop_->R << limited_goal, goal_velocity, 0.0;
        break;
      }

    case ESTOP:
      LOG(DEBUG, "ESTOP\n");
      LOG(WARNING, "have already given up\n");
      return 0.0;
  }

  // Update the observer.
  loop_->Update(position != NULL, !output_enabled);

  LOG(DEBUG, "X_hat={%f, %f, %f}\n",
      loop_->X_hat(0, 0), loop_->X_hat(1, 0), loop_->X_hat(2, 0));

  capped_goal_ = false;
  // Verify that the zeroing goal hasn't run away.
  switch (state_) {
    case UNINITIALIZED:
    case READY:
    case ESTOP:
      // Not zeroing.  No worries.
      break;
    case MOVING_OFF:
    case ZEROING:
      // Check if we have cliped and adjust the goal.
      if (loop_->uncapped_voltage() > config_data_.max_zeroing_voltage) {
        double dx = (loop_->uncapped_voltage() -
                     config_data_.max_zeroing_voltage) / loop_->K(0, 0);
        zeroing_position_ -= dx;
        capped_goal_ = true;
      } else if(loop_->uncapped_voltage() < -config_data_.max_zeroing_voltage) {
        double dx = (loop_->uncapped_voltage() +
                     config_data_.max_zeroing_voltage) / loop_->K(0, 0);
        zeroing_position_ -= dx;
        capped_goal_ = true;
      }
      break;
  }
  if (output_enabled) {
    double voltage = loop_->voltage();
    if (voltage > 0) {
      voltage += config_data_.deadband_voltage;
    } else if (voltage < 0) {
      voltage -= config_data_.deadband_voltage;
    }
    if (voltage > 12.0) {
      voltage = 12.0;
    } else if (voltage < -12.0) {
      voltage = -12.0;
    }
    return voltage;
  } else {
    return 0.0;
  }
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ZEROED_JOINT_H_
