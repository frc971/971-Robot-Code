#ifndef MOTORS_SEEMS_REASONABLE_SPRING_H_
#define MOTORS_SEEMS_REASONABLE_SPRING_H_

#include <cmath>

namespace motors {
namespace seems_reasonable {

float NextGoal(float current_goal, float goal);
float PreviousGoal(float current_goal, float goal);

class Spring {
 public:
  Spring() = default;
  Spring(const Spring &) = delete;
  Spring &operator=(const Spring &) = delete;

  // Iterates the loop.
  // If unload is true, unload.
  // If the encoder isn't valid, unload.
  // If prime is true, go to primed state.
  // If prime and fire are true, fire.
  void Iterate(bool unload, bool prime, bool fire, bool force_reset,
               bool force_move, bool encoder_valid, float angle);

  enum class State {
    UNINITIALIZED = 0,
    STUCK_UNLOAD = 1,
    UNLOAD = 2,
    LOAD = 3,
    PRIME = 4,
    FIRE = 5,
    WAIT_FOR_LOAD = 6,
    WAIT_FOR_LOAD_RELEASE = 7,
    FORCE_MOVE = 8,
  };

  // Returns the current to output to the spring motors.
  float output() const { return output_; }

  // Returns true if the motor is near the goal.
  bool Near() { return ::std::abs(angle_ - goal_) < 0.2f; }

  State state() const { return state_; }

  float angle() const { return angle_; }
  float goal() const { return goal_; }

  int timeout() const { return timeout_; }

 private:
  void Load() {
    timeout_ = 5 * 200;
    state_ = State::LOAD;
  }

  void Prime() {
    timeout_ = 1 * 200;
    state_ = State::PRIME;
  }

  void ForceMove() {
    timeout_ = 0;
    state_ = State::FORCE_MOVE;
  }

  void Unload() {
    timeout_ = 14 * 200;
    state_ = State::UNLOAD;
  }

  void StuckUnload() {
    timeout_ = 14 * 200;
    state_ = State::STUCK_UNLOAD;
  }

  void Fire() {
    timeout_ = 100;
    state_ = State::FIRE;
  }

  float NextGoal(float goal) {
    return ::motors::seems_reasonable::NextGoal(goal_, goal);
  }

  float PreviousGoal(float goal) {
    return ::motors::seems_reasonable::PreviousGoal(goal_, goal);
  }

  State state_ = State::UNINITIALIZED;

  // Note, these need to be (-M_PI, M_PI]
  constexpr static float kLoadGoal = -0.345f;
  constexpr static float kPrimeGoal = -0.269f;
  constexpr static float kFireGoal = -0.063f;
  constexpr static float kUnloadGoal = kFireGoal;

  float angle_ = 0.0f;
  float goal_ = 0.0f;

  int timeout_ = 0;

  float output_ = 0.0f;
  float last_error_ = 0.0f;
};

}  // namespace seems_reasonable
}  // namespace motors

#endif  //  MOTORS_SEEMS_REASONABLE_SPRING_H_
