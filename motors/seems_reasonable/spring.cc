#include "motors/seems_reasonable/spring.h"

#include "frc971/zeroing/wrap.h"

#include <cmath>

namespace motors {
namespace seems_reasonable {
namespace {

constexpr float kTwoPi = 2.0 * M_PI;

}  // namespace

float NextGoal(float current_goal, float goal) {
  float remainder = remainderf(current_goal - goal, kTwoPi);
  if (remainder >= 0.0f) {
    remainder -= kTwoPi;
  }
  return -remainder + current_goal;
}

float PreviousGoal(float current_goal, float goal) {
  float remainder = remainderf(current_goal - goal, kTwoPi);
  if (remainder <= 0.0f) {
    remainder += kTwoPi;
  }
  return -remainder + current_goal;
}

void Spring::Iterate(bool unload, bool prime, bool fire, bool force_reset,
                     bool force_move, bool encoder_valid, float angle) {
  // Angle is +- M_PI.  So, we need to find the nearest angle to the previous
  // one, and that's our new angle.
  angle_ = ::frc971::zeroing::Wrap(angle_, angle, kTwoPi);

  switch (state_) {
    case State::UNINITIALIZED:
      // Go to the previous unload from where we are.
      goal_ = angle_;
      goal_ = PreviousGoal(kUnloadGoal);
      if (force_move) {
        ForceMove();
      } else if (prime && fire) {
        Unload();
      }
      break;
    case State::FORCE_MOVE:
      if (!force_move) {
        state_ = State::UNINITIALIZED;
      }
      break;
    case State::UNLOAD:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        state_ = State::STUCK_UNLOAD;
      } else if (!unload && prime && fire) {
        // Go to the next goal from the current location.  This handles if we
        // fired or didn't on the previous cycle.
        goal_ = angle_;
        goal_ = NextGoal(kLoadGoal);
        Load();
      }
      // TODO(Austin): This should be a break, right?
      // fallthrough
    case State::STUCK_UNLOAD:
      if (force_move) {
        ForceMove();
      } else if (force_reset && encoder_valid &&
                 state_ == State::STUCK_UNLOAD) {
        state_ = State::UNINITIALIZED;
      } else if (timeout_ > 0) {
        --timeout_;
      }
      break;
    case State::LOAD:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        goal_ = PreviousGoal(kUnloadGoal);
        StuckUnload();
      } else if (unload) {
        goal_ = PreviousGoal(kUnloadGoal);
        Unload();
      } else if (!Near()) {
        if (timeout_ > 0) {
          --timeout_;
        } else {
          StuckUnload();
        }
      } else if (prime) {
        goal_ = NextGoal(kPrimeGoal);
        Prime();
      }
      break;
    case State::PRIME:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        goal_ = PreviousGoal(kUnloadGoal);
        StuckUnload();
      } else if (unload) {
        goal_ = PreviousGoal(kUnloadGoal);
        Unload();
      } else if (!prime) {
        goal_ = PreviousGoal(kLoadGoal);
        Load();
      } else if (!Near()) {
        if (timeout_ > 0) {
          --timeout_;
        } else {
          StuckUnload();
        }
      } else if (fire) {
        goal_ = NextGoal(kFireGoal);
        Fire();
      }
      break;

    case State::FIRE:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        goal_ = PreviousGoal(kUnloadGoal);
        StuckUnload();
      } else if (!Near()) {
        if (timeout_ > 0) {
          --timeout_;
        } else {
          StuckUnload();
        }
      } else {
        // TODO(austin): Maybe have a different timeout for success.
        if (timeout_ > 0) {
          timeout_--;
        } else if (!prime) {
          state_ = State::WAIT_FOR_LOAD;
        }
      }
      break;
    case State::WAIT_FOR_LOAD:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        StuckUnload();
      } else if (unload) {
        // Goal is as good as it is going to get since unload is the fire
        // position.
        Unload();
      } else if (prime) {
        state_ = State::WAIT_FOR_LOAD_RELEASE;
      }
      break;
    case State::WAIT_FOR_LOAD_RELEASE:
      if (force_move) {
        ForceMove();
      } else if (!encoder_valid) {
        StuckUnload();
      } else if (unload) {
        // Goal is as good as it is going to get since unload is the fire
        // position.
        Unload();
      } else if (!prime) {
        Load();
        goal_ = NextGoal(kLoadGoal);
      }
      break;
  }
  const float error = goal_ - angle_;
  const float derror = (error - last_error_) * 200.0f;

  switch (state_) {
    case State::UNINITIALIZED:
      output_ = 0.0f;
      break;
    case State::STUCK_UNLOAD:
    case State::UNLOAD:
      if (timeout_ > 0) {
        output_ = -0.1f;
      } else {
        output_ = 0.0f;
      }
      break;

    case State::FORCE_MOVE:
      output_ = 1.0f;
      break;

    case State::LOAD:
    case State::PRIME:
    case State::FIRE:
    case State::WAIT_FOR_LOAD:
    case State::WAIT_FOR_LOAD_RELEASE: {
      constexpr float kP = 3.00f;
      constexpr float kD = 0.00f;
      output_ = kP * error + kD * derror;
    } break;
  }
  last_error_ = error;
}

}  // namespace seems_reasonable
}  // namespace motors
