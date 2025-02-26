#ifndef FRC971_ESTIMAITON_KF_H_
#define FRC971_ESTIMAITON_KF_H_

#include "Eigen/Dense"
#include "absl/log/log.h"

#include "frc971/control_loops/c2d.h"

// Implements a simple kalman filter with an unknown input. Basically just
// implements the wikipedia page on Kalman Filters
// https://en.wikipedia.org/wiki/Kalman_filter#Details.

// TODO(max): There are ways of approaching this by basically modelling the
// forces when you assume u is unknown. It may be good to see if this improves
// performance, but I doubt it.

// TODO(max): We really need to make a test for this similar to what we do for
// the ekf. It likely won't catch most or all of our issues but its useful to at
// least make sure its sane.
namespace frc971::estimation {

template <int kNumStates, typename Scalar = double>
class Kf {
 public:
  typedef Eigen::Matrix<Scalar, kNumStates, 1> State;
  typedef Eigen::Matrix<Scalar, kNumStates, kNumStates> StateSquare;

  Kf(StateSquare Q_continuous, StateSquare A_continuous,
     aos::monotonic_clock::time_point now,
     StateSquare initial_P = StateSquare::Zero(),
     State initial_X_hat = State::Zero())
      : Q_continuous_(Q_continuous), A_continuous_(A_continuous) {
    Reset(now, initial_P, initial_X_hat);
  }

  void Reset(aos::monotonic_clock::time_point now,
             StateSquare initial_P = StateSquare::Zero(),
             State initial_X_hat = State::Zero()) {
    X_hat_ = initial_X_hat;
    P_ = initial_P;

    last_update_time_ = now;
  }

  template <int kNumObservations,
            typename Observation = Eigen::Matrix<Scalar, kNumObservations, 1>,
            typename ObservationSquare =
                Eigen::Matrix<Scalar, kNumObservations, kNumObservations>>
  void Correct(Observation observation,
               Eigen::Matrix<Scalar, kNumObservations, kNumStates> H,
               ObservationSquare R, aos::monotonic_clock::time_point now) {
    aos::monotonic_clock::duration dt = now - last_update_time_;

    if (dt.count() == 0) {
      VLOG(1) << "Correct called and dt = 0, skipping prediction step.";
    } else {
      StateSquare Q_discrete, A_discrete;

      frc971::controls::DiscretizeQAFast(Q_continuous_, A_continuous_, dt,
                                         &Q_discrete, &A_discrete);

      Predict(Q_discrete, A_discrete);
    }

    const Observation expected = H * X_hat_;

    const Eigen::Matrix<Scalar, kNumStates, kNumObservations> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

    const State update = K * (observation - expected);

    P_ = (StateSquare::Identity() - K * H) * P_;

    X_hat_ += update;

    last_update_time_ = now;
  }

  const State X_hat() { return X_hat_; }

  // This is exposed in order to normalize our angle after running position and
  // omega through the correct step and avoid floating point issues with
  // exploding thetas. In the future we should probably do this by storing the
  // sin and cos or rotation matrix in the state instead to avoid absolute
  // thetas.
  State *X_hat_mut() { return &X_hat_; }
  const StateSquare P() { return P_; }

 private:
  void Predict(StateSquare Q_discrete, StateSquare A_discrete) {
    X_hat_ = A_discrete * X_hat_;
    P_ = A_discrete * P_ * A_discrete.transpose() + Q_discrete;
  }

  State X_hat_;

  StateSquare Q_continuous_;
  StateSquare A_continuous_;
  StateSquare P_;

  aos::monotonic_clock::time_point last_update_time_;
};

}  // namespace frc971::estimation

#endif  // FRC971_ESTIMAITON_KF_H_
