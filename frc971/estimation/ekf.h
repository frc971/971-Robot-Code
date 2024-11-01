#ifndef FRC971_ESTIMATION_EKF_H_
#define FRC971_ESTIMATION_EKF_H_

#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/control_loops/swerve/auto_diff_jacobian.h"
#include "frc971/control_loops/swerve/linearization_utils.h"

namespace frc971::control_loops {

// This class implements an EKF (Extended Kalman Filter) for a system of the
// form:
// dX / dt = f(X, U) + w
// For some state X with kNumStates elements, some input U with kNumInputs
// elements, and some noise w.
// Measurements will be of the form:
// Z = h(X) + v
// For some measurement Z, state X, and measurement noise v.
// In order to implement the EKF, we then linearize all of these methods
// about X, producing linearized dynamics:
// dX / dt = A * X + B * U + N(Q)
// Z = H * X + N(R)
// where:
// H = dh / dX
// A = df / dX
// B = df / dU
// and N(C) is a gaussian distribution with zero mean and covariance C.
//
// When we perform the prediction step of the Kalman filter, we use a
// runge-kutta integration of f(); to calculate the covariance updates and to
// calculate the gains for the filter, we use all of the linearized matrices.
//
// kNumStates will be the size of X, kNumInputs is the size of U.
template <typename Scalar, int kNumStates, int kNumInputs>
class Ekf {
 public:
  typedef Eigen::Matrix<Scalar, kNumStates, 1> State;
  typedef Eigen::Matrix<Scalar, kNumStates, kNumStates> StateSquare;
  typedef Eigen::Matrix<Scalar, kNumInputs, 1> Input;
  typedef control_loops::swerve::DynamicsInterface<Scalar, kNumStates,
                                                   kNumInputs>
      Dynamics;

  struct Params {
    StateSquare Q_continuous;
    std::unique_ptr<Dynamics> dynamics;
  };

  Ekf(Params params)
      : dynamics_(std::move(params.dynamics)),
        Q_continuous_(params.Q_continuous),
        P_(StateSquare::Zero()) {}

  // Initializes the filter to a certain state. This can be used to get the
  // state initialized to something reasonable on startup to prevent massive
  // corrections on startup.
  void Initialize(const aos::monotonic_clock::time_point now,
                  const State &initial_state, const StateSquare &P) {
    last_update_ = now;
    X_hat_ = initial_state;
    P_ = P;
  }

  // Debug information associated with a single Correct step.
  template <int kNumMeasurements>
  struct CorrectionDebug {
    // The actual measurement associated with the correction.
    Eigen::Matrix<Scalar, kNumMeasurements, 1> measurement;
    // The expected measurement when we did the correction.
    Eigen::Matrix<Scalar, kNumMeasurements, 1> expected;
  };

  // Performs a predict & correct step for a given measurement.
  // now is the time corresponding to when the measurement was taken. Time must
  //     pass monotonically, although multiple measurements may occur at the
  //     same time.
  // measurement is the values of the actual measurement.
  // R is the covariance of the measurement noise.
  // U is the current control input.
  // expected_measurement is something which can be called as
  //     expected_measurement(Eigen::Map<const State>) and returns a
  //     Measurement.
  // H is the jacobian of expected_measurement.
  template <int kNumMeasurements, typename ExpectedMeasurementFunction,
            typename Measurement = Eigen::Matrix<Scalar, kNumMeasurements, 1>,
            typename MeasurementSquare =
                Eigen::Matrix<Scalar, kNumMeasurements, kNumMeasurements>>
  CorrectionDebug<kNumMeasurements> Correct(
      const aos::monotonic_clock::time_point now,
      const Measurement &measurement, const MeasurementSquare &R,
      const Input &U, const ExpectedMeasurementFunction &expected_measurement,
      const Eigen::Matrix<Scalar, kNumMeasurements, kNumStates> &H) {
    if (!last_update_.has_value()) {
      // TODO(james): May want to initialize more intelligently to avoid really
      // extreme corrections.
      last_update_ = now;
    }
    CHECK_LE(last_update_.value(), now);
    const aos::monotonic_clock::duration dt = now - last_update_.value();
    StateSquare Q_discrete, A_discrete;
    // Note: We don't actually use the linearized B matrix, but the cost of
    // calculating said matrix is reasonably low in most practical scenarios.
    const typename Dynamics::LinearDynamics linearized_dynamics =
        dynamics_->LinearizeDynamics(X_hat_, U);
    controls::DiscretizeQAFast(Q_continuous_, linearized_dynamics.A, dt,
                               &Q_discrete, &A_discrete);
    VLOG(3) << "Discretized Q\n" << Q_discrete;
    // Only do a predict step if time actually passed; this optimizes things in
    // the scenario where we do multiple correction steps at once.
    if (dt.count() != 0) {
      Predict(dt, U, A_discrete, Q_discrete);
      last_update_ = now;
    }

    const Measurement expected =
        expected_measurement(Eigen::Map<const State>(X_hat_.data()));
    const Eigen::Matrix<Scalar, kNumStates, kNumMeasurements> K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    P_ = (StateSquare::Identity() - K * H) * P_;
    VLOG(3) << "K\n" << K;
    const State update = K * (measurement - expected);
    VLOG(3) << "correction update\n" << update;
    X_hat_ += update;
    return {.measurement = measurement, .expected = expected};
  }

  // Uses Ceres to calculate the Jacobian of expected_measurement and calls
  // Correct().
  template <int kNumMeasurements, typename ExpectedMeasurementFunction,
            typename Measurement = Eigen::Matrix<Scalar, kNumMeasurements, 1>,
            typename MeasurementSquare =
                Eigen::Matrix<Scalar, kNumMeasurements, kNumMeasurements>>
  CorrectionDebug<kNumMeasurements> Correct(
      const aos::monotonic_clock::time_point now,
      const Measurement &measurement, const MeasurementSquare &R,
      const Input &U, const ExpectedMeasurementFunction &expected_measurement) {
    return Correct(
        now, measurement, R, U, expected_measurement,
        control_loops::swerve::AutoDiffJacobian<
            Scalar, ExpectedMeasurementFunction, kNumStates,
            kNumMeasurements>::Jacobian(expected_measurement, X_hat_));
  }

  const State &X_hat() const { return X_hat_; }
  const StateSquare &P() const { return P_; }

 private:
  void Predict(const aos::monotonic_clock::duration dt, const Input &U,
               const StateSquare &A_discrete, const StateSquare &Q_discrete) {
    const State updated_state = control_loops::RungeKuttaU(
        [this](const State &X, const Input &U) { return (*dynamics_)(X, U); },
        X_hat_, U, aos::time::DurationInSeconds(dt));
    VLOG(3) << "predict update\n" << updated_state - X_hat_;
    X_hat_ = updated_state;
    P_ = A_discrete * P_ * A_discrete.transpose() + Q_discrete;
  }
  std::unique_ptr<Dynamics> dynamics_;
  StateSquare Q_continuous_;
  std::optional<aos::monotonic_clock::time_point> last_update_;
  State X_hat_;
  StateSquare P_;
};

}  // namespace frc971::control_loops
#endif  // FRC971_ESTIMATION_EKF_H_
