#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_

#include <chrono>

#include "Eigen/Dense"
#include "aos/containers/priority_queue.h"
#include "aos/util/math.h"
#include "frc971/control_loops/c2d.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/runge_kutta.h"

namespace y2019 {
namespace control_loops {
namespace testing {
class ParameterizedLocalizerTest;
}  // namespace testing
}  // namespace control_loops
}  // namespace y2019

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace testing {
class HybridEkfTest;
}

// HybridEkf is an EKF for use in robot localization. It is currently
// coded for use with drivetrains in particular, and so the states and inputs
// are chosen as such.
// The "Hybrid" part of the name refers to the fact that it can take in
// measurements with variable time-steps.
// measurements can also have been taken in the past and we maintain a buffer
// so that we can replay the kalman filter whenever we get an old measurement.
// Currently, this class provides the necessary utilities for doing
// measurement updates with an encoder/gyro as well as a more generic
// update function that can be used for arbitrary nonlinear updates (presumably
// a camera update).
template <typename Scalar = double>
class HybridEkf {
 public:
  // An enum specifying what each index in the state vector is for.
  enum StateIdx {
    kX = 0,
    kY = 1,
    kTheta = 2,
    kLeftEncoder = 3,
    kLeftVelocity = 4,
    kRightEncoder = 5,
    kRightVelocity = 6,
    kLeftVoltageError = 7,
    kRightVoltageError = 8,
    kAngularError = 9,
  };
  static constexpr int kNStates = 10;
  static constexpr int kNInputs = 2;
  // Number of previous samples to save.
  static constexpr int kSaveSamples = 50;
  // Assume that all correction steps will have kNOutputs
  // dimensions.
  // TODO(james): Relax this assumption; relaxing it requires
  // figuring out how to deal with storing variable size
  // observation matrices, though.
  static constexpr int kNOutputs = 3;
  // Inputs are [left_volts, right_volts]
  typedef Eigen::Matrix<Scalar, kNInputs, 1> Input;
  // Outputs are either:
  // [left_encoder, right_encoder, gyro_vel]; or [heading, distance, skew] to
  // some target. This makes it so we don't have to figure out how we store
  // variable-size measurement updates.
  typedef Eigen::Matrix<Scalar, kNOutputs, 1> Output;
  typedef Eigen::Matrix<Scalar, kNStates, kNStates> StateSquare;
  // State is [x_position, y_position, theta, Kalman States], where
  // Kalman States are the states from the standard drivetrain Kalman Filter,
  // which is: [left encoder, left ground vel, right encoder, right ground vel,
  // left voltage error, right voltage error, angular_error], where:
  // left/right encoder should correspond directly to encoder readings
  // left/right velocities are the velocity of the left/right sides over the
  //   ground (i.e., corrected for angular_error).
  // voltage errors are the difference between commanded and effective voltage,
  //   used to estimate consistent modelling errors (e.g., friction).
  // angular error is the difference between the angular velocity as estimated
  //   by the encoders vs. estimated by the gyro, such as might be caused by
  //   wheels on one side of the drivetrain being too small or one side's
  //   wheels slipping more than the other.
  typedef Eigen::Matrix<Scalar, kNStates, 1> State;

  // Constructs a HybridEkf for a particular drivetrain.
  // Currently, we use the drivetrain config for modelling constants
  // (continuous time A and B matrices) and for the noise matrices for the
  // encoders/gyro.
  HybridEkf(const DrivetrainConfig<Scalar> &dt_config)
      : dt_config_(dt_config),
        velocity_drivetrain_coefficients_(
            dt_config.make_hybrid_drivetrain_velocity_loop()
                .plant()
                .coefficients()) {
    InitializeMatrices();
  }

  // Set the initial guess of the state. Can only be called once, and before
  // any measurement updates have occured.
  // TODO(james): We may want to actually re-initialize and reset things on
  // the field. Create some sort of Reset() function.
  void ResetInitialState(::aos::monotonic_clock::time_point t,
                         const State &state, const StateSquare &P) {
    observations_.clear();
    X_hat_ = state;
    have_zeroed_encoders_ = true;
    P_ = P;
    observations_.PushFromBottom(
        {t,
         t,
         X_hat_,
         P_,
         Input::Zero(),
         Output::Zero(),
         {},
         [](const State &, const Input &) { return Output::Zero(); },
         [](const State &) {
           return Eigen::Matrix<Scalar, kNOutputs, kNStates>::Zero();
         },
         Eigen::Matrix<Scalar, kNOutputs, kNOutputs>::Identity()});
  }

  // Correct with:
  // A measurement z at time t with z = h(X_hat, U) + v where v has noise
  // covariance R.
  // Input U is applied from the previous timestep until time t.
  // If t is later than any previous measurements, then U must be provided.
  // If the measurement falls between two previous measurements, then U
  // can be provided or not; if U is not provided, then it is filled in based
  // on an assumption that the voltage was held constant between the time steps.
  // TODO(james): Is it necessary to explicitly to provide a version with H as a
  // matrix for linear cases?
  void Correct(
      const Output &z, const Input *U,
      ::std::function<
          void(const State &, const StateSquare &,
               ::std::function<Output(const State &, const Input &)> *,
               ::std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(
                   const State &)> *)>
          make_h,
      ::std::function<Output(const State &, const Input &)> h,
      ::std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
          dhdx,
      const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
      aos::monotonic_clock::time_point t);

  // A utility function for specifically updating with encoder and gyro
  // measurements.
  void UpdateEncodersAndGyro(const Scalar left_encoder,
                             const Scalar right_encoder, const Scalar gyro_rate,
                             const Input &U,
                             ::aos::monotonic_clock::time_point t) {
    // Because the check below for have_zeroed_encoders_ will add an
    // Observation, do a check here to ensure that initialization has been
    // performed and so there is at least one observation.
    AOS_CHECK(!observations_.empty());
    if (!have_zeroed_encoders_) {
      // This logic handles ensuring that on the first encoder reading, we
      // update the internal state for the encoders to match the reading.
      // Otherwise, if we restart the drivetrain without restarting
      // wpilib_interface, then we can get some obnoxious initial corrections
      // that mess up the localization.
      State newstate = X_hat_;
      newstate(kLeftEncoder, 0) = left_encoder;
      newstate(kRightEncoder, 0) = right_encoder;
      newstate(kLeftVoltageError, 0) = 0.0;
      newstate(kRightVoltageError, 0) = 0.0;
      newstate(kAngularError, 0) = 0.0;
      ResetInitialState(t, newstate, P_);
      // We need to set have_zeroed_encoders_ after ResetInitialPosition because
      // the reset clears have_zeroed_encoders_...
      have_zeroed_encoders_ = true;
    }
    Output z(left_encoder, right_encoder, gyro_rate);
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;
    R.setZero();
    R.diagonal() << encoder_noise_, encoder_noise_, gyro_noise_;
    Correct(z, &U, {},
            [this](const State &X, const Input &) {
              return H_encoders_and_gyro_ * X;
            },
            [this](const State &) { return H_encoders_and_gyro_; }, R, t);
  }

  // Sundry accessor:
  State X_hat() const { return X_hat_; }
  Scalar X_hat(long i) const { return X_hat_(i, 0); }
  StateSquare P() const { return P_; }
  ::aos::monotonic_clock::time_point latest_t() const {
    return observations_.top().t;
  }

 private:
  struct Observation {
    // Time when the observation was taken.
    aos::monotonic_clock::time_point t;
    // Time that the previous observation was taken:
    aos::monotonic_clock::time_point prev_t;
    // Estimate of state at previous observation time t, after accounting for
    // the previous observation.
    State X_hat;
    // Noise matrix corresponding to X_hat_.
    StateSquare P;
    // The input applied from previous observation until time t.
    Input U;
    // Measurement taken at that time.
    Output z;
    // A function to create h and dhdx from a given position/covariance
    // estimate. This is used by the camera to make it so that we only have to
    // match targets once.
    // Only called if h and dhdx are empty.
    ::std::function<void(
        const State &, const StateSquare &,
        ::std::function<Output(const State &, const Input &)> *,
        ::std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(
            const State &)> *)>
        make_h;
    // A function to calculate the expected output at a given state/input.
    // TODO(james): For encoders/gyro, it is linear and the function call may
    // be expensive. Potential source of optimization.
    ::std::function<Output(const State &, const Input &)> h;
    // The Jacobian of h with respect to x.
    // We assume that U has no impact on the Jacobian.
    // TODO(james): Currently, none of the users of this actually make use of
    // the ability to have dynamic dhdx (technically, the camera code should
    // recalculate it to be strictly correct, but I was both too lazy to do
    // so and it seemed unnecessary). This is a potential source for future
    // optimizations if function calls are being expensive.
    ::std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
        dhdx;
    // The measurement noise matrix.
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;

    // In order to sort the observations in the PriorityQueue object, we
    // need a comparison function.
    friend bool operator<(const Observation &l, const Observation &r) {
      return l.t < r.t;
    }
  };

  void InitializeMatrices();

  StateSquare AForState(const State &X) const {
    StateSquare A_continuous = A_continuous_;
    const Scalar theta = X(kTheta, 0);
    const Scalar linear_vel =
        (X(kLeftVelocity, 0) + X(kRightVelocity, 0)) / 2.0;
    const Scalar stheta = ::std::sin(theta);
    const Scalar ctheta = ::std::cos(theta);
    // X and Y derivatives
    A_continuous(kX, kTheta) = -stheta * linear_vel;
    A_continuous(kX, kLeftVelocity) = ctheta / 2.0;
    A_continuous(kX, kRightVelocity) = ctheta / 2.0;
    A_continuous(kY, kTheta) = ctheta * linear_vel;
    A_continuous(kY, kLeftVelocity) = stheta / 2.0;
    A_continuous(kY, kRightVelocity) = stheta / 2.0;
    return A_continuous;
  }

  State DiffEq(const State &X, const Input &U) const {
    State Xdot = A_continuous_ * X + B_continuous_ * U;
    // And then we need to add on the terms for the x/y change:
    const Scalar theta = X(kTheta, 0);
    const Scalar linear_vel =
        (X(kLeftVelocity, 0) + X(kRightVelocity, 0)) / 2.0;
    const Scalar stheta = ::std::sin(theta);
    const Scalar ctheta = ::std::cos(theta);
    Xdot(kX, 0) = ctheta * linear_vel;
    Xdot(kY, 0) = stheta * linear_vel;
    return Xdot;
  }

  void PredictImpl(const Input &U, std::chrono::nanoseconds dt, State *state,
                   StateSquare *P) {
    StateSquare A_c = AForState(*state);
    StateSquare A_d;
    StateSquare Q_d;
    controls::DiscretizeQAFast(Q_continuous_, A_c, dt, &Q_d, &A_d);

    *state = RungeKuttaU(
        [this](const State &X, const Input &U) { return DiffEq(X, U); }, *state,
        U, ::aos::time::DurationInSeconds(dt));

    StateSquare Ptemp = A_d * *P * A_d.transpose() + Q_d;
    *P = Ptemp;
  }

  void CorrectImpl(const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
                   const Output &Z, const Output &expected_Z,
                   const Eigen::Matrix<Scalar, kNOutputs, kNStates> &H,
                   State *state, StateSquare *P) {
    Output err = Z - expected_Z;
    Eigen::Matrix<Scalar, kNStates, kNOutputs> PH = *P * H.transpose();
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> S = H * PH + R;
    Eigen::Matrix<Scalar, kNStates, kNOutputs> K = PH * S.inverse();
    *state += K * err;
    StateSquare Ptemp = (StateSquare::Identity() - K * H) * *P;
    *P = Ptemp;
  }

  void ProcessObservation(Observation *obs, const std::chrono::nanoseconds dt,
                          State *state, StateSquare *P) {
    *state = obs->X_hat;
    *P = obs->P;
    if (dt.count() != 0) {
      PredictImpl(obs->U, dt, state, P);
    }
    if (!(obs->h && obs->dhdx)) {
      AOS_CHECK(obs->make_h);
      obs->make_h(*state, *P, &obs->h, &obs->dhdx);
    }
    CorrectImpl(obs->R, obs->z, obs->h(*state, obs->U), obs->dhdx(*state),
                state, P);
  }

  DrivetrainConfig<Scalar> dt_config_;
  State X_hat_;
  StateFeedbackHybridPlantCoefficients<2, 2, 2, Scalar>
      velocity_drivetrain_coefficients_;
  StateSquare A_continuous_;
  StateSquare Q_continuous_;
  StateSquare P_;
  Eigen::Matrix<Scalar, kNOutputs, kNStates> H_encoders_and_gyro_;
  Scalar encoder_noise_, gyro_noise_;
  Eigen::Matrix<Scalar, kNStates, kNInputs> B_continuous_;

  bool have_zeroed_encoders_ = false;

  aos::PriorityQueue<Observation, kSaveSamples, ::std::less<Observation>>
      observations_;

  friend class testing::HybridEkfTest;
  friend class ::y2019::control_loops::testing::ParameterizedLocalizerTest;
};  // class HybridEkf

template <typename Scalar>
void HybridEkf<Scalar>::Correct(
    const Output &z, const Input *U,
    ::std::function<
        void(const State &, const StateSquare &,
             ::std::function<Output(const State &, const Input &)> *,
             ::std::function<
                 Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)> *)>
        make_h,
    ::std::function<Output(const State &, const Input &)> h,
    ::std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
        dhdx,
    const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
    aos::monotonic_clock::time_point t) {
  AOS_CHECK(!observations_.empty());
  if (!observations_.full() && t < observations_.begin()->t) {
    AOS_LOG(ERROR,
            "Dropped an observation that was received before we "
            "initialized.\n");
    return;
  }
  auto cur_it =
      observations_.PushFromBottom({t, t, State::Zero(), StateSquare::Zero(),
                                    Input::Zero(), z, make_h, h, dhdx, R});
  if (cur_it == observations_.end()) {
    AOS_LOG(
        DEBUG,
        "Camera dropped off of end with time of %fs; earliest observation in "
        "queue has time of %fs.\n",
        ::aos::time::DurationInSeconds(t.time_since_epoch()),
        ::aos::time::DurationInSeconds(
            observations_.begin()->t.time_since_epoch()));
    return;
  }

  // Now we populate any state information that depends on where the
  // observation was inserted into the queue. X_hat and P must be populated
  // from the values present in the observation *following* this one in
  // the queue (note that the X_hat and P that we store in each observation
  // is the values that they held after accounting for the previous
  // measurement and before accounting for the time between the previous and
  // current measurement). If we appended to the end of the queue, then
  // we need to pull from X_hat_ and P_ specifically.
  // Furthermore, for U:
  // -If the observation was inserted at the end, then the user must've
  //  provided U and we use it.
  // -Otherwise, only grab U if necessary.
  auto next_it = cur_it;
  ++next_it;
  if (next_it == observations_.end()) {
    cur_it->X_hat = X_hat_;
    cur_it->P = P_;
    // Note that if next_it == observations_.end(), then because we already
    // checked for !observations_.empty(), we are guaranteed to have
    // valid prev_it.
    auto prev_it = cur_it;
    --prev_it;
    cur_it->prev_t = prev_it->t;
    // TODO(james): Figure out a saner way of handling this.
    AOS_CHECK(U != nullptr);
    cur_it->U = *U;
  } else {
    cur_it->X_hat = next_it->X_hat;
    cur_it->P = next_it->P;
    cur_it->prev_t = next_it->prev_t;
    next_it->prev_t = cur_it->t;
    cur_it->U = (U == nullptr) ? next_it->U : *U;
  }
  // Now we need to rerun the predict step from the previous to the new
  // observation as well as every following correct/predict up to the current
  // time.
  while (true) {
    // We use X_hat_ and P_ to store the intermediate states, and then
    // once we reach the end they will all be up-to-date.
    ProcessObservation(&*cur_it, cur_it->t - cur_it->prev_t, &X_hat_, &P_);
    AOS_CHECK(X_hat_.allFinite());
    if (next_it != observations_.end()) {
      next_it->X_hat = X_hat_;
      next_it->P = P_;
    } else {
      break;
    }
    ++cur_it;
    ++next_it;
  }
}

template <typename Scalar>
void HybridEkf<Scalar>::InitializeMatrices() {
  A_continuous_.setZero();
  const Scalar diameter = 2.0 * dt_config_.robot_radius;
  // Theta derivative
  A_continuous_(kTheta, kLeftVelocity) = -1.0 / diameter;
  A_continuous_(kTheta, kRightVelocity) = 1.0 / diameter;

  // Encoder derivatives
  A_continuous_(kLeftEncoder, kLeftVelocity) = 1.0;
  A_continuous_(kLeftEncoder, kAngularError) = 1.0;
  A_continuous_(kRightEncoder, kRightVelocity) = 1.0;
  A_continuous_(kRightEncoder, kAngularError) = -1.0;

  // Pull velocity derivatives from velocity matrices.
  // Note that this looks really awkward (doesn't use
  // Eigen blocks) because someone decided that the full
  // drivetrain Kalman Filter should half a weird convention.
  // TODO(james): Support shifting drivetrains with changing A_continuous
  const auto &vel_coefs = velocity_drivetrain_coefficients_;
  A_continuous_(kLeftVelocity, kLeftVelocity) = vel_coefs.A_continuous(0, 0);
  A_continuous_(kLeftVelocity, kRightVelocity) = vel_coefs.A_continuous(0, 1);
  A_continuous_(kRightVelocity, kLeftVelocity) = vel_coefs.A_continuous(1, 0);
  A_continuous_(kRightVelocity, kRightVelocity) = vel_coefs.A_continuous(1, 1);

  // Provide for voltage error terms:
  B_continuous_.setZero();
  B_continuous_.row(kLeftVelocity) = vel_coefs.B_continuous.row(0);
  B_continuous_.row(kRightVelocity) = vel_coefs.B_continuous.row(1);
  A_continuous_.template block<kNStates, kNInputs>(0, 7) = B_continuous_;

  Q_continuous_.setZero();
  // TODO(james): Improve estimates of process noise--e.g., X/Y noise can
  // probably be reduced when we are stopped because you rarely jump randomly.
  // Or maybe it's more appropriate to scale wheelspeed noise with wheelspeed,
  // since the wheels aren't likely to slip much stopped.
  Q_continuous_(kX, kX) = 0.002;
  Q_continuous_(kY, kY) = 0.002;
  Q_continuous_(kTheta, kTheta) = 0.0001;
  Q_continuous_(kLeftEncoder, kLeftEncoder) = ::std::pow(0.15, 2.0);
  Q_continuous_(kRightEncoder, kRightEncoder) = ::std::pow(0.15, 2.0);
  Q_continuous_(kLeftVelocity, kLeftVelocity) = ::std::pow(0.5, 2.0);
  Q_continuous_(kRightVelocity, kRightVelocity) = ::std::pow(0.5, 2.0);
  Q_continuous_(kLeftVoltageError, kLeftVoltageError) = ::std::pow(10.0, 2.0);
  Q_continuous_(kRightVoltageError, kRightVoltageError) = ::std::pow(10.0, 2.0);
  Q_continuous_(kAngularError, kAngularError) = ::std::pow(2.0, 2.0);

  P_.setZero();
  P_.diagonal() << 0.01, 0.01, 0.01, 0.02, 0.01, 0.02, 0.01, 1, 1, 0.03;

  H_encoders_and_gyro_.setZero();
  // Encoders are stored directly in the state matrix, so are a minor
  // transform away.
  H_encoders_and_gyro_(0, kLeftEncoder) = 1.0;
  H_encoders_and_gyro_(1, kRightEncoder) = 1.0;
  // Gyro rate is just the difference between right/left side speeds:
  H_encoders_and_gyro_(2, kLeftVelocity) = -1.0 / diameter;
  H_encoders_and_gyro_(2, kRightVelocity) = 1.0 / diameter;

  const Eigen::Matrix<Scalar, 4, 4> R_kf_drivetrain =
      dt_config_.make_kf_drivetrain_loop().observer().coefficients().R;
  // TODO(james): The multipliers here are hand-waving things that I put in when
  // tuning things. I haven't yet tried messing with these values again.
  encoder_noise_ = 0.5 * R_kf_drivetrain(0, 0);
  gyro_noise_ = 0.1 * R_kf_drivetrain(2, 2);

  X_hat_.setZero();
  P_.setZero();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
