#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_

#include <chrono>

#include "Eigen/Dense"
#include "aos/commonmath.h"
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
//
// Discussion of the model:
// In the current model, we try to rely primarily on IMU measurements for
// estimating robot state--we also need additional information (some combination
// of output voltages, encoders, and camera data) to help eliminate the biases
// that can accumulate due to integration of IMU data.
// We use IMU measurements as inputs rather than measurement outputs because
// that seemed to be easier to implement. I tried initially running with
// the IMU as a measurement, but it seemed to blow up the complexity of the
// model.
//
// On each prediction update, we take in inputs of the left/right voltages and
// the current measured longitudinal/lateral accelerations. In the current
// setup, the accelerometer readings will be used for estimating how the
// evolution of the longitudinal/lateral velocities. The voltages (and voltage
// errors) will solely be used for estimating the current rotational velocity of
// the robot (I do this because currently I suspect that the accelerometer is a
// much better indicator of current robot state than the voltages). We also
// deliberately decay all of the velocity estimates towards zero to help address
// potential accelerometer biases. We use two separate decay models:
// -The longitudinal velocity is modelled as decaying at a constant rate (see
//  the documentation on the VelocityAccel() method)--this needs a more
//  complex model because the robot will, under normal circumstances, be
//  travelling at non-zero velocities.
// -The lateral velocity is modelled as exponentially decaying towards zero.
//  This is simpler to model and should be reasonably valid, since we will
//  not *normally* be travelling sideways consistently (this assumption may
//  need to be revisited).
// -The "longitudinal velocity offset" (described below) also uses an
//  exponential decay, albeit with a different time constant. A future
//  improvement may remove the decay modelling on the longitudinal velocity
//  itself and instead use that decay model on the longitudinal velocity offset.
//  This would place a bit more trust in the encoder measurements but also
//  more correctly model situations where the robot is legitimately moving at
//  a certain velocity.
//
// For modelling how the drivetrain encoders evolve, and to help prevent the
// aforementioned decay functions from affecting legitimate high-velocity
// maneuvers too much, we have a "longitudinal velocity offset" term. This term
// models the difference between the actual longitudinal velocity of the robot
// (estimated by the average of the left/right velocities) and the velocity
// experienced by the wheels (which can be observed from the encoders more
// directly). Because we model this velocity offset as decaying towards zero,
// what this will do is allow the encoders to be a constant velocity off from
// the accelerometer updates for short periods of time but then gradually
// pull the "actual" longitudinal velocity offset towards that of the encoders,
// helping to reduce constant biases.
template <typename Scalar = double>
class HybridEkf {
 public:
  // An enum specifying what each index in the state vector is for.
  enum StateIdx {
    // Current X/Y position, in meters, of the robot.
    kX = 0,
    kY = 1,
    // Current heading of the robot.
    kTheta = 2,
    // Current estimated encoder reading of the left wheels, in meters.
    // Rezeroed once on startup.
    kLeftEncoder = 3,
    // Current estimated actual velocity of the left side of the robot, in m/s.
    kLeftVelocity = 4,
    // Same variables, for the right side of the robot.
    kRightEncoder = 5,
    kRightVelocity = 6,
    // Estimated offset to input voltage. Used as a generic error term, Volts.
    kLeftVoltageError = 7,
    kRightVoltageError = 8,
    // These error terms are used to estimate the difference between the actual
    // movement of the drivetrain and that implied by the wheel odometry.
    // Angular error effectively estimates a constant angular rate offset of the
    // encoders relative to the actual rotation of the robot.
    // Semi-arbitrary units (we don't bother accounting for robot radius in
    // this).
    kAngularError = 9,
    // Estimate of slip between the drivetrain wheels and the actual
    // forwards/backwards velocity of the robot, in m/s.
    // I.e., (left velocity + right velocity) / 2.0 = (left wheel velocity +
    //        right wheel velocity) / 2.0 + longitudinal velocity offset
    kLongitudinalVelocityOffset = 10,
    // Current estimate of the lateral velocity of the robot, in m/s.
    // Positive implies the robot is moving to its left.
    kLateralVelocity = 11,
  };
  static constexpr int kNStates = 12;
  enum InputIdx {
    // Left/right drivetrain voltages.
    kLeftVoltage = 0,
    kRightVoltage = 1,
    // Current accelerometer readings, in m/s/s, along the longitudinal and
    // lateral axes of the robot. Should be projected onto the X/Y plane, to
    // compensate for tilt of the robot before being passed to this filter. The
    // HybridEkf has no knowledge of the current pitch/roll of the robot, and so
    // can't do anything to compensate for it.
    kLongitudinalAccel = 2,
    kLateralAccel = 3,
  };
  static constexpr int kNInputs = 4;
  // Number of previous samples to save.
  static constexpr int kSaveSamples = 50;
  // Whether we should completely rerun the entire stored history of
  // kSaveSamples on every correction. Enabling this will increase overall CPU
  // usage substantially; however, leaving it disabled makes it so that we are
  // less likely to notice if processing camera frames is causing delays in the
  // drivetrain.
  // If we are having CPU issues, we have three easy avenues to improve things:
  // (1) Reduce kSaveSamples (e.g., if all camera frames arive within
  //     100 ms, then we can reduce kSaveSamples to be 25 (125 ms of samples)).
  // (2) Don't actually rely on the ability to insert corrections into the
  //     timeline.
  // (3) Set this to false.
  static constexpr bool kFullRewindOnEverySample = false;
  // Assume that all correction steps will have kNOutputs
  // dimensions.
  // TODO(james): Relax this assumption; relaxing it requires
  // figuring out how to deal with storing variable size
  // observation matrices, though.
  static constexpr int kNOutputs = 3;
  // Time constant to use for estimating how the longitudinal/lateral velocity
  // offsets decay, in seconds.
  static constexpr double kVelocityOffsetTimeConstant = 1.0;
  static constexpr double kLateralVelocityTimeConstant = 1.0;
  // Inputs are [left_volts, right_volts]
  typedef Eigen::Matrix<Scalar, kNInputs, 1> Input;
  // Outputs are either:
  // [left_encoder, right_encoder, gyro_vel]; or [heading, distance, skew] to
  // some target. This makes it so we don't have to figure out how we store
  // variable-size measurement updates.
  typedef Eigen::Matrix<Scalar, kNOutputs, 1> Output;
  typedef Eigen::Matrix<Scalar, kNStates, kNStates> StateSquare;
  // State contains the states defined by the StateIdx enum. See comments there.
  typedef Eigen::Matrix<Scalar, kNStates, 1> State;

  // Constructs a HybridEkf for a particular drivetrain.
  // Currently, we use the drivetrain config for modelling constants
  // (continuous time A and B matrices) and for the noise matrices for the
  // encoders/gyro.
  HybridEkf(const DrivetrainConfig<double> &dt_config)
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
    observations_.PushFromBottom({
        t,
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
        Eigen::Matrix<Scalar, kNOutputs, kNOutputs>::Identity(),
        StateSquare::Identity(),
        StateSquare::Zero(),
        std::chrono::seconds(0),
        State::Zero(),
    });
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
      std::function<
          void(const State &, const StateSquare &,
               std::function<Output(const State &, const Input &)> *,
               std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(
                   const State &)> *)> make_h,
      std::function<Output(const State &, const Input &)> h,
      std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
          dhdx, const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
      aos::monotonic_clock::time_point t);

  // A utility function for specifically updating with encoder and gyro
  // measurements.
  void UpdateEncodersAndGyro(const Scalar left_encoder,
                             const Scalar right_encoder, const Scalar gyro_rate,
                             const Eigen::Matrix<Scalar, 2, 1> &voltage,
                             const Eigen::Matrix<Scalar, 3, 1> &accel,
                             aos::monotonic_clock::time_point t) {
    Input U;
    U.template block<2, 1>(0, 0) = voltage;
    U.template block<2, 1>(kLongitudinalAccel, 0) =
        accel.template block<2, 1>(0, 0);
    RawUpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, t);
  }
  // Version of UpdateEncodersAndGyro that takes a input matrix rather than
  // taking in a voltage/acceleration separately.
  void RawUpdateEncodersAndGyro(const Scalar left_encoder,
                                const Scalar right_encoder,
                                const Scalar gyro_rate, const Input &U,
                                aos::monotonic_clock::time_point t) {
    // Because the check below for have_zeroed_encoders_ will add an
    // Observation, do a check here to ensure that initialization has been
    // performed and so there is at least one observation.
    CHECK(!observations_.empty());
    if (!have_zeroed_encoders_) {
      // This logic handles ensuring that on the first encoder reading, we
      // update the internal state for the encoders to match the reading.
      // Otherwise, if we restart the drivetrain without restarting
      // wpilib_interface, then we can get some obnoxious initial corrections
      // that mess up the localization.
      State newstate = X_hat_;
      newstate(kLeftEncoder) = left_encoder;
      newstate(kRightEncoder) = right_encoder;
      newstate(kLeftVoltageError) = 0.0;
      newstate(kRightVoltageError) = 0.0;
      newstate(kAngularError) = 0.0;
      newstate(kLongitudinalVelocityOffset) = 0.0;
      newstate(kLateralVelocity) = 0.0;
      ResetInitialState(t, newstate, P_);
      // We need to set have_zeroed_encoders_ after ResetInitialPosition because
      // the reset clears have_zeroed_encoders_...
      have_zeroed_encoders_ = true;
    }

    Output z(left_encoder, right_encoder, gyro_rate);

    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;
    R.setZero();
    R.diagonal() << encoder_noise_, encoder_noise_, gyro_noise_;
    Correct(
        z, &U, {},
        [this](const State &X, const Input &) {
          return H_encoders_and_gyro_ * X;
        },
        [this](const State &) { return H_encoders_and_gyro_; }, R, t);
  }

  // Sundry accessor:
  State X_hat() const { return X_hat_; }
  Scalar X_hat(long i) const { return X_hat_(i); }
  StateSquare P() const { return P_; }
  aos::monotonic_clock::time_point latest_t() const {
    return observations_.top().t;
  }

  static Scalar CalcLongitudinalVelocity(const State &X) {
    return (X(kLeftVelocity) + X(kRightVelocity)) / 2.0;
  }

  Scalar CalcYawRate(const State &X) const {
    return (X(kRightVelocity) - X(kLeftVelocity)) / 2.0 /
           dt_config_.robot_radius;
  }

  // Returns the last state before the specified time.
  // Returns nullopt if time is older than the oldest measurement.
  std::optional<State> LastStateBeforeTime(
      aos::monotonic_clock::time_point time) {
    if (observations_.empty() || observations_.begin()->t > time) {
      return std::nullopt;
    }
    for (const auto &observation : observations_) {
      if (observation.t > time) {
        // Note that observation.X_hat actually references the _previous_ X_hat.
        return observation.X_hat;
      }
    }
    return X_hat();
  }

  // Returns the most recent input vector.
  Input MostRecentInput() {
    CHECK(!observations_.empty());
    Input U = observations_.top().U;
    return U;
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
    std::function<void(const State &, const StateSquare &,
                       std::function<Output(const State &, const Input &)> *,
                       std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(
                           const State &)> *)> make_h;
    // A function to calculate the expected output at a given state/input.
    // TODO(james): For encoders/gyro, it is linear and the function call may
    // be expensive. Potential source of optimization.
    std::function<Output(const State &, const Input &)> h;
    // The Jacobian of h with respect to x.
    // We assume that U has no impact on the Jacobian.
    // TODO(james): Currently, none of the users of this actually make use of
    // the ability to have dynamic dhdx (technically, the camera code should
    // recalculate it to be strictly correct, but I was both too lazy to do
    // so and it seemed unnecessary). This is a potential source for future
    // optimizations if function calls are being expensive.
    std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
        dhdx;
    // The measurement noise matrix.
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;

    // Discretized A and Q to use on this update step. These will only be
    // recalculated if the timestep changes.
    StateSquare A_d;
    StateSquare Q_d;
    aos::monotonic_clock::duration discretization_time;

    // A cached value indicating how much we change X_hat in the prediction step
    // of this Observation.
    State predict_update;

    // In order to sort the observations in the PriorityQueue object, we
    // need a comparison function.
    friend bool operator<(const Observation &l, const Observation &r) {
      return l.t < r.t;
    }
  };

  void InitializeMatrices();

  // These constants and functions define how the longitudinal velocity
  // (the average of the left and right velocities) decays. We model it as
  // decaying at a constant rate, except very near zero where the decay rate is
  // exponential (this is more numerically stable than just using a constant
  // rate the whole time). We use this model rather than a simpler exponential
  // decay because an exponential decay will result in the robot's velocity
  // estimate consistently being far too low when at high velocities, and since
  // the acceleromater-based estimate of the velocity will only drift at a
  // relatively slow rate and doesn't get worse at higher velocities, we can
  // safely decay pretty slowly.
  static constexpr double kMaxVelocityAccel = 0.005;
  static constexpr double kMaxVelocityGain = 1.0;
  static Scalar VelocityAccel(Scalar velocity) {
    return -std::clamp(kMaxVelocityGain * velocity, -kMaxVelocityAccel,
                       kMaxVelocityAccel);
  }

  static Scalar VelocityAccelDiff(Scalar velocity) {
    return (std::abs(kMaxVelocityGain * velocity) > kMaxVelocityAccel)
               ? 0.0
               : -kMaxVelocityGain;
  }

  // Returns the "A" matrix for a given state. See DiffEq for discussion of
  // ignore_accel.
  StateSquare AForState(const State &X, bool ignore_accel = false) const {
    // Calculate the A matrix for a given state. Note that A = partial Xdot /
    // partial X. This is distinct from saying that Xdot = A * X. This is
    // particularly relevant for the (kX, kTheta) members which otherwise seem
    // odd.
    StateSquare A_continuous = A_continuous_;
    const Scalar theta = X(kTheta);
    const Scalar stheta = std::sin(theta);
    const Scalar ctheta = std::cos(theta);
    const Scalar lng_vel = CalcLongitudinalVelocity(X);
    const Scalar lat_vel = X(kLateralVelocity);
    const Scalar diameter = 2.0 * dt_config_.robot_radius;
    const Scalar yaw_rate = CalcYawRate(X);
    // X and Y derivatives
    A_continuous(kX, kTheta) =
        -stheta * lng_vel - ctheta * lat_vel;
    A_continuous(kX, kLeftVelocity) = ctheta / 2.0;
    A_continuous(kX, kRightVelocity) = ctheta / 2.0;
    A_continuous(kX, kLateralVelocity) = -stheta;
    A_continuous(kY, kTheta) = ctheta * lng_vel - stheta * lat_vel;
    A_continuous(kY, kLeftVelocity) = stheta / 2.0;
    A_continuous(kY, kRightVelocity) = stheta / 2.0;
    A_continuous(kY, kLateralVelocity) = ctheta;

    if (!ignore_accel) {
      const Eigen::Matrix<Scalar, 1, kNStates> lng_vel_row =
          (A_continuous.row(kLeftVelocity) + A_continuous.row(kRightVelocity)) /
          2.0;
      A_continuous.row(kLeftVelocity) -= lng_vel_row;
      A_continuous.row(kRightVelocity) -= lng_vel_row;
      // Terms to account for centripetal accelerations.
      // lateral centripetal accel = -yaw_rate * lng_vel
      A_continuous(kLateralVelocity, kLeftVelocity) +=
          X(kLeftVelocity) / diameter;
      A_continuous(kLateralVelocity, kRightVelocity) +=
          -X(kRightVelocity) / diameter;
      A_continuous(kRightVelocity, kLateralVelocity) += yaw_rate;
      A_continuous(kLeftVelocity, kLateralVelocity) += yaw_rate;
      const Scalar dlng_accel_dwheel_vel = X(kLateralVelocity) / diameter;
      A_continuous(kRightVelocity, kRightVelocity) += dlng_accel_dwheel_vel;
      A_continuous(kLeftVelocity, kRightVelocity) += dlng_accel_dwheel_vel;
      A_continuous(kRightVelocity, kLeftVelocity) += -dlng_accel_dwheel_vel;
      A_continuous(kLeftVelocity, kLeftVelocity) += -dlng_accel_dwheel_vel;

      A_continuous(kRightVelocity, kRightVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kRightVelocity, kLeftVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kLeftVelocity, kRightVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kLeftVelocity, kLeftVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
    }
    return A_continuous;
  }

  // Returns dX / dt given X and U. If ignore_accel is set, then we ignore the
  // accelerometer-based components of U (this is solely used in testing).
  State DiffEq(const State &X, const Input &U,
               bool ignore_accel = false) const {
    State Xdot = A_continuous_ * X + B_continuous_ * U;
    // And then we need to add on the terms for the x/y change:
    const Scalar theta = X(kTheta);
    const Scalar lng_vel = CalcLongitudinalVelocity(X);
    const Scalar lat_vel = X(kLateralVelocity);
    const Scalar stheta = std::sin(theta);
    const Scalar ctheta = std::cos(theta);
    Xdot(kX) = ctheta * lng_vel - stheta * lat_vel;
    Xdot(kY) = stheta * lng_vel + ctheta * lat_vel;

    const Scalar yaw_rate = CalcYawRate(X);
    const Scalar expected_lat_accel = lng_vel * yaw_rate;
    const Scalar expected_lng_accel =
        CalcLongitudinalVelocity(Xdot) - yaw_rate * lat_vel;
    const Scalar lng_accel_offset =
        U(kLongitudinalAccel) - expected_lng_accel;
    constexpr double kAccelWeight = 1.0;
    if (!ignore_accel) {
      Xdot(kLeftVelocity) += kAccelWeight * lng_accel_offset;
      Xdot(kRightVelocity) += kAccelWeight * lng_accel_offset;
      Xdot(kLateralVelocity) += U(kLateralAccel) - expected_lat_accel;

      Xdot(kRightVelocity) += VelocityAccel(lng_vel);
      Xdot(kLeftVelocity) += VelocityAccel(lng_vel);
    }
    return Xdot;
  }

  void PredictImpl(Observation *obs, std::chrono::nanoseconds dt, State *state,
                   StateSquare *P) {
    // Only recalculate the discretization if the timestep has changed.
    // Technically, this isn't quite correct, since the discretization will
    // change depending on the current state. However, the slight loss of
    // precision seems acceptable for the sake of significantly reducing CPU
    // usage.
    if (obs->discretization_time != dt) {
      // TODO(james): By far the biggest CPU sink in the localization appears to
      // be this discretization--it's possible the spline code spikes higher,
      // but it doesn't create anywhere near the same sustained load. There
      // are a few potential options for optimizing this code, but none of
      // them are entirely trivial, e.g. we could:
      // -Reduce the number of states (this function grows at O(kNStates^3))
      // -Adjust the discretization function itself (there're a few things we
      //  can tune there).
      // -Try to come up with some sort of lookup table or other way of
      //  pre-calculating A_d and Q_d.
      // I also have to figure out how much we care about the precision of
      // some of these values--I don't think we care much, but we probably
      // do want to maintain some of the structure of the matrices.
      const StateSquare A_c = AForState(*state);
      controls::DiscretizeQAFast(Q_continuous_, A_c, dt, &obs->Q_d, &obs->A_d);
      obs->discretization_time = dt;

      obs->predict_update =
          RungeKuttaU(
              [this](const State &X, const Input &U) { return DiffEq(X, U); },
              *state, obs->U, aos::time::DurationInSeconds(dt)) -
          *state;
    }

    *state += obs->predict_update;

    StateSquare Ptemp = obs->A_d * *P * obs->A_d.transpose() + obs->Q_d;
    *P = Ptemp;
  }

  void CorrectImpl(Observation *obs, State *state, StateSquare *P) {
    const Eigen::Matrix<Scalar, kNOutputs, kNStates> H = obs->dhdx(*state);
    // Note: Technically, this does calculate P * H.transpose() twice. However,
    // when I was mucking around with some things, I found that in practice
    // putting everything into one expression and letting Eigen optimize it
    // directly actually improved performance relative to precalculating P *
    // H.transpose().
    const Eigen::Matrix<Scalar, kNStates, kNOutputs> K =
        *P * H.transpose() * (H * *P * H.transpose() + obs->R).inverse();
    const StateSquare Ptemp = (StateSquare::Identity() - K * H) * *P;
    *P = Ptemp;
    *state += K * (obs->z - obs->h(*state, obs->U));
  }

  void ProcessObservation(Observation *obs, const std::chrono::nanoseconds dt,
                          State *state, StateSquare *P) {
    *state = obs->X_hat;
    *P = obs->P;
    if (dt.count() != 0) {
      PredictImpl(obs, dt, state, P);
    }
    if (!(obs->h && obs->dhdx)) {
      CHECK(obs->make_h);
      obs->make_h(*state, *P, &obs->h, &obs->dhdx);
    }
    CorrectImpl(obs, state, P);
  }

  DrivetrainConfig<double> dt_config_;
  State X_hat_;
  StateFeedbackHybridPlantCoefficients<2, 2, 2, double>
      velocity_drivetrain_coefficients_;
  StateSquare A_continuous_;
  StateSquare Q_continuous_;
  StateSquare P_;
  Eigen::Matrix<Scalar, kNOutputs, kNStates> H_encoders_and_gyro_;
  Scalar encoder_noise_, gyro_noise_;
  Eigen::Matrix<Scalar, kNStates, kNInputs> B_continuous_;

  bool have_zeroed_encoders_ = false;

  aos::PriorityQueue<Observation, kSaveSamples, std::less<Observation>>
      observations_;


  friend class testing::HybridEkfTest;
  friend class y2019::control_loops::testing::ParameterizedLocalizerTest;
};  // class HybridEkf

template <typename Scalar>
void HybridEkf<Scalar>::Correct(
    const Output &z, const Input *U,
    std::function<void(const State &, const StateSquare &,
                       std::function<Output(const State &, const Input &)> *,
                       std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(
                           const State &)> *)> make_h,
    std::function<Output(const State &, const Input &)> h,
    std::function<Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)>
        dhdx, const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
    aos::monotonic_clock::time_point t) {
  CHECK(!observations_.empty());
  if (!observations_.full() && t < observations_.begin()->t) {
    LOG(ERROR) << "Dropped an observation that was received before we "
                  "initialized.\n";
    return;
  }
  auto cur_it = observations_.PushFromBottom(
      {t, t, State::Zero(), StateSquare::Zero(), Input::Zero(), z, make_h, h,
       dhdx, R, StateSquare::Identity(), StateSquare::Zero(),
       std::chrono::seconds(0), State::Zero()});
  if (cur_it == observations_.end()) {
    VLOG(1) << "Camera dropped off of end with time of "
            << aos::time::DurationInSeconds(t.time_since_epoch())
            << "s; earliest observation in "
               "queue has time of "
            << aos::time::DurationInSeconds(
                   observations_.begin()->t.time_since_epoch())
            << "s.\n";
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
    CHECK(U != nullptr);
    cur_it->U = *U;
  } else {
    cur_it->X_hat = next_it->X_hat;
    cur_it->P = next_it->P;
    cur_it->prev_t = next_it->prev_t;
    next_it->prev_t = cur_it->t;
    cur_it->U = (U == nullptr) ? next_it->U : *U;
  }

  if (kFullRewindOnEverySample) {
    next_it = observations_.begin();
    cur_it = next_it++;
  }

  // Now we need to rerun the predict step from the previous to the new
  // observation as well as every following correct/predict up to the current
  // time.
  while (true) {
    // We use X_hat_ and P_ to store the intermediate states, and then
    // once we reach the end they will all be up-to-date.
    ProcessObservation(&*cur_it, cur_it->t - cur_it->prev_t, &X_hat_, &P_);
    CHECK(X_hat_.allFinite());
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
  A_continuous_(kLeftEncoder, kLongitudinalVelocityOffset) = -1.0;
  A_continuous_(kRightEncoder, kRightVelocity) = 1.0;
  A_continuous_(kRightEncoder, kAngularError) = -1.0;
  A_continuous_(kRightEncoder, kLongitudinalVelocityOffset) = -1.0;

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

  A_continuous_(kLongitudinalVelocityOffset, kLongitudinalVelocityOffset) =
      -1.0 / kVelocityOffsetTimeConstant;
  A_continuous_(kLateralVelocity, kLateralVelocity) =
      -1.0 / kLateralVelocityTimeConstant;

  // We currently ignore all voltage-related modelling terms.
  // TODO(james): Decide what to do about these terms. They don't really matter
  // too much when we have accelerometer readings available.
  B_continuous_.setZero();
  B_continuous_.template block<1, 2>(kLeftVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(0).template cast<Scalar>();
  B_continuous_.template block<1, 2>(kRightVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(1).template cast<Scalar>();
  A_continuous_.template block<kNStates, 2>(0, kLeftVoltageError) =
      B_continuous_.template block<kNStates, 2>(0, kLeftVoltage);

  Q_continuous_.setZero();
  // TODO(james): Improve estimates of process noise--e.g., X/Y noise can
  // probably be reduced when we are stopped because you rarely jump randomly.
  // Or maybe it's more appropriate to scale wheelspeed noise with wheelspeed,
  // since the wheels aren't likely to slip much stopped.
  Q_continuous_(kX, kX) = 0.002;
  Q_continuous_(kY, kY) = 0.002;
  Q_continuous_(kTheta, kTheta) = 0.0001;
  Q_continuous_(kLeftEncoder, kLeftEncoder) = std::pow(0.15, 2.0);
  Q_continuous_(kRightEncoder, kRightEncoder) = std::pow(0.15, 2.0);
  Q_continuous_(kLeftVelocity, kLeftVelocity) = std::pow(0.5, 2.0);
  Q_continuous_(kRightVelocity, kRightVelocity) = std::pow(0.5, 2.0);
  Q_continuous_(kLeftVoltageError, kLeftVoltageError) = std::pow(10.0, 2.0);
  Q_continuous_(kRightVoltageError, kRightVoltageError) = std::pow(10.0, 2.0);
  Q_continuous_(kAngularError, kAngularError) = std::pow(2.0, 2.0);
  // This noise value largely governs whether we will trust the encoders or
  // accelerometer more for estimating the robot position.
  // Note that this also affects how we interpret camera measurements,
  // particularly when using a heading/distance/skew measurement--if the
  // noise on these numbers is particularly high, then we can end up with weird
  // dynamics where a camera update both shifts our X/Y position and adjusts our
  // velocity estimates substantially, causing the camera updates to create
  // "momentum" and if we don't trust the encoders enough, then we have no way of
  // determining that the velocity updates are bogus. This also interacts with
  // kVelocityOffsetTimeConstant.
  Q_continuous_(kLongitudinalVelocityOffset, kLongitudinalVelocityOffset) =
      std::pow(1.1, 2.0);
  Q_continuous_(kLateralVelocity, kLateralVelocity) = std::pow(0.1, 2.0);

  H_encoders_and_gyro_.setZero();
  // Encoders are stored directly in the state matrix, so are a minor
  // transform away.
  H_encoders_and_gyro_(0, kLeftEncoder) = 1.0;
  H_encoders_and_gyro_(1, kRightEncoder) = 1.0;
  // Gyro rate is just the difference between right/left side speeds:
  H_encoders_and_gyro_(2, kLeftVelocity) = -1.0 / diameter;
  H_encoders_and_gyro_(2, kRightVelocity) = 1.0 / diameter;

  encoder_noise_ = 5e-9;
  gyro_noise_ = 1e-13;

  X_hat_.setZero();
  P_.setZero();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
