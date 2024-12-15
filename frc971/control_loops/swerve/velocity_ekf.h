#ifndef FRC971_CONTROL_LOOPS_SWERVE_VELOCITY_EKF_H_
#define FRC971_CONTROL_LOOPS_SWERVE_VELOCITY_EKF_H_
#include "absl/flags/declare.h"
#include "absl/flags/flag.h"

#include "frc971/control_loops/swerve/linearization_utils.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_static.h"
#include "frc971/estimation/ekf.h"
#include "frc971/math/flatbuffers_matrix.h"

ABSL_DECLARE_FLAG(double, velocity_ekf_q_thetas);
ABSL_DECLARE_FLAG(double, velocity_ekf_q_omegas);
ABSL_DECLARE_FLAG(double, velocity_ekf_q_theta);
ABSL_DECLARE_FLAG(double, velocity_ekf_q_omega);
ABSL_DECLARE_FLAG(double, velocity_ekf_q_velocity);

namespace frc971::control_loops::swerve {
// Implements an Extended Kalman Filter for estimating the velocity states of a
// swerve drivebase using the SimplifiedDynamics class.
template <typename Scalar>
class VelocityEkf {
 public:
  using Dynamics = SimplifiedDynamics<Scalar>;
  static constexpr int kNumStates = Dynamics::kNumVelocityStates;
  using States = Dynamics::States;
  using EkfType = Ekf<Scalar, kNumStates, kNumInputs>;
  using VirtualDynamics = Dynamics::VirtualVelocityDynamics;
  using DynamicsParameters = Dynamics::Parameters;
  using StateSquare = EkfType::StateSquare;
  using Input = EkfType::Input;
  using State = EkfType::State;

  // Functor which calculates the expected velocity of each drive wheel given
  // the current state.
  // This just assumes some straightforwards kinematics, without any particular
  // accounting for slip.
  class ExpectedDriveVelocity {
   public:
    ExpectedDriveVelocity(const DynamicsParameters &params) : params_(params) {}

    // Implementation of the functor which returns a matrix of the expected
    // drive velocities of each motor given the current state.
    // The drive velocities will be in the same order that the modules are in
    // the state vector.
    template <typename LocalScalar>
    Eigen::Matrix<LocalScalar, 4, 1> operator()(
        const Eigen::Map<const Eigen::Matrix<LocalScalar, kNumStates, 1>> X)
        const {
      const Eigen::Matrix<LocalScalar, 2, 1> robot_vel{{X(States::kVx)},
                                                       {X(States::kVy)}};
      const LocalScalar theta = X(States::kTheta);
      const LocalScalar omega = X(States::kOmega);
      // Ceres only plays nice with global-namespace sin/cos.
      const LocalScalar sneg_theta = sin(-theta);
      const LocalScalar cneg_theta = cos(-theta);
      const Eigen::Matrix<LocalScalar, 2, 2> Rneg_theta{
          {cneg_theta, -sneg_theta}, {sneg_theta, cneg_theta}};
      const Eigen::Matrix<LocalScalar, 2, 1> robot_vel_in_robot_frame =
          Rneg_theta * robot_vel;
      // module_velocity_in_robot_frame = R(-theta) * robot_vel +
      //     omega.cross(module_position);
      // module_vel_x = (cos(-theta) * vx - sin(-theta) * vy) - omega
      //     * module_y
      // module_vel_y = (sin(-theta) * vx + cos(-theta) * vy) + omega
      //     * module_x
      // And to project into the drive direction, do the dot product of the
      // unit vector in the module_theta direction.
      Eigen::Matrix<LocalScalar, 4, 1> result;
      for (size_t module_index = 0; module_index < params_.modules.size();
           ++module_index) {
        const size_t thetas_index = States::kThetas0 + 2 * module_index;
        const LocalScalar module_theta = X(thetas_index);
        const Eigen::Matrix<LocalScalar, 2, 1> module_position =
            params_.modules[module_index].position.template cast<LocalScalar>();
        const LocalScalar module_vel_x =
            robot_vel_in_robot_frame.x() - omega * module_position.y();
        const LocalScalar module_vel_y =
            robot_vel_in_robot_frame.y() + omega * module_position.x();
        result(module_index) =
            module_vel_x * cos(module_theta) + module_vel_y * sin(module_theta);
      }
      return result;
    }

    // Overload which takes in a regular Eigen::Matrix rather than a Eigen::Map.
    template <typename LocalScalar>
    Eigen::Matrix<LocalScalar, 4, 1> operator()(
        const Eigen::Matrix<LocalScalar, kNumStates, 1> &X) const {
      return (*this)(
          Eigen::Map<const Eigen::Matrix<LocalScalar, kNumStates, 1>>(
              X.data()));
    }

   private:
    DynamicsParameters params_;
  };

  VelocityEkf(const DynamicsParameters &params,
              const StateSquare &Q = MakeReasonableQ())
      : params_(params),
        ekf_({.Q_continuous = Q,
              .dynamics = std::make_unique<VirtualDynamics>(params)}),
        H_drive_encoder_(params_) {
    // TODO(james): Allow parameterization of the noises for the various
    // measurements.
    H_gyro_.setZero();
    H_gyro_(0, States::kOmega) = 1;

    steer_encoder_noise_.setIdentity();
    steer_encoder_noise_ *= 1e-4;
    H_steer_encoder_.setZero();
    H_steer_encoder_(0, States::kThetas0) = 1.0;
    H_steer_encoder_(1, States::kThetas1) = 1.0;
    H_steer_encoder_(2, States::kThetas2) = 1.0;
    H_steer_encoder_(3, States::kThetas3) = 1.0;

    drive_encoder_noise_.setIdentity();
    drive_encoder_noise_ *= 1e-2;
  }

  // Initializes the state noises to some values which are known to be
  // reasonable. This is mostly meant for use in tests.
  static StateSquare MakeReasonableQ() {
    StateSquare Q = StateSquare::Identity();
    Q.diagonal()(States::kThetas0) = absl::GetFlag(FLAGS_velocity_ekf_q_thetas);
    Q.diagonal()(States::kThetas1) = absl::GetFlag(FLAGS_velocity_ekf_q_thetas);
    Q.diagonal()(States::kThetas2) = absl::GetFlag(FLAGS_velocity_ekf_q_thetas);
    Q.diagonal()(States::kThetas3) = absl::GetFlag(FLAGS_velocity_ekf_q_thetas);
    Q.diagonal()(States::kOmegas0) = absl::GetFlag(FLAGS_velocity_ekf_q_omegas);
    Q.diagonal()(States::kOmegas1) = absl::GetFlag(FLAGS_velocity_ekf_q_omegas);
    Q.diagonal()(States::kOmegas2) = absl::GetFlag(FLAGS_velocity_ekf_q_omegas);
    Q.diagonal()(States::kOmegas3) = absl::GetFlag(FLAGS_velocity_ekf_q_omegas);
    Q.diagonal()(States::kTheta) = absl::GetFlag(FLAGS_velocity_ekf_q_theta);
    Q.diagonal()(States::kOmega) = absl::GetFlag(FLAGS_velocity_ekf_q_omega);
    Q.diagonal()(States::kVx) = absl::GetFlag(FLAGS_velocity_ekf_q_velocity);
    Q.diagonal()(States::kVy) = absl::GetFlag(FLAGS_velocity_ekf_q_velocity);
    return Q;
  }

  void Initialize(const aos::monotonic_clock::time_point now,
                  const State &initial_state) {
    // TODO(james): Do a more principled initialization of the estimate
    // covariance.
    ekf_.Initialize(now, initial_state, MakeReasonableQ());
  }

  // Converts the CorrectionDebug object from the Ekf class into a flatbuffer
  // for debugging.
  template <int NumMeasurements>
  void MakeCorrectionDebug(
      const EkfType::template CorrectionDebug<NumMeasurements> &debug,
      CorrectionDebugStatic *debug_fbs) {
    CHECK(FromEigen(debug.measurement.template cast<double>(),
                    debug_fbs->add_measurement()));
    CHECK(FromEigen(debug.expected.template cast<double>(),
                    debug_fbs->add_expected()));
    CHECK(FromEigen(debug.correct_update.template cast<double>(),
                    debug_fbs->add_correct_update()));
    CHECK(FromEigen(debug.predict_update.template cast<double>(),
                    debug_fbs->add_predict_update()));
  }

  // Updates the EKF for the current time with measurements from various
  // sensors.
  // Parameters:
  // now: Current system time.
  // steer_encoders: Current readings from the mag encoders on the steering
  //     module, in radians.
  //     TODO(james): Since these readings are currently forwarded from one
  //     device to another we should compensate for the latency.
  // drive_encoder_sample_time: A timestamp for when the drive encoder readings
  //     were samples. This will generally be from the clock on the Kraken
  //     itself.
  // drive_encoders: Translation motor encoder readings to use, in meters.
  // yaw_rate: Current gyro yaw rate, in radians / sec.
  // U: Current currents being applied to the motors.
  // status_fbs: Status flatbuffer to populate with debug information.
  void Update(const aos::monotonic_clock::time_point now,
              const Eigen::Matrix<Scalar, 4, 1> &steer_encoders,
              const std::array<aos::monotonic_clock::time_point, 4>
                  &drive_encoder_sample_time,
              const Eigen::Matrix<Scalar, 4, 1> &drive_encoders,
              Scalar yaw_rate, const Input &U,
              VelocityEkfStatusStatic *status_fbs = nullptr) {
    // Yaw rate update:
    {
      const typename EkfType::template CorrectionDebug<1> debug = ekf_.Correct(
          now, Eigen::Matrix<Scalar, 1, 1>{{yaw_rate}}, gyro_noise_, U,
          [this](const Eigen::Map<const State> X) { return H_gyro_ * X; },
          H_gyro_);
      if (status_fbs != nullptr) {
        MakeCorrectionDebug(debug, status_fbs->add_gyro_correction());
      }
    }
    // Steer encoder updates:
    {
      if (!initialized_steer_motors_) {
        // TODO(james): Do something with this; I think not handling the
        // first-iteration case was causing issues.
        initialized_steer_motors_ = true;
      }
      const typename EkfType::template CorrectionDebug<4> debug = ekf_.Correct(
          now, steer_encoders, steer_encoder_noise_, U,
          [this](const Eigen::Map<const State> X) {
            return H_steer_encoder_ * X;
          },
          H_steer_encoder_);
      if (status_fbs != nullptr) {
        MakeCorrectionDebug(debug, status_fbs->add_steer_correction());
      }
    }
    // For the drive encoders, we manually differentiate the input encoder
    // values and then use those for correction.
    // This saves us having to track encoder states within the kalman filter
    // itself, and should not lose any information (considering that all the
    // encoder is doing is counting ticks in-between each time period).
    {
      if (last_drive_encoder_update_.has_value()) {
        Eigen::Matrix<Scalar, 4, 1> drive_velocities =
            H_drive_encoder_(Eigen::Map<const State>{ekf_.X_hat().data()});

        for (int module_index = 0; module_index < 4; ++module_index) {
          // TODO(james): Handle Kraken reboots.
          const double drive_dt = aos::time::DurationInSeconds(
              drive_encoder_sample_time[module_index] -
              last_drive_encoder_update_.value()[module_index]);
          if (drive_dt != 0.0) {
            drive_velocities(module_index) =
                (drive_encoders(module_index) -
                 last_drive_encoders_(module_index)) /
                drive_dt;
          }
        }
        const typename EkfType::template CorrectionDebug<4> debug =
            ekf_.template Correct<4>(now, drive_velocities,
                                     drive_encoder_noise_, U, H_drive_encoder_);
        if (status_fbs != nullptr) {
          MakeCorrectionDebug(debug, status_fbs->add_drive_correction());
        }
      }
      last_drive_encoder_update_ = drive_encoder_sample_time;
      last_drive_encoders_ = drive_encoders;
    }
    if (status_fbs != nullptr) {
      CHECK(FromEigen(ekf_.X_hat().template cast<double>(),
                      status_fbs->add_x_hat()));
      CHECK(FromEigen(ekf_.P().template cast<double>(), status_fbs->add_p()));
    }
  }

  const State &X_hat() const { return ekf_.X_hat(); }
  const StateSquare &P() const { return ekf_.P(); }

 private:
  DynamicsParameters params_;
  EkfType ekf_;
  bool initialized_steer_motors_ = false;
  Eigen::Matrix<Scalar, 1, 1> gyro_noise_{{1e-5}};
  Eigen::Matrix<Scalar, 1, kNumStates> H_gyro_;
  Eigen::Matrix<Scalar, 4, 4> steer_encoder_noise_;
  Eigen::Matrix<Scalar, 4, kNumStates> H_steer_encoder_;
  Eigen::Matrix<Scalar, 4, 4> drive_encoder_noise_;
  Eigen::Matrix<Scalar, 4, 1> last_drive_encoders_;
  std::optional<std::array<aos::monotonic_clock::time_point, 4>>
      last_drive_encoder_update_;
  ExpectedDriveVelocity H_drive_encoder_;
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_VELOCITY_EKF_H_
