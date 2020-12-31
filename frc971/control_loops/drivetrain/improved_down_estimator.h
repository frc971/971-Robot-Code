#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/runge_kutta.h"
#include "glog/logging.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// Generates the sigma points to use in the UKF given the current estimate and
// covariance.
Eigen::Matrix<double, 4, 3 * 2 + 1> GenerateSigmaPoints(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 3, 3> &covariance);

// Computes the covariance of the noise given the mean and the transformed sigma
// points. The residual corresponds with the W' variable from the original
// paper.
Eigen::Matrix<double, 3, 3> ComputeQuaternionCovariance(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 4, 7> &points,
    Eigen::Matrix<double, 3, 7> *residual);

// This class provides a quaternion-based Kalman filter for estimating
// orientation using a 3-axis gyro and 3-axis accelerometer. It does leave open
// the option of overridding the system process model and the function used to
// calculate the expected measurement (which is relevant if, e.g., the IMU is
// not mounted horizontally in the robot).
class QuaternionUkf {
 public:
  // The state is just a quaternion representing the current robot orientaiton.
  // The zero/identity quaternion (1, 0, 0, 0) implies that the robot is
  // position flat on the ground with a heading of zero (which, in our normal
  // field coordinates, means pointed straight away from our driver's station
  // wall).
  // The X axis is pointed straight out from the driver's station, the Z axis
  // straight up and the Y axis straight to the left (i.e., a right-handed
  // coordinate system).
  // The quaternion itself represents the transformation from the body frame to
  // global frame. E.g., for the gravity vector, the acceleration due to gravity
  // in the global frame is equal to X_hat_ * gravity_in_robot_frame. Note that
  // this convention does seem to be the inverse of that used in the paper
  // referenced on Quaternion UKFs.
  constexpr static int kNumStates = 4;
  // Inputs to the system--we use the (x, y, z)  gyro measurements as the inputs
  // to the system.
  constexpr static int kNumInputs = 3;
  // Measurements to use for correcting the estimated system state. These
  // correspond to (x, y, z) measurements from the accelerometer.
  constexpr static int kNumMeasurements = 3;
  QuaternionUkf(const Eigen::Matrix<double, 3, 3> &imu_transform =
                    Eigen::Matrix<double, 3, 3>::Identity())
      : imu_transform_(imu_transform) {
    Reset();
  }

  void Reset();

  // Handles updating the state of the UKF, given the gyro and accelerometer
  // measurements. Given the design of the filter, U is the x/y/z gyro
  // measurements and measurement is the accelerometer x/y/z measurements.
  // dt is the length of the current timestep.
  // U specifically corresponds with the U in the paper, which corresponds with
  // the input to the system used by the filter.
  // Accelerometer measurements should be in g's, and gyro measurements in
  // radians / sec.
  void Predict(const Eigen::Matrix<double, kNumInputs, 1> &U,
               const Eigen::Matrix<double, kNumMeasurements, 1> &measurement,
               const aos::monotonic_clock::duration dt);

  // Returns the updated state for X after one time step, given the current
  // state and gyro measurements.
  virtual Eigen::Matrix<double, kNumStates, 1> A(
      const Eigen::Matrix<double, kNumStates, 1> &X,
      const Eigen::Matrix<double, kNumInputs, 1> &U,
      const aos::monotonic_clock::duration dt) const = 0;

  // Returns the current expected accelerometer measurements given the current
  // state.
  virtual Eigen::Matrix<double, kNumMeasurements, 1> H(
      const Eigen::Matrix<double, kNumStates, 1> &X) const = 0;

  // Returns the current estimate of the robot's orientation. Note that this
  // filter does not have anything other than the gyro with which to estimate
  // the robot's yaw heading, and so it may need to be corrected for by upstream
  // filters.
  const Eigen::Quaternion<double> &X_hat() const { return X_hat_; }

  Eigen::Matrix<double, kNumMeasurements, 1> Z_hat() const { return Z_hat_; };

  Eigen::Matrix<double, 6, 1> pos_vel() const { return pos_vel_; }
  double yaw() const { return yaw_; }
  double avg_recent_yaw_rates() const {
    double avg = 0.0;
    for (const auto &yaw_rate : last_yaw_rates_) {
      avg += yaw_rate;
    };
    avg /= last_yaw_rates_.size();
    return avg;
  }
  Eigen::Matrix<double, 3, 1> avg_recent_accel() const {
    Eigen::Vector3d avg;
    avg.setZero();
    for (const auto &accel : last_accels_) {
      avg += accel;
    };
    avg /= last_accels_.size();
    return avg;
  }

  double gravity_magnitude() const { return gravity_magnitude_; }
  int consecutive_still() const { return consecutive_still_; }

  // Causes the down estimator to assume that gravity is exactly one g and that
  // the accelerometer readings have no meaningful noise. This is used for
  // dealing with tests where the IMU readings are actually nearly perfect.
  void assume_perfect_gravity() { assume_perfect_gravity_ = true; }

 protected:
  // Updates the position/velocity integration.
  void UpdatePosition(aos::monotonic_clock::duration dt,
                      const Eigen::Vector3d &accel);

  aos::monotonic_clock::time_point last_pos_vel_update_ =
      aos::monotonic_clock::min_time;

 private:
  // Length of buffer to maintain for averaging recent acceleration/gyro values.
  static constexpr size_t kBufferSize = 10;

  // Does all the heavy lifting from the Predict() call.
  void DoPredict(const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro,
                 const aos::monotonic_clock::duration dt);
  // Runs some cleanup that needs to happen at the end of any iteration.
  void IterationCleanup(const Eigen::Vector3d &accel,
                        const Eigen::Vector3d &gyro);

  // Takes in the current pos_vel_ vector as well as an acceleration in the
  // world-frame and returns the derivative. All numbers in m, m/s, or m/s/s.
  Eigen::Matrix<double, 6, 1> PosVelDerivative(
      const Eigen::Matrix<double, 6, 1> &pos_vel, const Eigen::Vector3d &accel);
  // Measurement Noise (Uncertainty)
  Eigen::Matrix<double, kNumInputs, kNumInputs> R_;
  // Model noise. Note that both this and P are 3 x 3 matrices, despite the
  // state having 4 dimensions.
  Eigen::Matrix<double, 3, 3> Q_;
  // Current estimate covariance.
  Eigen::Matrix<double, 3, 3> P_;

  // Current state estimate.
  Eigen::Quaternion<double> X_hat_;

  // Current expected accelerometer measurement.
  Eigen::Matrix<double, kNumMeasurements, 1> Z_hat_;

  // Current position and velocity vector in format:
  // {pos_x, pos_y, pos_z, vel_x, vel_y, vel_z}, in meters and meters / sec.
  // This is just used for cosmetic purposes, as it only accounts for IMU
  // measurements and so is prone to drift.
  Eigen::Matrix<double, 6, 1> pos_vel_;
  // Current yaw estimate, obtained purely by integrating the gyro measurements.
  double yaw_ = 0;
  // Circular buffer in which to store the most recent acceleration
  // measurements. These accelerations are transformed into the robot's yaw
  // frame and have gravity removed (i.e., the users of last_accels_ should not
  // have to worry about pitch/roll or the gravitational component of the
  // acceleration).
  // As such, x is the robot's longitudinal acceleration, y is the lateral
  // acceleration, and z is the up/down acceleration. All are in m/s/s.
  int buffer_index_ = 0;
  std::array<Eigen::Matrix<double, 3, 1>, kBufferSize> last_accels_;
  // Array of the most recent yaw rates, in rad/sec.
  std::array<double, kBufferSize> last_yaw_rates_;

  // Number of consecutive iterations in which we think that the robot has been
  // in a zero-acceleration state. We only accept accelerometer corrections to
  // the down estimator when we've been static for a sufficiently long time.
  int consecutive_still_ = 0;
  // This variable tracks the current estimated acceleration due to gravity, in
  // g's. This helps to compensate for both local variations in gravity as well
  // as for calibration errors on individual accelerometer axes.
  double gravity_magnitude_ = 1.0;

  // The transformation from the IMU's frame to the robot frame.
  Eigen::Matrix<double, 3, 3> imu_transform_;

  bool assume_perfect_gravity_ = false;
};

// TODO(james): The lines between DrivetrainUkf and QuaternionUkf have blurred
// to the point where there is minimal distinction. Either remove the
// unnecessary abstraction or figure out what we actually care about abstracting
// (e.g., we do eventually need to add some ability to provide a custom
// accelerometer calibration).
class DrivetrainUkf : public QuaternionUkf {
 public:
  DrivetrainUkf(const DrivetrainConfig<double> &dt_config)
      : QuaternionUkf(dt_config.imu_transform) {}
  // UKF for http://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf
  // Reference in case the link is dead:
  // Kraft, Edgar. "A quaternion-based unscented Kalman filter for orientation
  // tracking." In Proceedings of the Sixth International Conference of
  // Information Fusion, vol. 1, pp. 47-54. 2003.

  // A good reference for quaternions is available at
  // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/
  //
  // A good reference for angular velocity vectors with quaternions is at
  // http://www.euclideanspace.com/physics/kinematics/angularvelocity/

  // Creates a rotational velocity vector to be integrated.
  //
  // omega is the rotational velocity vector in body coordinates.
  // q is a matrix with the compononents of the quaternion in it.
  //
  // Returns dq / dt
  static Eigen::Vector4d QuaternionDerivative(Eigen::Vector3d omega,
                                              const Eigen::Vector4d &q_matrix) {
    Eigen::Quaternion<double> q(q_matrix);

    Eigen::Quaternion<double> omega_q;
    omega_q.w() = 0.0;
    omega_q.vec() = 0.5 * (q * omega);

    Eigen::Quaternion<double> deriv = omega_q * q;
    return deriv.coeffs();
  }

  // Moves the robot by the provided rotation vector (U).
  Eigen::Matrix<double, kNumStates, 1> A(
      const Eigen::Matrix<double, kNumStates, 1> &X,
      const Eigen::Matrix<double, kNumInputs, 1> &U,
      const aos::monotonic_clock::duration dt) const override {
    return RungeKutta(
        std::bind(&QuaternionDerivative, U, std::placeholders::_1), X,
        aos::time::DurationInSeconds(dt));
  }

  // Returns the expected accelerometer measurement (which is just going to be
  // 1g downwards).
  Eigen::Matrix<double, kNumMeasurements, 1> H(
      const Eigen::Matrix<double, kNumStates, 1> &X) const override {
    // Assume that we expect to see a reading of (0, 0, 1) when flat on the
    // ground.
    // TODO(james): Figure out a calibration routine for managing the fact that
    // the accelerometer will not be perfectly oriented within the robot (or
    // determine that calibration routines would be unnecessary).
    Eigen::Quaternion<double> Xquat(X);
    Eigen::Matrix<double, 3, 1> gprime =
        Xquat.conjugate() * Eigen::Matrix<double, 3, 1>(0.0, 0.0, 1.0) * 1.0;
    return gprime;
  }

  void UpdateIntegratedPositions(aos::monotonic_clock::time_point now);

  flatbuffers::Offset<DownEstimatorState> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb,
      aos::monotonic_clock::time_point now);
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_
