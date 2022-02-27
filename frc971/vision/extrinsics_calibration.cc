#include "frc971/vision/extrinsics_calibration.h"

#include "aos/time/time.h"
#include "ceres/ceres.h"
#include "frc971/analysis/in_process_plotter.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/vision/calibration_accumulator.h"
#include "frc971/vision/charuco_lib.h"

namespace frc971 {
namespace vision {

namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

constexpr double kGravity = 9.8;

// The basic ideas here are taken from Kalibr.
// (https://github.com/ethz-asl/kalibr), but adapted to work with AOS, and to be
// simpler.
//
// Camera readings and IMU readings come in at different times, on different
// time scales.  Our first problem is to align them in time so we can actually
// compute an error.  This is done in the calibration accumulator code.  The
// kalibr paper uses splines, while this uses kalman filters to solve the same
// interpolation problem so we can get the expected vs actual pose at the time
// each image arrives.
//
// The cost function is then fed the computed angular and positional error for
// each camera sample before the kalman filter update.  Intuitively, the smaller
// the corrections to the kalman filter each step, the better the estimate
// should be.
//
// We don't actually implement the angular kalman filter because the IMU is so
// good.  We give the solver an initial position and bias, and let it solve from
// there.  This lets us represent drift that is linear in time, which should be
// good enough for ~1 minute calibration.
//
// TODO(austin): Kalman smoother ala
// https://stanford.edu/~boyd/papers/pdf/auto_ks.pdf should allow for better
// parallelism, and since we aren't causal, will take that into account a lot
// better.

// This class takes the initial parameters and biases, and computes the error
// between the measured and expected camera readings.  When optimized, this
// gives us a cost function to minimize.
template <typename Scalar>
class CeresPoseFilter : public CalibrationDataObserver {
 public:
  typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Affine3s;

  CeresPoseFilter(Eigen::Quaternion<Scalar> initial_orientation,
                  Eigen::Quaternion<Scalar> pivot_to_camera,
                  Eigen::Quaternion<Scalar> pivot_to_imu,
                  Eigen::Matrix<Scalar, 3, 1> gyro_bias,
                  Eigen::Matrix<Scalar, 6, 1> initial_state,
                  Eigen::Quaternion<Scalar> board_to_world,
                  Eigen::Matrix<Scalar, 3, 1> pivot_to_camera_translation,
                  Eigen::Matrix<Scalar, 3, 1> pivot_to_imu_translation,
                  Scalar gravity_scalar,
                  Eigen::Matrix<Scalar, 3, 1> accelerometer_bias)
      : accel_(Eigen::Matrix<double, 3, 1>::Zero()),
        omega_(Eigen::Matrix<double, 3, 1>::Zero()),
        imu_bias_(gyro_bias),
        orientation_(initial_orientation),
        x_hat_(initial_state),
        p_(Eigen::Matrix<Scalar, 6, 6>::Zero()),
        pivot_to_camera_rotation_(pivot_to_camera),
        pivot_to_camera_translation_(pivot_to_camera_translation),
        pivot_to_imu_rotation_(pivot_to_imu),
        pivot_to_imu_translation_(pivot_to_imu_translation),
        board_to_world_(board_to_world),
        gravity_scalar_(gravity_scalar),
        accelerometer_bias_(accelerometer_bias) {}

  Scalar gravity_scalar() { return gravity_scalar_; }

  virtual void ObserveCameraUpdate(
      distributed_clock::time_point /*t*/,
      Eigen::Vector3d /*board_to_camera_rotation*/,
      Eigen::Quaternion<Scalar> /*imu_to_world_rotation*/,
      Affine3s /*imu_to_world*/) {}

  void UpdateTurret(distributed_clock::time_point t,
                    Eigen::Vector2d state) override {
    state_ = state;
    state_time_ = t;
  }

  Eigen::Vector2d state_ = Eigen::Vector2d::Zero();
  distributed_clock::time_point state_time_ = distributed_clock::min_time;

  // Observes a camera measurement by applying a kalman filter correction and
  // accumulating up the error associated with the step.
  void UpdateCamera(distributed_clock::time_point t,
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) override {
    Integrate(t);

    const double pivot_angle =
        state_time_ == distributed_clock::min_time
            ? 0.0
            : state_(0) +
                  state_(1) * chrono::duration<double>(t - state_time_).count();

    const Eigen::Quaternion<Scalar> board_to_camera_rotation(
        frc971::controls::ToQuaternionFromRotationVector(rt.first)
            .cast<Scalar>());
    const Affine3s board_to_camera =
        Eigen::Translation3d(rt.second).cast<Scalar>() *
        board_to_camera_rotation;

    const Affine3s pivot_to_camera =
        pivot_to_camera_translation_ * pivot_to_camera_rotation_;
    const Affine3s pivot_to_imu =
        pivot_to_imu_translation_ * pivot_to_imu_rotation_;

    // This converts us from (facing the board),
    //   x right, y up, z towards us -> x right, y away, z up.
    // Confirmed to be right.

    // Want world -> imu rotation.
    // world <- board <- camera <- imu.
    const Eigen::Quaternion<Scalar> imu_to_world_rotation =
        board_to_world_ * board_to_camera_rotation.inverse() *
        pivot_to_camera_rotation_ *
        Eigen::AngleAxis<Scalar>(static_cast<Scalar>(-pivot_angle),
                                 Eigen::Vector3d::UnitZ().cast<Scalar>()) *
        pivot_to_imu_rotation_.inverse();

    const Affine3s imu_to_world =
        board_to_world_ * board_to_camera.inverse() * pivot_to_camera *
        Eigen::AngleAxis<Scalar>(static_cast<Scalar>(-pivot_angle),
                                 Eigen::Vector3d::UnitZ().cast<Scalar>()) *
        pivot_to_imu.inverse();

    const Eigen::Matrix<Scalar, 3, 1> z =
        imu_to_world * Eigen::Matrix<Scalar, 3, 1>::Zero();

    Eigen::Matrix<Scalar, 3, 6> H = Eigen::Matrix<Scalar, 3, 6>::Zero();
    H(0, 0) = static_cast<Scalar>(1.0);
    H(1, 1) = static_cast<Scalar>(1.0);
    H(2, 2) = static_cast<Scalar>(1.0);
    const Eigen::Matrix<Scalar, 3, 1> y = z - H * x_hat_;

    const Eigen::Matrix<double, 3, 3> R =
        (::Eigen::DiagonalMatrix<double, 3>().diagonal() << ::std::pow(0.01, 2),
         ::std::pow(0.01, 2), ::std::pow(0.01, 2))
            .finished()
            .asDiagonal();

    const Eigen::Matrix<Scalar, 3, 3> S =
        H * p_ * H.transpose() + R.cast<Scalar>();
    const Eigen::Matrix<Scalar, 6, 3> K = p_ * H.transpose() * S.inverse();

    x_hat_ += K * y;
    p_ = (Eigen::Matrix<Scalar, 6, 6>::Identity() - K * H) * p_;

    const Eigen::Quaternion<Scalar> error(imu_to_world_rotation.inverse() *
                                          orientation());

    errors_.emplace_back(
        Eigen::Matrix<Scalar, 3, 1>(error.x(), error.y(), error.z()));
    position_errors_.emplace_back(y);

    ObserveCameraUpdate(t, rt.first, imu_to_world_rotation, imu_to_world);
  }

  virtual void ObserveIMUUpdate(
      distributed_clock::time_point /*t*/,
      std::pair<Eigen::Vector3d, Eigen::Vector3d> /*wa*/) {}

  void UpdateIMU(distributed_clock::time_point t,
                 std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) override {
    Integrate(t);
    omega_ = wa.first;
    accel_ = wa.second;

    ObserveIMUUpdate(t, wa);
  }

  const Eigen::Quaternion<Scalar> &orientation() const { return orientation_; }

  size_t num_errors() const { return errors_.size(); }
  Scalar errorx(size_t i) const { return errors_[i].x(); }
  Scalar errory(size_t i) const { return errors_[i].y(); }
  Scalar errorz(size_t i) const { return errors_[i].z(); }

  size_t num_perrors() const { return position_errors_.size(); }
  Scalar errorpx(size_t i) const { return position_errors_[i].x(); }
  Scalar errorpy(size_t i) const { return position_errors_[i].y(); }
  Scalar errorpz(size_t i) const { return position_errors_[i].z(); }

 private:
  Eigen::Matrix<Scalar, 46, 1> Pack(Eigen::Quaternion<Scalar> q,
                                    Eigen::Matrix<Scalar, 6, 1> x_hat,
                                    Eigen::Matrix<Scalar, 6, 6> p) {
    Eigen::Matrix<Scalar, 46, 1> result = Eigen::Matrix<Scalar, 46, 1>::Zero();
    result.template block<4, 1>(0, 0) = q.coeffs();
    result.template block<6, 1>(4, 0) = x_hat;
    result.template block<36, 1>(10, 0) =
        Eigen::Map<Eigen::Matrix<Scalar, 36, 1>>(p.data(), p.size());

    return result;
  }

  std::tuple<Eigen::Quaternion<Scalar>, Eigen::Matrix<Scalar, 6, 1>,
             Eigen::Matrix<Scalar, 6, 6>>
  UnPack(Eigen::Matrix<Scalar, 46, 1> input) {
    Eigen::Quaternion<Scalar> q(input.template block<4, 1>(0, 0));
    Eigen::Matrix<Scalar, 6, 1> x_hat(input.template block<6, 1>(4, 0));
    Eigen::Matrix<Scalar, 6, 6> p =
        Eigen::Map<Eigen::Matrix<Scalar, 6, 6>>(input.data() + 10, 6, 6);
    return std::make_tuple(q, x_hat, p);
  }

  Eigen::Matrix<Scalar, 46, 1> Derivative(
      const Eigen::Matrix<Scalar, 46, 1> &input) {
    auto [q, x_hat, p] = UnPack(input);

    Eigen::Quaternion<Scalar> omega_q;
    omega_q.w() = Scalar(0.0);
    omega_q.vec() = 0.5 * (omega_.cast<Scalar>() - imu_bias_);
    Eigen::Matrix<Scalar, 4, 1> q_dot = (q * omega_q).coeffs();

    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A(0, 3) = 1.0;
    A(1, 4) = 1.0;
    A(2, 5) = 1.0;

    Eigen::Matrix<Scalar, 6, 1> x_hat_dot = A * x_hat;
    x_hat_dot.template block<3, 1>(3, 0) =
        orientation() * (accel_.cast<Scalar>() - accelerometer_bias_) -
        Eigen::Vector3d(0, 0, kGravity).cast<Scalar>() * gravity_scalar_;

    // Initialize the position noise to 0.  If the solver is going to back-solve
    // for the most likely starting position, let's just say that the noise is
    // small.
    constexpr double kPositionNoise = 0.0;
    constexpr double kAccelerometerNoise = 2.3e-6 * 9.8;
    constexpr double kIMUdt = 5.0e-4;
    Eigen::Matrix<double, 6, 6> Q_dot(
        (::Eigen::DiagonalMatrix<double, 6>().diagonal()
             << ::std::pow(kPositionNoise, 2) / kIMUdt,
         ::std::pow(kPositionNoise, 2) / kIMUdt,
         ::std::pow(kPositionNoise, 2) / kIMUdt,
         ::std::pow(kAccelerometerNoise, 2) / kIMUdt,
         ::std::pow(kAccelerometerNoise, 2) / kIMUdt,
         ::std::pow(kAccelerometerNoise, 2) / kIMUdt)
            .finished()
            .asDiagonal());
    Eigen::Matrix<Scalar, 6, 6> p_dot = A.cast<Scalar>() * p +
                                        p * A.transpose().cast<Scalar>() +
                                        Q_dot.cast<Scalar>();

    return Pack(Eigen::Quaternion<Scalar>(q_dot), x_hat_dot, p_dot);
  }

  virtual void ObserveIntegrated(distributed_clock::time_point /*t*/,
                                 Eigen::Matrix<Scalar, 6, 1> /*x_hat*/,
                                 Eigen::Quaternion<Scalar> /*orientation*/,
                                 Eigen::Matrix<Scalar, 6, 6> /*p*/) {}

  void Integrate(distributed_clock::time_point t) {
    if (last_time_ != distributed_clock::min_time) {
      Eigen::Matrix<Scalar, 46, 1> next = control_loops::RungeKutta(
          [this](auto r) { return Derivative(r); },
          Pack(orientation_, x_hat_, p_),
          aos::time::DurationInSeconds(t - last_time_));

      std::tie(orientation_, x_hat_, p_) = UnPack(next);

      // Normalize q so it doesn't drift.
      orientation_.normalize();
    }

    last_time_ = t;
    ObserveIntegrated(t, x_hat_, orientation_, p_);
  }

  Eigen::Matrix<double, 3, 1> accel_;
  Eigen::Matrix<double, 3, 1> omega_;
  Eigen::Matrix<Scalar, 3, 1> imu_bias_;

  // IMU -> world quaternion
  Eigen::Quaternion<Scalar> orientation_;
  Eigen::Matrix<Scalar, 6, 1> x_hat_;
  Eigen::Matrix<Scalar, 6, 6> p_;
  distributed_clock::time_point last_time_ = distributed_clock::min_time;

  Eigen::Quaternion<Scalar> pivot_to_camera_rotation_;
  Eigen::Translation<Scalar, 3> pivot_to_camera_translation_ =
      Eigen::Translation3d(0, 0, 0).cast<Scalar>();

  Eigen::Quaternion<Scalar> pivot_to_imu_rotation_;
  Eigen::Translation<Scalar, 3> pivot_to_imu_translation_ =
      Eigen::Translation3d(0, 0, 0).cast<Scalar>();

  Eigen::Quaternion<Scalar> board_to_world_;
  Scalar gravity_scalar_;
  Eigen::Matrix<Scalar, 3, 1> accelerometer_bias_;
  // States:
  //   xyz position
  //   xyz velocity
  //
  // Inputs
  //   xyz accel
  //
  // Measurement:
  //   xyz position from camera.
  //
  // Since the gyro is so good, we can just solve for the bias and initial
  // position with the solver and see what it learns.

  // Returns the angular errors for each camera sample.
  std::vector<Eigen::Matrix<Scalar, 3, 1>> errors_;
  std::vector<Eigen::Matrix<Scalar, 3, 1>> position_errors_;
};

// Drives the Z coordinate of the quaternion to 0.
struct PenalizeQuaternionZ {
  template <typename S>
  bool operator()(const S *const pivot_to_imu_ptr, S *residual) const {
    Eigen::Quaternion<S> pivot_to_imu(pivot_to_imu_ptr[3], pivot_to_imu_ptr[0],
                                      pivot_to_imu_ptr[1], pivot_to_imu_ptr[2]);
    residual[0] = pivot_to_imu.z();
    return true;
  }
};

// Subclass of the filter above which has plotting.  This keeps debug code and
// actual code separate.
class PoseFilter : public CeresPoseFilter<double> {
 public:
  PoseFilter(Eigen::Quaternion<double> initial_orientation,
             Eigen::Quaternion<double> pivot_to_camera,
             Eigen::Quaternion<double> pivot_to_imu,
             Eigen::Matrix<double, 3, 1> gyro_bias,
             Eigen::Matrix<double, 6, 1> initial_state,
             Eigen::Quaternion<double> board_to_world,
             Eigen::Matrix<double, 3, 1> pivot_to_camera_translation,
             Eigen::Matrix<double, 3, 1> pivot_to_imu_translation,
             double gravity_scalar,
             Eigen::Matrix<double, 3, 1> accelerometer_bias)
      : CeresPoseFilter<double>(
            initial_orientation, pivot_to_camera, pivot_to_imu, gyro_bias,
            initial_state, board_to_world, pivot_to_camera_translation,
            pivot_to_imu_translation, gravity_scalar, accelerometer_bias) {}

  void Plot() {
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> rz;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    for (const Eigen::Quaternion<double> &q : orientations_) {
      Eigen::Matrix<double, 3, 1> rotation_vector =
          frc971::controls::ToRotationVectorFromQuaternion(q);
      rx.emplace_back(rotation_vector(0, 0));
      ry.emplace_back(rotation_vector(1, 0));
      rz.emplace_back(rotation_vector(2, 0));
    }
    for (const Eigen::Matrix<double, 6, 1> &x_hat : x_hats_) {
      x.emplace_back(x_hat(0));
      y.emplace_back(x_hat(1));
      z.emplace_back(x_hat(2));
      vx.emplace_back(x_hat(3));
      vy.emplace_back(x_hat(4));
      vz.emplace_back(x_hat(5));
    }

    frc971::analysis::Plotter plotter;
    plotter.AddFigure("position");
    plotter.AddLine(times_, rx, "x_hat(0)");
    plotter.AddLine(times_, ry, "x_hat(1)");
    plotter.AddLine(times_, rz, "x_hat(2)");
    plotter.AddLine(camera_times_, camera_x_, "Camera x");
    plotter.AddLine(camera_times_, camera_y_, "Camera y");
    plotter.AddLine(camera_times_, camera_z_, "Camera z");
    plotter.AddLine(camera_times_, camera_error_x_, "Camera error x");
    plotter.AddLine(camera_times_, camera_error_y_, "Camera error y");
    plotter.AddLine(camera_times_, camera_error_z_, "Camera error z");
    plotter.Publish();

    plotter.AddFigure("error");
    plotter.AddLine(times_, rx, "x_hat(0)");
    plotter.AddLine(times_, ry, "x_hat(1)");
    plotter.AddLine(times_, rz, "x_hat(2)");
    plotter.AddLine(camera_times_, camera_error_x_, "Camera error x");
    plotter.AddLine(camera_times_, camera_error_y_, "Camera error y");
    plotter.AddLine(camera_times_, camera_error_z_, "Camera error z");
    plotter.Publish();

    plotter.AddFigure("imu");
    plotter.AddLine(camera_times_, world_gravity_x_, "world_gravity(0)");
    plotter.AddLine(camera_times_, world_gravity_y_, "world_gravity(1)");
    plotter.AddLine(camera_times_, world_gravity_z_, "world_gravity(2)");
    plotter.AddLine(imu_times_, imu_x_, "imu x");
    plotter.AddLine(imu_times_, imu_y_, "imu y");
    plotter.AddLine(imu_times_, imu_z_, "imu z");
    plotter.AddLine(times_, rx, "rotation x");
    plotter.AddLine(times_, ry, "rotation y");
    plotter.AddLine(times_, rz, "rotation z");
    plotter.Publish();

    plotter.AddFigure("raw");
    plotter.AddLine(imu_times_, imu_x_, "imu x");
    plotter.AddLine(imu_times_, imu_y_, "imu y");
    plotter.AddLine(imu_times_, imu_z_, "imu z");
    plotter.AddLine(imu_times_, imu_ratex_, "omega x");
    plotter.AddLine(imu_times_, imu_ratey_, "omega y");
    plotter.AddLine(imu_times_, imu_ratez_, "omega z");
    plotter.AddLine(camera_times_, raw_camera_x_, "Camera x");
    plotter.AddLine(camera_times_, raw_camera_y_, "Camera y");
    plotter.AddLine(camera_times_, raw_camera_z_, "Camera z");
    plotter.Publish();

    plotter.AddFigure("xyz vel");
    plotter.AddLine(times_, x, "x");
    plotter.AddLine(times_, y, "y");
    plotter.AddLine(times_, z, "z");
    plotter.AddLine(times_, vx, "vx");
    plotter.AddLine(times_, vy, "vy");
    plotter.AddLine(times_, vz, "vz");
    plotter.AddLine(camera_times_, camera_position_x_, "Camera x");
    plotter.AddLine(camera_times_, camera_position_y_, "Camera y");
    plotter.AddLine(camera_times_, camera_position_z_, "Camera z");
    plotter.Publish();

    plotter.Spin();
  }

  void ObserveIntegrated(distributed_clock::time_point t,
                         Eigen::Matrix<double, 6, 1> x_hat,
                         Eigen::Quaternion<double> orientation,
                         Eigen::Matrix<double, 6, 6> p) override {
    VLOG(1) << t << " -> " << p;
    VLOG(1) << t << " xhat -> " << x_hat.transpose();
    times_.emplace_back(chrono::duration<double>(t.time_since_epoch()).count());
    x_hats_.emplace_back(x_hat);
    orientations_.emplace_back(orientation);
  }

  void ObserveIMUUpdate(
      distributed_clock::time_point t,
      std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) override {
    imu_times_.emplace_back(chrono::duration<double>(t.time_since_epoch()).count());
    imu_ratex_.emplace_back(wa.first.x());
    imu_ratey_.emplace_back(wa.first.y());
    imu_ratez_.emplace_back(wa.first.z());
    imu_x_.emplace_back(wa.second.x());
    imu_y_.emplace_back(wa.second.y());
    imu_z_.emplace_back(wa.second.z());

    last_accel_ = wa.second;
  }

  void ObserveCameraUpdate(distributed_clock::time_point t,
                           Eigen::Vector3d board_to_camera_rotation,
                           Eigen::Quaternion<double> imu_to_world_rotation,
                           Eigen::Affine3d imu_to_world) override {
    raw_camera_x_.emplace_back(board_to_camera_rotation(0, 0));
    raw_camera_y_.emplace_back(board_to_camera_rotation(1, 0));
    raw_camera_z_.emplace_back(board_to_camera_rotation(2, 0));

    Eigen::Matrix<double, 3, 1> rotation_vector =
        frc971::controls::ToRotationVectorFromQuaternion(imu_to_world_rotation);
    camera_times_.emplace_back(
        chrono::duration<double>(t.time_since_epoch()).count());

    Eigen::Matrix<double, 3, 1> camera_error =
        frc971::controls::ToRotationVectorFromQuaternion(
            imu_to_world_rotation.inverse() * orientation());

    camera_x_.emplace_back(rotation_vector(0, 0));
    camera_y_.emplace_back(rotation_vector(1, 0));
    camera_z_.emplace_back(rotation_vector(2, 0));

    camera_error_x_.emplace_back(camera_error(0, 0));
    camera_error_y_.emplace_back(camera_error(1, 0));
    camera_error_z_.emplace_back(camera_error(2, 0));

    const Eigen::Vector3d world_gravity =
        imu_to_world_rotation * last_accel_ -
        Eigen::Vector3d(0, 0, kGravity) * gravity_scalar();

    const Eigen::Vector3d camera_position =
        imu_to_world * Eigen::Vector3d::Zero();

    world_gravity_x_.emplace_back(world_gravity.x());
    world_gravity_y_.emplace_back(world_gravity.y());
    world_gravity_z_.emplace_back(world_gravity.z());

    camera_position_x_.emplace_back(camera_position.x());
    camera_position_y_.emplace_back(camera_position.y());
    camera_position_z_.emplace_back(camera_position.z());
  }

  std::vector<double> camera_times_;
  std::vector<double> camera_x_;
  std::vector<double> camera_y_;
  std::vector<double> camera_z_;
  std::vector<double> raw_camera_x_;
  std::vector<double> raw_camera_y_;
  std::vector<double> raw_camera_z_;
  std::vector<double> camera_error_x_;
  std::vector<double> camera_error_y_;
  std::vector<double> camera_error_z_;

  std::vector<double> world_gravity_x_;
  std::vector<double> world_gravity_y_;
  std::vector<double> world_gravity_z_;
  std::vector<double> imu_x_;
  std::vector<double> imu_y_;
  std::vector<double> imu_z_;
  std::vector<double> camera_position_x_;
  std::vector<double> camera_position_y_;
  std::vector<double> camera_position_z_;

  std::vector<double> imu_times_;
  std::vector<double> imu_ratex_;
  std::vector<double> imu_ratey_;
  std::vector<double> imu_ratez_;

  std::vector<double> times_;
  std::vector<Eigen::Matrix<double, 6, 1>> x_hats_;
  std::vector<Eigen::Quaternion<double>> orientations_;

  Eigen::Matrix<double, 3, 1> last_accel_ = Eigen::Matrix<double, 3, 1>::Zero();
};

// Adapter class from the KF above to a Ceres cost function.
struct CostFunctor {
  CostFunctor(const CalibrationData *d) : data(d) {}

  const CalibrationData *data;

  template <typename S>
  bool operator()(const S *const initial_orientation_ptr,
                  const S *const pivot_to_camera_ptr,
                  const S *const pivot_to_imu_ptr, const S *const gyro_bias_ptr,
                  const S *const initial_state_ptr,
                  const S *const board_to_world_ptr,
                  const S *const pivot_to_camera_translation_ptr,
                  const S *const pivot_to_imu_translation_ptr,
                  const S *const gravity_scalar_ptr,
                  const S *const accelerometer_bias_ptr, S *residual) const {
    Eigen::Quaternion<S> initial_orientation(
        initial_orientation_ptr[3], initial_orientation_ptr[0],
        initial_orientation_ptr[1], initial_orientation_ptr[2]);
    Eigen::Quaternion<S> pivot_to_camera(
        pivot_to_camera_ptr[3], pivot_to_camera_ptr[0], pivot_to_camera_ptr[1],
        pivot_to_camera_ptr[2]);
    Eigen::Quaternion<S> pivot_to_imu(pivot_to_imu_ptr[3], pivot_to_imu_ptr[0],
                                      pivot_to_imu_ptr[1], pivot_to_imu_ptr[2]);
    Eigen::Quaternion<S> board_to_world(
        board_to_world_ptr[3], board_to_world_ptr[0], board_to_world_ptr[1],
        board_to_world_ptr[2]);
    Eigen::Matrix<S, 3, 1> gyro_bias(gyro_bias_ptr[0], gyro_bias_ptr[1],
                                     gyro_bias_ptr[2]);
    Eigen::Matrix<S, 6, 1> initial_state;
    initial_state(0) = initial_state_ptr[0];
    initial_state(1) = initial_state_ptr[1];
    initial_state(2) = initial_state_ptr[2];
    initial_state(3) = initial_state_ptr[3];
    initial_state(4) = initial_state_ptr[4];
    initial_state(5) = initial_state_ptr[5];
    Eigen::Matrix<S, 3, 1> pivot_to_camera_translation(
        pivot_to_camera_translation_ptr[0], pivot_to_camera_translation_ptr[1],
        pivot_to_camera_translation_ptr[2]);
    Eigen::Matrix<S, 3, 1> pivot_to_imu_translation(
        pivot_to_imu_translation_ptr[0], pivot_to_imu_translation_ptr[1],
        pivot_to_imu_translation_ptr[2]);
    Eigen::Matrix<S, 3, 1> accelerometer_bias(accelerometer_bias_ptr[0],
                                              accelerometer_bias_ptr[1],
                                              accelerometer_bias_ptr[2]);

    CeresPoseFilter<S> filter(
        initial_orientation, pivot_to_camera, pivot_to_imu, gyro_bias,
        initial_state, board_to_world, pivot_to_camera_translation,
        pivot_to_imu_translation, *gravity_scalar_ptr, accelerometer_bias);
    data->ReviewData(&filter);

    for (size_t i = 0; i < filter.num_errors(); ++i) {
      residual[3 * i + 0] = filter.errorx(i);
      residual[3 * i + 1] = filter.errory(i);
      residual[3 * i + 2] = filter.errorz(i);
    }

    for (size_t i = 0; i < filter.num_perrors(); ++i) {
      residual[3 * filter.num_errors() + 3 * i + 0] = filter.errorpx(i);
      residual[3 * filter.num_errors() + 3 * i + 1] = filter.errorpy(i);
      residual[3 * filter.num_errors() + 3 * i + 2] = filter.errorpz(i);
    }

    return true;
  }
};

void Solve(const CalibrationData &data,
           CalibrationParameters *calibration_parameters) {
  ceres::Problem problem;

  ceres::EigenQuaternionParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization();
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

  {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, ceres::DYNAMIC, 4, 4, 4, 3,
                                        6, 4, 3, 3, 1, 3>(
            new CostFunctor(&data), data.camera_samples_size() * 6);
    problem.AddResidualBlock(
        cost_function, new ceres::HuberLoss(1.0),
        calibration_parameters->initial_orientation.coeffs().data(),
        calibration_parameters->pivot_to_camera.coeffs().data(),
        calibration_parameters->pivot_to_imu.coeffs().data(),
        calibration_parameters->gyro_bias.data(),
        calibration_parameters->initial_state.data(),
        calibration_parameters->board_to_world.coeffs().data(),
        calibration_parameters->pivot_to_camera_translation.data(),
        calibration_parameters->pivot_to_imu_translation.data(),
        &calibration_parameters->gravity_scalar,
        calibration_parameters->accelerometer_bias.data());
  }

  {
    ceres::CostFunction *turret_z_cost_function =
        new ceres::AutoDiffCostFunction<PenalizeQuaternionZ, 1, 4>(
            new PenalizeQuaternionZ());
    problem.AddResidualBlock(
        turret_z_cost_function, nullptr,
        calibration_parameters->pivot_to_imu.coeffs().data());
  }

  if (calibration_parameters->has_pivot) {
    // Constrain Z.
    problem.SetParameterization(
        calibration_parameters->pivot_to_imu_translation.data(),
        new ceres::SubsetParameterization(3, {2}));
  } else {
    problem.SetParameterBlockConstant(
        calibration_parameters->pivot_to_imu.coeffs().data());
    problem.SetParameterBlockConstant(
        calibration_parameters->pivot_to_imu_translation.data());
  }

  problem.SetParameterization(
      calibration_parameters->initial_orientation.coeffs().data(),
      quaternion_local_parameterization);
  problem.SetParameterization(
      calibration_parameters->pivot_to_camera.coeffs().data(),
      quaternion_local_parameterization);
  problem.SetParameterization(
      calibration_parameters->board_to_world.coeffs().data(),
      quaternion_local_parameterization);
  for (int i = 0; i < 3; ++i) {
    problem.SetParameterLowerBound(calibration_parameters->gyro_bias.data(), i,
                                   -0.05);
    problem.SetParameterUpperBound(calibration_parameters->gyro_bias.data(), i,
                                   0.05);
    problem.SetParameterLowerBound(
        calibration_parameters->accelerometer_bias.data(), i, -0.05);
    problem.SetParameterUpperBound(
        calibration_parameters->accelerometer_bias.data(), i, 0.05);
  }
  problem.SetParameterLowerBound(&calibration_parameters->gravity_scalar, 0,
                                 0.95);
  problem.SetParameterUpperBound(&calibration_parameters->gravity_scalar, 0,
                                 1.05);

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-12;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-12;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  LOG(INFO) << "initial_orientation "
            << calibration_parameters->initial_orientation.coeffs().transpose();
  LOG(INFO) << "pivot_to_imu "
            << calibration_parameters->pivot_to_imu.coeffs().transpose();
  LOG(INFO) << "pivot_to_imu(rotation) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters->pivot_to_imu)
                   .transpose();
  LOG(INFO) << "pivot_to_camera "
            << calibration_parameters->pivot_to_camera.coeffs().transpose();
  LOG(INFO) << "pivot_to_camera(rotation) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters->pivot_to_camera)
                   .transpose();
  LOG(INFO) << "gyro_bias " << calibration_parameters->gyro_bias.transpose();
  LOG(INFO) << "board_to_world "
            << calibration_parameters->board_to_world.coeffs().transpose();
  LOG(INFO) << "board_to_world(rotation) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters->board_to_world)
                   .transpose();
  LOG(INFO) << "pivot_to_imu_translation "
            << calibration_parameters->pivot_to_imu_translation.transpose();
  LOG(INFO) << "pivot_to_camera_translation "
            << calibration_parameters->pivot_to_camera_translation.transpose();
  LOG(INFO) << "gravity " << kGravity * calibration_parameters->gravity_scalar;
  LOG(INFO) << "accelerometer bias "
            << calibration_parameters->accelerometer_bias.transpose();
}

void Plot(const CalibrationData &data,
          const CalibrationParameters &calibration_parameters) {
  PoseFilter filter(calibration_parameters.initial_orientation,
                    calibration_parameters.pivot_to_camera,
                    calibration_parameters.pivot_to_imu,
                    calibration_parameters.gyro_bias,
                    calibration_parameters.initial_state,
                    calibration_parameters.board_to_world,
                    calibration_parameters.pivot_to_camera_translation,
                    calibration_parameters.pivot_to_imu_translation,
                    calibration_parameters.gravity_scalar,
                    calibration_parameters.accelerometer_bias);
  data.ReviewData(&filter);
  filter.Plot();
}

}  // namespace vision
}  // namespace frc971
