#include "frc971/vision/extrinsics_calibration.h"

#include "ceres/ceres.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/time/time.h"
#include "frc971/analysis/in_process_plotter.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/vision/calibration_accumulator.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/visualize_robot.h"

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
      Eigen::Vector3d /*board_to_camera_translation*/,
      Eigen::Quaternion<Scalar> /*imu_to_world_rotation*/,
      Affine3s /*imu_to_world*/, double /*turret_angle*/) {}

  // Observes a camera measurement by applying a kalman filter correction and
  // accumulating up the error associated with the step.
  void UpdateCamera(distributed_clock::time_point t,
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) override {
    Integrate(t);

    const double pivot_angle =
        state_time_ == distributed_clock::min_time
            ? 0.0
            : turret_state_(0) +
                  turret_state_(1) *
                      chrono::duration<double>(t - state_time_).count();

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

    // TODO<Jim>: Need to understand dependence on this-- solutions vary by 20cm
    // when changing from 0.01 -> 0.1
    double obs_noise_var = ::std::pow(0.01, 2);
    const Eigen::Matrix<double, 3, 3> R =
        (::Eigen::DiagonalMatrix<double, 3>().diagonal() << obs_noise_var,
         obs_noise_var, obs_noise_var)
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

    ObserveCameraUpdate(t, rt.first, rt.second, imu_to_world_rotation,
                        imu_to_world, pivot_angle);
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

  virtual void ObserveTurretUpdate(distributed_clock::time_point /*t*/,
                                   Eigen::Vector2d /*turret_state*/) {}

  void UpdateTurret(distributed_clock::time_point t,
                    Eigen::Vector2d state) override {
    turret_state_ = state;
    state_time_ = t;

    ObserveTurretUpdate(t, state);
  }

  Eigen::Vector2d turret_state_ = Eigen::Vector2d::Zero();
  distributed_clock::time_point state_time_ = distributed_clock::min_time;

  const Eigen::Quaternion<Scalar> &orientation() const { return orientation_; }
  const Eigen::Matrix<Scalar, 6, 1> &get_x_hat() const { return x_hat_; }

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

    // TODO<Jim>: Could probably still do a bit more work on naming
    // conventions and what is being shown here
    frc971::analysis::Plotter plotter;
    plotter.AddFigure("bot (imu) position");
    plotter.AddLine(times_, x, "x_hat(0)");
    plotter.AddLine(times_, y, "x_hat(1)");
    plotter.AddLine(times_, z, "x_hat(2)");
    plotter.Publish();

    plotter.AddFigure("bot (imu) rotation");
    plotter.AddLine(camera_times_, imu_rot_x_, "bot (imu) rot x");
    plotter.AddLine(camera_times_, imu_rot_y_, "bot (imu) rot y");
    plotter.AddLine(camera_times_, imu_rot_z_, "bot (imu) rot z");
    plotter.Publish();

    plotter.AddFigure("rotation error");
    plotter.AddLine(camera_times_, rotation_error_x_, "Error x");
    plotter.AddLine(camera_times_, rotation_error_y_, "Error y");
    plotter.AddLine(camera_times_, rotation_error_z_, "Error z");
    plotter.Publish();

    plotter.AddFigure("translation error");
    plotter.AddLine(camera_times_, translation_error_x_, "Error x");
    plotter.AddLine(camera_times_, translation_error_y_, "Error y");
    plotter.AddLine(camera_times_, translation_error_z_, "Error z");
    plotter.Publish();

    plotter.AddFigure("imu");
    plotter.AddLine(imu_times_, imu_rate_x_, "imu gyro x");
    plotter.AddLine(imu_times_, imu_rate_y_, "imu gyro y");
    plotter.AddLine(imu_times_, imu_rate_z_, "imu gyro z");
    plotter.AddLine(imu_times_, imu_accel_x_, "imu accel x");
    plotter.AddLine(imu_times_, imu_accel_y_, "imu accel y");
    plotter.AddLine(imu_times_, imu_accel_z_, "imu accel z");
    plotter.AddLine(camera_times_, accel_minus_gravity_x_,
                    "accel_minus_gravity(0)");
    plotter.AddLine(camera_times_, accel_minus_gravity_y_,
                    "accel_minus_gravity(1)");
    plotter.AddLine(camera_times_, accel_minus_gravity_z_,
                    "accel_minus_gravity(2)");
    plotter.Publish();

    plotter.AddFigure("raw camera observations");
    plotter.AddLine(camera_times_, raw_camera_rot_x_, "Camera rot x");
    plotter.AddLine(camera_times_, raw_camera_rot_y_, "Camera rot y");
    plotter.AddLine(camera_times_, raw_camera_rot_z_, "Camera rot z");
    plotter.AddLine(camera_times_, raw_camera_trans_x_, "Camera trans x");
    plotter.AddLine(camera_times_, raw_camera_trans_y_, "Camera trans y");
    plotter.AddLine(camera_times_, raw_camera_trans_z_, "Camera trans z");
    plotter.Publish();

    plotter.AddFigure("xyz pos, vel estimates");
    plotter.AddLine(times_, x, "x (x_hat(0))");
    plotter.AddLine(times_, y, "y (x_hat(1))");
    plotter.AddLine(times_, z, "z (x_hat(2))");
    plotter.AddLine(times_, vx, "vx");
    plotter.AddLine(times_, vy, "vy");
    plotter.AddLine(times_, vz, "vz");
    plotter.AddLine(camera_times_, imu_position_x_, "x pos from board");
    plotter.AddLine(camera_times_, imu_position_y_, "y pos from board");
    plotter.AddLine(camera_times_, imu_position_z_, "z pos from board");
    plotter.Publish();

    // If we've got 'em, plot 'em
    if (turret_times_.size() > 0) {
      plotter.AddFigure("Turret angle");
      plotter.AddLine(turret_times_, turret_angles_, "turret angle");
      plotter.Publish();
    }

    plotter.Spin();
  }

  void Visualize(const CalibrationParameters &calibration_parameters) {
    // Set up virtual camera for visualization
    VisualizeRobot vis_robot;

    // Set virtual viewing point 10 meters above the origin, rotated so the
    // camera faces straight down
    Eigen::Translation3d camera_trans(0, 0, 10.0);
    Eigen::AngleAxisd camera_rot(M_PI, Eigen::Vector3d::UnitX());
    Eigen::Affine3d camera_viewpoint = camera_trans * camera_rot;
    vis_robot.SetViewpoint(camera_viewpoint);

    // Create camera with origin in center, and focal length suitable to fit
    // robot visualization fully in view
    int image_width = 500;
    double focal_length = 1000.0;
    double intr[] = {focal_length, 0.0,          image_width / 2.0,
                     0.0,          focal_length, image_width / 2.0,
                     0.0,          0.0,          1.0};
    cv::Mat camera_mat = cv::Mat(3, 3, CV_64FC1, intr);
    cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, 0.0);
    vis_robot.SetCameraParameters(camera_mat);
    vis_robot.SetDistortionCoefficients(dist_coeffs);

    uint current_state_index = 0;
    uint current_turret_index = 0;
    for (uint i = 0; i < camera_times_.size() - 1; i++) {
      // reset image each frame
      cv::Mat image_mat =
          cv::Mat::zeros(cv::Size(image_width, image_width), CV_8UC3);
      vis_robot.SetImage(image_mat);

      // Jump to state closest to current camera_time
      while (camera_times_[i] > times_[current_state_index] &&
             current_state_index < times_.size()) {
        current_state_index++;
      }

      // H_world_imu: map from world origin to imu (robot) frame
      Eigen::Vector3d T_world_imu_vec =
          x_hats_[current_state_index].block<3, 1>(0, 0);
      Eigen::Translation3d T_world_imu(T_world_imu_vec);
      Eigen::Affine3d H_world_imu =
          T_world_imu * orientations_[current_state_index];

      vis_robot.DrawFrameAxes(H_world_imu, "imu_kf");

      // H_world_pivot: map from world origin to pivot point
      // Do this via the imu (using H_world_pivot = H_world_imu * H_imu_pivot)
      Eigen::Quaterniond R_imu_pivot(calibration_parameters.pivot_to_imu);
      Eigen::Translation3d T_imu_pivot(
          calibration_parameters.pivot_to_imu_translation);
      Eigen::Affine3d H_imu_pivot = T_imu_pivot * R_imu_pivot;
      Eigen::Affine3d H_world_pivot = H_world_imu * H_imu_pivot;
      vis_robot.DrawFrameAxes(H_world_pivot, "pivot");

      // Jump to turret sample closest to current camera_time
      while (turret_times_.size() > 0 &&
             camera_times_[i] > turret_times_[current_turret_index] &&
             current_turret_index < turret_times_.size()) {
        current_turret_index++;
      }

      // Draw the camera frame

      Eigen::Affine3d H_imupivot_camerapivot(Eigen::Matrix4d::Identity());
      if (turret_angles_.size() > 0) {
        // Need to rotate by the turret angle in the middle of all this
        H_imupivot_camerapivot = Eigen::Affine3d(Eigen::AngleAxisd(
            turret_angles_[current_turret_index], Eigen::Vector3d::UnitZ()));
      }

      // H_world_camera: map from world origin to camera frame
      // Via imu->pivot->pivot rotation
      Eigen::Quaterniond R_camera_pivot(calibration_parameters.pivot_to_camera);
      Eigen::Translation3d T_camera_pivot(
          calibration_parameters.pivot_to_camera_translation);
      Eigen::Affine3d H_camera_pivot = T_camera_pivot * R_camera_pivot;
      Eigen::Affine3d H_world_camera = H_world_imu * H_imu_pivot *
                                       H_imupivot_camerapivot *
                                       H_camera_pivot.inverse();
      vis_robot.DrawFrameAxes(H_world_camera, "camera");

      // H_world_board: board location from world reference frame
      // Uses the estimate from camera-> board, on top of H_world_camera
      Eigen::Quaterniond R_camera_board(
          frc971::controls::ToQuaternionFromRotationVector(
              board_to_camera_rotations_[i]));
      Eigen::Translation3d T_camera_board(board_to_camera_translations_[i]);
      Eigen::Affine3d H_camera_board = T_camera_board * R_camera_board;
      Eigen::Affine3d H_world_board = H_world_camera * H_camera_board;

      vis_robot.DrawFrameAxes(H_world_board, "board est");

      // H_world_board_solve: board in world frame based on solver
      // Find world -> board via solved parameter of H_world_board
      // (parameter "board_to_world" and assuming origin of board frame is
      // coincident with origin of world frame, i.e., T_world_board == 0)
      Eigen::Quaterniond R_world_board_solve(
          calibration_parameters.board_to_world);
      Eigen::Translation3d T_world_board_solve(Eigen::Vector3d(0, 0, 0));
      Eigen::Affine3d H_world_board_solve =
          T_world_board_solve * R_world_board_solve;

      vis_robot.DrawFrameAxes(H_world_board_solve, "board_solve");

      // H_world_imu_from_board: imu location in world frame, via the board
      // Determine the imu location via the board_to_world solved
      // transformation
      Eigen::Affine3d H_world_imu_from_board =
          H_world_board_solve * H_camera_board.inverse() * H_camera_pivot *
          H_imupivot_camerapivot.inverse() * H_imu_pivot.inverse();

      vis_robot.DrawFrameAxes(H_world_imu_from_board, "imu_board");

      // These errors should match up with the residuals in the optimizer
      // (Note: rotation seems to differ by sign, but that's OK in residual)
      Eigen::Affine3d error = H_world_imu_from_board.inverse() * H_world_imu;
      Eigen::Vector3d trans_error =
          H_world_imu_from_board.translation() - H_world_imu.translation();
      Eigen::Quaterniond error_rot(error.rotation());
      VLOG(1) << "Error: \n"
              << "Rotation: " << error_rot.coeffs().transpose() << "\n"
              << "Translation: " << trans_error.transpose();

      cv::imshow("Live", image_mat);
      cv::waitKey(50);
    }
    LOG(INFO) << "Finished visualizing robot.  Press any key to continue";
    cv::waitKey();
  }

  void ObserveIntegrated(distributed_clock::time_point t,
                         Eigen::Matrix<double, 6, 1> x_hat,
                         Eigen::Quaternion<double> orientation,
                         Eigen::Matrix<double, 6, 6> p) override {
    VLOG(2) << t << " -> " << p;
    VLOG(2) << t << " xhat -> " << x_hat.transpose();
    times_.emplace_back(chrono::duration<double>(t.time_since_epoch()).count());
    x_hats_.emplace_back(x_hat);
    orientations_.emplace_back(orientation);
  }

  void ObserveIMUUpdate(
      distributed_clock::time_point t,
      std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) override {
    imu_times_.emplace_back(
        chrono::duration<double>(t.time_since_epoch()).count());
    imu_rate_x_.emplace_back(wa.first.x());
    imu_rate_y_.emplace_back(wa.first.y());
    imu_rate_z_.emplace_back(wa.first.z());
    imu_accel_x_.emplace_back(wa.second.x());
    imu_accel_y_.emplace_back(wa.second.y());
    imu_accel_z_.emplace_back(wa.second.z());

    last_accel_ = wa.second;
  }

  void ObserveCameraUpdate(distributed_clock::time_point t,
                           Eigen::Vector3d board_to_camera_rotation,
                           Eigen::Vector3d board_to_camera_translation,
                           Eigen::Quaternion<double> imu_to_world_rotation,
                           Eigen::Affine3d imu_to_world,
                           double turret_angle) override {
    board_to_camera_rotations_.emplace_back(board_to_camera_rotation);
    board_to_camera_translations_.emplace_back(board_to_camera_translation);

    camera_times_.emplace_back(
        chrono::duration<double>(t.time_since_epoch()).count());

    raw_camera_rot_x_.emplace_back(board_to_camera_rotation(0, 0));
    raw_camera_rot_y_.emplace_back(board_to_camera_rotation(1, 0));
    raw_camera_rot_z_.emplace_back(board_to_camera_rotation(2, 0));

    raw_camera_trans_x_.emplace_back(board_to_camera_translation(0, 0));
    raw_camera_trans_y_.emplace_back(board_to_camera_translation(1, 0));
    raw_camera_trans_z_.emplace_back(board_to_camera_translation(2, 0));

    Eigen::Matrix<double, 3, 1> rotation_vector =
        frc971::controls::ToRotationVectorFromQuaternion(imu_to_world_rotation);
    imu_rot_x_.emplace_back(rotation_vector(0, 0));
    imu_rot_y_.emplace_back(rotation_vector(1, 0));
    imu_rot_z_.emplace_back(rotation_vector(2, 0));

    Eigen::Matrix<double, 3, 1> rotation_error =
        frc971::controls::ToRotationVectorFromQuaternion(
            imu_to_world_rotation.inverse() * orientation());

    rotation_error_x_.emplace_back(rotation_error(0, 0));
    rotation_error_y_.emplace_back(rotation_error(1, 0));
    rotation_error_z_.emplace_back(rotation_error(2, 0));

    Eigen::Matrix<double, 3, 1> imu_pos = get_x_hat().block<3, 1>(0, 0);
    Eigen::Translation3d T_world_imu(imu_pos);
    Eigen::Affine3d H_world_imu = T_world_imu * orientation();
    Eigen::Affine3d H_error = imu_to_world.inverse() * H_world_imu;

    Eigen::Matrix<double, 3, 1> translation_error = H_error.translation();
    translation_error_x_.emplace_back(translation_error(0, 0));
    translation_error_y_.emplace_back(translation_error(1, 0));
    translation_error_z_.emplace_back(translation_error(2, 0));

    const Eigen::Vector3d accel_minus_gravity =
        imu_to_world_rotation * last_accel_ -
        Eigen::Vector3d(0, 0, kGravity) * gravity_scalar();

    accel_minus_gravity_x_.emplace_back(accel_minus_gravity.x());
    accel_minus_gravity_y_.emplace_back(accel_minus_gravity.y());
    accel_minus_gravity_z_.emplace_back(accel_minus_gravity.z());

    const Eigen::Vector3d imu_position = imu_to_world * Eigen::Vector3d::Zero();

    imu_position_x_.emplace_back(imu_position.x());
    imu_position_y_.emplace_back(imu_position.y());
    imu_position_z_.emplace_back(imu_position.z());

    turret_angles_from_camera_.emplace_back(turret_angle);
    imu_to_world_save_.emplace_back(imu_to_world);
  }

  void ObserveTurretUpdate(distributed_clock::time_point t,
                           Eigen::Vector2d turret_state) override {
    turret_times_.emplace_back(
        chrono::duration<double>(t.time_since_epoch()).count());
    turret_angles_.emplace_back(turret_state(0));
  }

  std::vector<double> camera_times_;
  std::vector<double> imu_rot_x_;
  std::vector<double> imu_rot_y_;
  std::vector<double> imu_rot_z_;
  std::vector<double> raw_camera_rot_x_;
  std::vector<double> raw_camera_rot_y_;
  std::vector<double> raw_camera_rot_z_;
  std::vector<double> raw_camera_trans_x_;
  std::vector<double> raw_camera_trans_y_;
  std::vector<double> raw_camera_trans_z_;
  std::vector<double> rotation_error_x_;
  std::vector<double> rotation_error_y_;
  std::vector<double> rotation_error_z_;
  std::vector<double> translation_error_x_;
  std::vector<double> translation_error_y_;
  std::vector<double> translation_error_z_;
  std::vector<Eigen::Vector3d> board_to_camera_rotations_;
  std::vector<Eigen::Vector3d> board_to_camera_translations_;

  std::vector<double> turret_angles_from_camera_;
  std::vector<Eigen::Affine3d> imu_to_world_save_;

  std::vector<double> imu_position_x_;
  std::vector<double> imu_position_y_;
  std::vector<double> imu_position_z_;

  std::vector<double> imu_times_;
  std::vector<double> imu_rate_x_;
  std::vector<double> imu_rate_y_;
  std::vector<double> imu_rate_z_;
  std::vector<double> accel_minus_gravity_x_;
  std::vector<double> accel_minus_gravity_y_;
  std::vector<double> accel_minus_gravity_z_;
  std::vector<double> imu_accel_x_;
  std::vector<double> imu_accel_y_;
  std::vector<double> imu_accel_z_;

  std::vector<double> turret_times_;
  std::vector<double> turret_angles_;

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
    const aos::monotonic_clock::time_point start_time =
        aos::monotonic_clock::now();
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

    // Since the angular error scale is bounded by 1 (quaternion, so unit
    // vector, scaled by sin(alpha)), I found it necessary to scale the
    // angular error to have it properly balance with the translational error
    double ang_error_scale = 5.0;
    for (size_t i = 0; i < filter.num_errors(); ++i) {
      residual[3 * i + 0] = ang_error_scale * filter.errorx(i);
      residual[3 * i + 1] = ang_error_scale * filter.errory(i);
      residual[3 * i + 2] = ang_error_scale * filter.errorz(i);
    }

    double trans_error_scale = 1.0;
    for (size_t i = 0; i < filter.num_perrors(); ++i) {
      residual[3 * filter.num_errors() + 3 * i + 0] =
          trans_error_scale * filter.errorpx(i);
      residual[3 * filter.num_errors() + 3 * i + 1] =
          trans_error_scale * filter.errorpy(i);
      residual[3 * filter.num_errors() + 3 * i + 2] =
          trans_error_scale * filter.errorpz(i);
    }

    VLOG(2) << "Cost function calc took "
            << chrono::duration<double>(aos::monotonic_clock::now() -
                                        start_time)
                   .count()
            << " seconds";

    return true;
  }
};

std::vector<float> MatrixToVector(const Eigen::Matrix<double, 4, 4> &H) {
  std::vector<float> data;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      data.push_back(H(row, col));
    }
  }
  return data;
}

aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> Solve(
    const CalibrationData &data,
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

  if (calibration_parameters->has_pivot) {
    // The turret's Z rotation is redundant with the camera's mounting z
    // rotation since it's along the rotation axis.
    ceres::CostFunction *turret_z_cost_function =
        new ceres::AutoDiffCostFunction<PenalizeQuaternionZ, 1, 4>(
            new PenalizeQuaternionZ());
    problem.AddResidualBlock(
        turret_z_cost_function, nullptr,
        calibration_parameters->pivot_to_imu.coeffs().data());
  }

  if (calibration_parameters->has_pivot) {
    // Constrain Z since it's along the rotation axis and therefore
    // redundant.
    problem.SetParameterization(
        calibration_parameters->pivot_to_imu_translation.data(),
        new ceres::SubsetParameterization(3, {2}));
  } else {
    problem.SetParameterBlockConstant(
        calibration_parameters->pivot_to_imu.coeffs().data());
    problem.SetParameterBlockConstant(
        calibration_parameters->pivot_to_imu_translation.data());
  }

  {
    // The board rotation in z is a bit arbitrary, so hoping to limit it to
    // increase repeatability
    ceres::CostFunction *board_z_cost_function =
        new ceres::AutoDiffCostFunction<PenalizeQuaternionZ, 1, 4>(
            new PenalizeQuaternionZ());
    problem.AddResidualBlock(
        board_z_cost_function, nullptr,
        calibration_parameters->board_to_world.coeffs().data());
  }

  problem.SetParameterization(
      calibration_parameters->initial_orientation.coeffs().data(),
      quaternion_local_parameterization);
  problem.SetParameterization(
      calibration_parameters->pivot_to_camera.coeffs().data(),
      quaternion_local_parameterization);
  problem.SetParameterization(
      calibration_parameters->pivot_to_imu.coeffs().data(),
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
  options.gradient_tolerance = 1e-6;
  options.function_tolerance = 1e-6;
  options.parameter_tolerance = 1e-6;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();
  LOG(INFO) << "Solution is " << (summary.IsSolutionUsable() ? "" : "NOT ")
            << "usable";

  {
    flatbuffers::FlatBufferBuilder fbb;
    flatbuffers::Offset<flatbuffers::Vector<float>> data_offset =
        fbb.CreateVector(MatrixToVector(
            (Eigen::Translation3d(
                 calibration_parameters->pivot_to_camera_translation) *
             Eigen::Quaterniond(calibration_parameters->pivot_to_camera))
                .inverse()
                .matrix()));
    calibration::TransformationMatrix::Builder matrix_builder(fbb);
    matrix_builder.add_data(data_offset);
    flatbuffers::Offset<calibration::TransformationMatrix>
        camera_to_pivot_offset = matrix_builder.Finish();

    flatbuffers::Offset<calibration::TransformationMatrix> pivot_to_imu_offset;
    if (calibration_parameters->has_pivot) {
      flatbuffers::Offset<flatbuffers::Vector<float>> data_offset =
          fbb.CreateVector(MatrixToVector(
              (Eigen::Translation3d(
                   calibration_parameters->pivot_to_imu_translation) *
               Eigen::Quaterniond(calibration_parameters->pivot_to_imu))
                  .matrix()));
      calibration::TransformationMatrix::Builder matrix_builder(fbb);
      matrix_builder.add_data(data_offset);
      pivot_to_imu_offset = matrix_builder.Finish();
    }

    calibration::CameraCalibration::Builder calibration_builder(fbb);
    if (calibration_parameters->has_pivot) {
      calibration_builder.add_fixed_extrinsics(pivot_to_imu_offset);
      calibration_builder.add_turret_extrinsics(camera_to_pivot_offset);
    } else {
      calibration_builder.add_fixed_extrinsics(camera_to_pivot_offset);
    }
    fbb.Finish(calibration_builder.Finish());
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> extrinsics =
        fbb.Release();
    return extrinsics;
  }
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

void Visualize(const CalibrationData &data,
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
  filter.Visualize(calibration_parameters);
}

}  // namespace vision
}  // namespace frc971
