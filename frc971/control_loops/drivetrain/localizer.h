#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"
#include "frc971/control_loops/pose.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// An interface for target selection. This provides an object that will take in
// state updates and then determine what poes we should be driving to.
class TargetSelectorInterface {
 public:
  // Take the state as [x, y, theta, left_vel, right_vel]
  // If unable to determine what target to go for, returns false. If a viable
  // target is selected, then returns true and sets target_pose.
  // command_speed is the goal speed of the current drivetrain, generally
  // generated from the throttle and meant to signify driver intent.
  // TODO(james): Some implementations may also want a drivetrain goal so that
  // driver intent can be divined more directly.
  virtual bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                               double command_speed) = 0;
  // Gets the current target pose. Should only be called if UpdateSelection has
  // returned true.
  virtual TypedPose<double> TargetPose() const = 0;
  // The "radius" of the target--for y2019, we wanted to drive in so that a disc
  // with radius r would hit the plane of the target at an offset of exactly r
  // from the TargetPose--this is distinct from wanting the center of the
  // robot to project straight onto the center of the target.
  virtual double TargetRadius() const = 0;
};

// Defines an interface for classes that provide field-global localization.
class LocalizerInterface {
 public:
  // Perform a single step of the filter, using the information that is
  // available on every drivetrain iteration.
  // The user should pass in the U that the real system experienced from the
  // previous timestep until now; internally, any filters will first perform a
  // prediction step to get the estimate at time now, and then will apply
  // corrections based on the encoder/gyro/accelerometer values from time now.
  // TODO(james): Consider letting implementations subscribe to the sensor
  // values themselves, and then only passing in U. This requires more
  // coordination on timing, however.
  virtual void Update(const ::Eigen::Matrix<double, 2, 1> &U,
                      ::aos::monotonic_clock::time_point now,
                      double left_encoder, double right_encoder,
                      double gyro_rate, double longitudinal_accelerometer) = 0;
  // Reset the absolute position of the estimator.
  virtual void ResetPosition(::aos::monotonic_clock::time_point t, double x,
                             double y, double theta, double theta_uncertainty,
                             bool reset_theta) = 0;
  // There are several subtly different norms floating around for state
  // matrices. In order to avoid that mess, we jus tprovide direct accessors for
  // the values that most people care about.
  virtual double x() const = 0;
  virtual double y() const = 0;
  virtual double theta() const = 0;
  virtual double left_velocity() const = 0;
  virtual double right_velocity() const = 0;
  virtual double left_encoder() const = 0;
  virtual double right_encoder() const = 0;
  virtual double left_voltage_error() const = 0;
  virtual double right_voltage_error() const = 0;
  virtual TargetSelectorInterface *target_selector() = 0;
};

// A target selector, primarily for testing purposes, that just lets a user
// manually set the target selector state.
class TrivialTargetSelector : public TargetSelectorInterface {
 public:
  bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &, double) override {
    return has_target_;
  }
  TypedPose<double> TargetPose() const override { return pose_; }
  double TargetRadius() const override { return target_radius_; }

  void set_pose(const TypedPose<double> &pose) { pose_ = pose; }
  void set_target_radius(double radius) { target_radius_ = radius; }
  void set_has_target(bool has_target) { has_target_ = has_target; }
  bool has_target() const { return has_target_; }

 private:
  bool has_target_ = true;
  TypedPose<double> pose_;
  double target_radius_ = 0.0;
};

// Uses the generic HybridEkf implementation to provide a basic field estimator.
// This provides no method for using cameras or the such to get global
// measurements and just assumes that you can dead-reckon perfectly.
class DeadReckonEkf : public LocalizerInterface {
  typedef HybridEkf<double> Ekf;
  typedef typename Ekf::StateIdx StateIdx;

 public:
  DeadReckonEkf(::aos::EventLoop *event_loop,
                const DrivetrainConfig<double> &dt_config)
      : ekf_(dt_config) {
    event_loop->OnRun([this, event_loop]() {
      ekf_.ResetInitialState(event_loop->monotonic_now(), Ekf::State::Zero(),
                             ekf_.P());
    });
    target_selector_.set_has_target(false);
  }

  void Update(const ::Eigen::Matrix<double, 2, 1> &U,
              ::aos::monotonic_clock::time_point now, double left_encoder,
              double right_encoder, double gyro_rate,
              double /*longitudinal_accelerometer*/) override {
    ekf_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, now);
  }

  void ResetPosition(::aos::monotonic_clock::time_point t, double x, double y,
                     double theta, double /*theta_override*/,
                     bool /*reset_theta*/) override {
    const double left_encoder = ekf_.X_hat(StateIdx::kLeftEncoder);
    const double right_encoder = ekf_.X_hat(StateIdx::kRightEncoder);
    ekf_.ResetInitialState(t, (Ekf::State() << x, y, theta, left_encoder, 0,
                               right_encoder, 0, 0, 0,
                               0).finished(),
                           ekf_.P());
  };

  double x() const override { return ekf_.X_hat(StateIdx::kX); }
  double y() const override { return ekf_.X_hat(StateIdx::kY); }
  double theta() const override { return ekf_.X_hat(StateIdx::kTheta); }
  double left_encoder() const override {
    return ekf_.X_hat(StateIdx::kLeftEncoder);
  }
  double right_encoder() const override {
    return ekf_.X_hat(StateIdx::kRightEncoder);
  }
  double left_velocity() const override {
    return ekf_.X_hat(StateIdx::kLeftVelocity);
  }
  double right_velocity() const override {
    return ekf_.X_hat(StateIdx::kRightVelocity);
  }
  double left_voltage_error() const override {
    return ekf_.X_hat(StateIdx::kLeftVoltageError);
  }
  double right_voltage_error() const override {
    return ekf_.X_hat(StateIdx::kRightVoltageError);
  }

  TrivialTargetSelector *target_selector() override {
    return &target_selector_;
  }

 private:
  Ekf ekf_;
  TrivialTargetSelector target_selector_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
