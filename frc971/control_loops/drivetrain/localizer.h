#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_

#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

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
  // There are several subtly different norms floating around for state
  // matrices. In order to avoid that mess, we jus tprovide direct accessors for
  // the values that most people care about.
  virtual double x() const = 0;
  virtual double y() const = 0;
  virtual double theta() const = 0;
  virtual double left_velocity() const = 0;
  virtual double right_velocity() const = 0;
  virtual double left_voltage_error() const = 0;
  virtual double right_voltage_error() const = 0;
};

// Uses the generic HybridEkf implementation to provide a basic field estimator.
// This provides no method for using cameras or the such to get global
// measurements and just assumes that you can dead-reckon perfectly.
class DeadReckonEkf : public LocalizerInterface {
  typedef HybridEkf<double> Ekf;
  typedef typename Ekf::StateIdx StateIdx;
 public:
  DeadReckonEkf(const DrivetrainConfig<double> &dt_config) : ekf_(dt_config) {
    ekf_.ResetInitialState(::aos::monotonic_clock::now(), Ekf::State::Zero(),
                           ekf_.P());
  }

  void Update(const ::Eigen::Matrix<double, 2, 1> &U,
                      ::aos::monotonic_clock::time_point now,
                      double left_encoder, double right_encoder,
                      double gyro_rate,
                      double /*longitudinal_accelerometer*/) override {
    ekf_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, now);
  }

  double x() const override { return ekf_.X_hat(StateIdx::kX); }
  double y() const override { return ekf_.X_hat(StateIdx::kY); }
  double theta() const override { return ekf_.X_hat(StateIdx::kTheta); }
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

 private:
  Ekf ekf_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
