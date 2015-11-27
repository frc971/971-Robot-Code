#ifndef FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_

#include <assert.h>

#include <vector>
#include <memory>
#include <iostream>

#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/macros.h"

// For everything in this file, "inputs" and "outputs" are defined from the
// perspective of the plant. This means U is an input and Y is an output
// (because you give the plant U (powers) and it gives you back a Y (sensor
// values). This is the opposite of what they mean from the perspective of the
// controller (U is an output because that's what goes to the motors and Y is an
// input because that's what comes back from the sensors).

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackPlantCoefficients final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlantCoefficients(const StateFeedbackPlantCoefficients &other)
      : A_(other.A()),
        B_(other.B()),
        C_(other.C()),
        D_(other.D()),
        U_min_(other.U_min()),
        U_max_(other.U_max()) {
  }

  StateFeedbackPlantCoefficients(
      const Eigen::Matrix<double, number_of_states, number_of_states> &A,
      const Eigen::Matrix<double, number_of_states, number_of_inputs> &B,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_max,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_min)
      : A_(A),
        B_(B),
        C_(C),
        D_(D),
        U_min_(U_min),
        U_max_(U_max) {
  }

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return A_;
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return B_;
  }
  double B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_states> &C() const {
    return C_;
  }
  double C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D() const {
    return D_;
  }
  double D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_min() const {
    return U_min_;
  }
  double U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return U_max_;
  }
  double U_max(int i, int j) const { return U_max()(i, j); }

 private:
  const Eigen::Matrix<double, number_of_states, number_of_states> A_;
  const Eigen::Matrix<double, number_of_states, number_of_inputs> B_;
  const Eigen::Matrix<double, number_of_outputs, number_of_states> C_;
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> D_;
  const Eigen::Matrix<double, number_of_inputs, 1> U_min_;
  const Eigen::Matrix<double, number_of_inputs, 1> U_max_;

  StateFeedbackPlantCoefficients &operator=(
      StateFeedbackPlantCoefficients other) = delete;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlant(
      ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>> *
          coefficients)
      : coefficients_(::std::move(*coefficients)), plant_index_(0) {
    Reset();
  }

  StateFeedbackPlant(StateFeedbackPlant &&other)
      : plant_index_(other.plant_index_) {
    ::std::swap(coefficients_, other.coefficients_);
    X_.swap(other.X_);
    Y_.swap(other.Y_);
    U_.swap(other.U_);
  }

  virtual ~StateFeedbackPlant() {}

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return coefficients().A();
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return coefficients().B();
  }
  double B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_states> &C() const {
    return coefficients().C();
  }
  double C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D() const {
    return coefficients().D();
  }
  double D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_min() const {
    return coefficients().U_min();
  }
  double U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return coefficients().U_max();
  }
  double U_max(int i, int j) const { return U_max()(i, j); }

  const Eigen::Matrix<double, number_of_states, 1> &X() const { return X_; }
  double X(int i, int j) const { return X()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, 1> &Y() const { return Y_; }
  double Y(int i, int j) const { return Y()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U() const { return U_; }
  double U(int i, int j) const { return U()(i, j); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X() { return X_; }
  double &mutable_X(int i, int j) { return mutable_X()(i, j); }
  Eigen::Matrix<double, number_of_outputs, 1> &mutable_Y() { return Y_; }
  double &mutable_Y(int i, int j) { return mutable_Y()(i, j); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U() { return U_; }
  double &mutable_U(int i, int j) { return mutable_U()(i, j); }

  const StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>
          &coefficients() const {
    return *coefficients_[plant_index_];
  }

  int plant_index() const { return plant_index_; }
  void set_plant_index(int plant_index) {
    if (plant_index < 0) {
      plant_index_ = 0;
    } else if (plant_index >= static_cast<int>(coefficients_.size())) {
      plant_index_ = static_cast<int>(coefficients_.size()) - 1;
    } else {
      plant_index_ = plant_index;
    }
  }

  void Reset() {
    X_.setZero();
    Y_.setZero();
    U_.setZero();
  }

  // Assert that U is within the hardware range.
  virtual void CheckU() {
    for (int i = 0; i < kNumInputs; ++i) {
      assert(U(i, 0) <= U_max(i, 0) + 0.00001);
      assert(U(i, 0) >= U_min(i, 0) - 0.00001);
    }
  }

  // Computes the new X and Y given the control input.
  void Update() {
    // Powers outside of the range are more likely controller bugs than things
    // that the plant should deal with.
    CheckU();
    X_ = A() * X() + B() * U();
    Y_ = C() * X() + D() * U();
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  Eigen::Matrix<double, number_of_states, 1> X_;
  Eigen::Matrix<double, number_of_outputs, 1> Y_;
  Eigen::Matrix<double, number_of_inputs, 1> U_;

  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>> coefficients_;

  int plant_index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackPlant);
};

// A Controller is a structure which holds a plant and the K and L matrices.
// This is designed such that multiple controllers can share one set of state to
// support gain scheduling easily.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct StateFeedbackController final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_states, number_of_outputs> L;
  const Eigen::Matrix<double, number_of_inputs, number_of_states> K;
  const Eigen::Matrix<double, number_of_states, number_of_states> A_inv;
  StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                 number_of_outputs> plant;

  StateFeedbackController(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L,
      const Eigen::Matrix<double, number_of_inputs, number_of_states> &K,
      const Eigen::Matrix<double, number_of_states, number_of_states> &A_inv,
      const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                           number_of_outputs> &plant)
      : L(L),
        K(K),
        A_inv(A_inv),
        plant(plant) {
  }
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackLoop(const StateFeedbackController<
      number_of_states, number_of_inputs, number_of_outputs> &controller)
      : controller_index_(0) {
    controllers_.emplace_back(new StateFeedbackController<
        number_of_states, number_of_inputs, number_of_outputs>(controller));
    Reset();
  }

  StateFeedbackLoop(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L,
      const Eigen::Matrix<double, number_of_inputs, number_of_states> &K,
      const Eigen::Matrix<double, number_of_states, number_of_states> &A_inv,
      const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                               number_of_outputs> &plant)
      : controller_index_(0) {
    controllers_.emplace_back(
        new StateFeedbackController<number_of_states, number_of_inputs,
                                    number_of_outputs>(L, K, A_inv, plant));

    Reset();
  }

  StateFeedbackLoop(::std::vector< ::std::unique_ptr<StateFeedbackController<
      number_of_states, number_of_inputs, number_of_outputs>>> *controllers)
      : controllers_(::std::move(*controllers)), controller_index_(0) {
    Reset();
  }

  StateFeedbackLoop(StateFeedbackLoop &&other) {
    X_hat_.swap(other.X_hat_);
    R_.swap(other.R_);
    U_.swap(other.U_);
    U_uncapped_.swap(other.U_uncapped_);
    ::std::swap(controllers_, other.controllers_);
    controller_index_ = other.controller_index_;
  }

  virtual ~StateFeedbackLoop() {}

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return controller().plant.A();
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return controller().plant.B();
  }
  const Eigen::Matrix<double, number_of_states, number_of_states> &A_inv() const {
    return controller().A_inv;
  }
  double A_inv(int i, int j) const { return A_inv()(i, j); }
  double B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_states> &C() const {
    return controller().plant.C();
  }
  double C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D() const {
    return controller().plant.D();
  }
  double D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_min() const {
    return controller().plant.U_min();
  }
  double U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return controller().plant.U_max();
  }
  double U_max(int i, int j) const { return U_max()(i, j); }

  const Eigen::Matrix<double, number_of_inputs, number_of_states> &K() const {
    return controller().K;
  }
  double K(int i, int j) const { return K()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_outputs> &L() const {
    return controller().L;
  }
  double L(int i, int j) const { return L()(i, j); }

  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  double X_hat(int i, int j) const { return X_hat()(i, j); }
  const Eigen::Matrix<double, number_of_states, 1> &R() const { return R_; }
  double R(int i, int j) const { return R()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U() const { return U_; }
  double U(int i, int j) const { return U()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_uncapped() const {
    return U_uncapped_;
  }
  double U_uncapped(int i, int j) const { return U_uncapped()(i, j); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X_hat() { return X_hat_; }
  double &mutable_X_hat(int i, int j) { return mutable_X_hat()(i, j); }
  Eigen::Matrix<double, number_of_states, 1> &mutable_R() { return R_; }
  double &mutable_R(int i, int j) { return mutable_R()(i, j); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U() { return U_; }
  double &mutable_U(int i, int j) { return mutable_U()(i, j); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U_uncapped() {
    return U_uncapped_;
  }
  double &mutable_U_uncapped(int i, int j) {
    return mutable_U_uncapped()(i, j);
  }

  const StateFeedbackController<number_of_states, number_of_inputs,
                                number_of_outputs> &controller() const {
    return *controllers_[controller_index_];
  }

  const StateFeedbackController<number_of_states, number_of_inputs,
                                number_of_outputs> &controller(
                                    int index) const {
    return *controllers_[index];
  }

  void Reset() {
    X_hat_.setZero();
    R_.setZero();
    U_.setZero();
    U_uncapped_.setZero();
  }

  // If U is outside the hardware range, limit it before the plant tries to use
  // it.
  virtual void CapU() {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > U_max(i, 0)) {
        U_(i, 0) = U_max(i, 0);
      } else if (U(i, 0) < U_min(i, 0)) {
        U_(i, 0) = U_min(i, 0);
      }
    }
  }

  // Corrects X_hat given the observation in Y.
  void Correct(const Eigen::Matrix<double, number_of_outputs, 1> &Y) {
    X_hat_ += A_inv() * L() * (Y - C() * X_hat_ - D() * U());
  }

  // stop_motors is whether or not to output all 0s.
  void Update(bool stop_motors) {
    if (stop_motors) {
      U_.setZero();
      U_uncapped_.setZero();
    } else {
      U_ = U_uncapped_ = K() * (R() - X_hat());
      CapU();
    }

    UpdateObserver(U_);
  }

  void UpdateObserver(const Eigen::Matrix<double, number_of_inputs, 1> &new_u) {
    X_hat_ = A() * X_hat() + B() * new_u;
  }

  // Sets the current controller to be index, clamped to be within range.
  void set_controller_index(int index) {
    if (index < 0) {
      controller_index_ = 0;
    } else if (index >= static_cast<int>(controllers_.size())) {
      controller_index_ = static_cast<int>(controllers_.size()) - 1;
    } else {
      controller_index_ = index;
    }
  }

  int controller_index() const { return controller_index_; }

 protected:
  ::std::vector< ::std::unique_ptr<StateFeedbackController<
      number_of_states, number_of_inputs, number_of_outputs>>> controllers_;

  // These are accessible from non-templated subclasses.
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  Eigen::Matrix<double, number_of_states, 1> X_hat_;
  Eigen::Matrix<double, number_of_states, 1> R_;
  Eigen::Matrix<double, number_of_inputs, 1> U_;
  Eigen::Matrix<double, number_of_inputs, 1> U_uncapped_;

  int controller_index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackLoop);
};

#endif  // FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
