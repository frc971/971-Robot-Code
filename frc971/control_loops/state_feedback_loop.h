#ifndef FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_

#include <assert.h>

#include <vector>
#include <iostream>

#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/macros.h"

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
      const Eigen::Matrix<double, number_of_outputs, 1> &U_max,
      const Eigen::Matrix<double, number_of_outputs, 1> &U_min)
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
  double U_min(int i) const { return U_min()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return U_max_;
  }
  double U_max(int i) const { return U_max()(i, 0); }

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
      const ::std::vector<StateFeedbackPlantCoefficients<
          number_of_states, number_of_inputs,
          number_of_outputs> *> &coefficients)
      : coefficients_(coefficients),
        plant_index_(0) {
    Reset();
  }

  StateFeedbackPlant(StateFeedbackPlant &&other)
      : plant_index_(other.plant_index_) {
    ::std::swap(coefficients_, other.coefficients_);
    X_.swap(other.X_);
    Y_.swap(other.Y_);
    U_.swap(other.U_);
  }

  virtual ~StateFeedbackPlant() {
    for (auto c : coefficients_) {
      delete c;
    }
  }

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
  double U_min(int i) const { return U_min()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return coefficients().U_max();
  }
  double U_max(int i) const { return U_max()(i, 0); }

  const Eigen::Matrix<double, number_of_states, 1> &X() const { return X_; }
  double X(int i) const { return X()(i, 0); }
  const Eigen::Matrix<double, number_of_outputs, 1> &Y() const { return Y_; }
  double Y(int i) const { return Y()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U() const { return U_; }
  double U(int i) const { return X()(i, 0); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X() { return X_; }
  double &mutable_X(int i) { return mutable_X()(i, 0); }
  Eigen::Matrix<double, number_of_outputs, 1> &mutable_Y() { return Y_; }
  double &mutable_Y(int i) { return mutable_Y()(i, 0); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U() { return U_; }
  double &mutable_U(int i) { return mutable_U()(i, 0); }

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
    for (int i = 0; i < kNumOutputs; ++i) {
      assert(U(i) <= U_max(i) + 0.00001);
      assert(U(i) >= U_min(i) - 0.00001);
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

  ::std::vector<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs> *> coefficients_;

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
  const Eigen::Matrix<double, number_of_outputs, number_of_states> K;
  StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                 number_of_outputs> plant;

  StateFeedbackController(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &K,
      const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                           number_of_outputs> &plant)
      : L(L),
        K(K),
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
    controllers_.push_back(new StateFeedbackController<
        number_of_states, number_of_inputs, number_of_outputs>(controller));
    Reset();
  }

  StateFeedbackLoop(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &K,
      const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                               number_of_outputs> &plant)
      : controller_index_(0) {
    controllers_.push_back(
        new StateFeedbackController<number_of_states, number_of_inputs,
                                    number_of_outputs>(L, K, plant));

    Reset();
  }

  StateFeedbackLoop(const ::std::vector<StateFeedbackController<
      number_of_states, number_of_inputs, number_of_outputs> *> &controllers)
      : controllers_(controllers), controller_index_(0) {
    Reset();
  }

  StateFeedbackLoop(StateFeedbackLoop &&other) {
    X_hat_.swap(other.X_hat_);
    R_.swap(other.R_);
    U_.swap(other.U_);
    U_uncapped_.swap(other.U_uncapped_);
    ::std::swap(controllers_, other.controllers_);
    Y_.swap(other.Y_);
    new_y_ = other.new_y_;
    controller_index_ = other.controller_index_;
  }

  virtual ~StateFeedbackLoop() {
    for (auto c : controllers_) {
      delete c;
    }
  }

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return controller().plant.A();
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return controller().plant.B();
  }
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
  double U_min(int i) const { return U_min()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return controller().plant.U_max();
  }
  double U_max(int i) const { return U_max()(i, 0); }

  const Eigen::Matrix<double, number_of_outputs, number_of_states> &K() const {
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
  double X_hat(int i) const { return X_hat()(i, 0); }
  const Eigen::Matrix<double, number_of_states, 1> &R() const { return R_; }
  double R(int i) const { return R()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U() const { return U_; }
  double U(int i) const { return U()(i, 0); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_uncapped() const {
    return U_uncapped_;
  }
  double U_uncapped(int i) const { return U_uncapped()(i, 0); }
  const Eigen::Matrix<double, number_of_outputs, 1> &Y() const { return Y_; }
  double Y(int i) const { return Y()(i, 0); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X_hat() { return X_hat_; }
  double &mutable_X_hat(int i) { return mutable_X_hat()(i, 0); }
  Eigen::Matrix<double, number_of_states, 1> &mutable_R() { return R_; }
  double &mutable_R(int i) { return mutable_R()(i, 0); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U() { return U_; }
  double &mutable_U(int i) { return mutable_U()(i, 0); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U_uncapped() {
    return U_uncapped_;
  }
  double &mutable_U_uncapped(int i) { return mutable_U_uncapped()(i, 0); }
  Eigen::Matrix<double, number_of_outputs, 1> &mutable_Y() { return Y_; }
  double &mutable_Y(int i) { return mutable_Y()(i, 0); }

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
    for (int i = 0; i < kNumOutputs; ++i) {
      if (U(i) > U_max(i)) {
        U_(i, 0) = U_max(i);
      } else if (U(i) < U_min(i)) {
        U_(i, 0) = U_min(i);
      }
    }
  }

  // Corrects X_hat given the observation in Y.
  void Correct(const Eigen::Matrix<double, number_of_outputs, 1> &Y) {
  /*
    auto eye =
        Eigen::Matrix<double, number_of_states, number_of_states>::Identity();
    //LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
    ::std::cout << "Identity " << eye << ::std::endl;
    ::std::cout << "X_hat " << X_hat << ::std::endl;
    ::std::cout << "LC " << L() * C() << ::std::endl;
    ::std::cout << "L " << L() << ::std::endl;
    ::std::cout << "C " << C() << ::std::endl;
    ::std::cout << "y " << Y << ::std::endl;
    ::std::cout << "z " << (Y - C() * X_hat) << ::std::endl;
    ::std::cout << "correction " << L() * (Y - C() * X_hat) << ::std::endl;
    X_hat = (eye - L() * C()) * X_hat + L() * Y;
    ::std::cout << "X_hat after " << X_hat << ::std::endl;
    ::std::cout << ::std::endl;
    */
    //LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
    Y_ = Y;
    new_y_ = true;
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

    UpdateObserver();
  }

  void UpdateObserver() {
    if (new_y_) {
      X_hat_ = (A() - L() * C()) * X_hat() + L() * Y() + B() * U();
      new_y_ = false;
    } else {
      X_hat_ = A() * X_hat() + B() * U();
    }
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
  ::std::vector<StateFeedbackController<number_of_states, number_of_inputs,
                                        number_of_outputs> *> controllers_;

  // These are accessible from non-templated subclasses.
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  Eigen::Matrix<double, number_of_states, 1> X_hat_;
  Eigen::Matrix<double, number_of_states, 1> R_;
  Eigen::Matrix<double, number_of_inputs, 1> U_;
  Eigen::Matrix<double, number_of_inputs, 1> U_uncapped_;

  // Temporary storage for a measurement until I can figure out why I can't
  // correct when the measurement is made.
  Eigen::Matrix<double, number_of_outputs, 1> Y_;
  bool new_y_ = false;

  int controller_index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackLoop);
};

#endif  // FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
