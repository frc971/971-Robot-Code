#ifndef FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_

#include <assert.h>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <chrono>

#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/macros.h"

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename PlantType, typename ObserverType>
class StateFeedbackLoop;

// For everything in this file, "inputs" and "outputs" are defined from the
// perspective of the plant. This means U is an input and Y is an output
// (because you give the plant U (powers) and it gives you back a Y (sensor
// values). This is the opposite of what they mean from the perspective of the
// controller (U is an output because that's what goes to the motors and Y is an
// input because that's what comes back from the sensors).

template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct StateFeedbackPlantCoefficients final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlantCoefficients(const StateFeedbackPlantCoefficients &other)
      : A(other.A),
        A_inv(other.A_inv),
        B(other.B),
        C(other.C),
        D(other.D),
        U_min(other.U_min),
        U_max(other.U_max) {}

  StateFeedbackPlantCoefficients(
      const Eigen::Matrix<double, number_of_states, number_of_states> &A,
      const Eigen::Matrix<double, number_of_states, number_of_states> &A_inv,
      const Eigen::Matrix<double, number_of_states, number_of_inputs> &B,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_max,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_min)
      : A(A), A_inv(A_inv), B(B), C(C), D(D), U_min(U_min), U_max(U_max) {}

  const Eigen::Matrix<double, number_of_states, number_of_states> A;
  const Eigen::Matrix<double, number_of_states, number_of_states> A_inv;
  const Eigen::Matrix<double, number_of_states, number_of_inputs> B;
  const Eigen::Matrix<double, number_of_outputs, number_of_states> C;
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> D;
  const Eigen::Matrix<double, number_of_inputs, 1> U_min;
  const Eigen::Matrix<double, number_of_inputs, 1> U_max;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlant(
      ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>>
          *coefficients)
      : coefficients_(::std::move(*coefficients)), index_(0) {
    Reset();
  }

  StateFeedbackPlant(StateFeedbackPlant &&other)
      : index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
    X_.swap(other.X_);
    Y_.swap(other.Y_);
  }

  virtual ~StateFeedbackPlant() {}

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return coefficients().A;
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_states> &A_inv() const {
    return coefficients().A_inv;
  }
  double A_inv(int i, int j) const { return A_inv()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return coefficients().B;
  }
  double B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_states> &C() const {
    return coefficients().C;
  }
  double C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D() const {
    return coefficients().D;
  }
  double D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_min() const {
    return coefficients().U_min;
  }
  double U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return coefficients().U_max;
  }
  double U_max(int i, int j) const { return U_max()(i, j); }

  const Eigen::Matrix<double, number_of_states, 1> &X() const { return X_; }
  double X(int i, int j) const { return X()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, 1> &Y() const { return Y_; }
  double Y(int i, int j) const { return Y()(i, j); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X() { return X_; }
  double &mutable_X(int i, int j) { return mutable_X()(i, j); }
  Eigen::Matrix<double, number_of_outputs, 1> &mutable_Y() { return Y_; }
  double &mutable_Y(int i, int j) { return mutable_Y()(i, j); }

  const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                       number_of_outputs>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                       number_of_outputs>
      &coefficients() const {
    return *coefficients_[index_];
  }

  int index() const { return index_; }
  void set_index(int index) {
    assert(index >= 0);
    assert(index < static_cast<int>(coefficients_.size()));
    index_ = index;
  }

  void Reset() {
    X_.setZero();
    Y_.setZero();
  }

  // Assert that U is within the hardware range.
  virtual void CheckU(const Eigen::Matrix<double, number_of_inputs, 1> &U) {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > U_max(i, 0) + 0.00001 || U(i, 0) < U_min(i, 0) - 0.00001) {
        LOG(FATAL, "U out of range\n");
      }
    }
  }

  // Computes the new X and Y given the control input.
  void Update(const Eigen::Matrix<double, number_of_inputs, 1> &U) {
    // Powers outside of the range are more likely controller bugs than things
    // that the plant should deal with.
    CheckU(U);
    X_ = Update(X(), U);
    UpdateY(U);
  }

  // Computes the new Y given the control input.
  void UpdateY(const Eigen::Matrix<double, number_of_inputs, 1> &U) {
    Y_ = C() * X() + D() * U;
  }

  Eigen::Matrix<double, number_of_states, 1> Update(
      const Eigen::Matrix<double, number_of_states, 1> X,
      const Eigen::Matrix<double, number_of_inputs, 1> &U) const {
    return A() * X + B() * U;
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  Eigen::Matrix<double, number_of_states, 1> X_;
  Eigen::Matrix<double, number_of_outputs, 1> Y_;

  ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;

  int index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackPlant);
};

// A container for all the controller coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct StateFeedbackControllerCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_inputs, number_of_states> K;
  const Eigen::Matrix<double, number_of_inputs, number_of_states> Kff;

  StateFeedbackControllerCoefficients(
      const Eigen::Matrix<double, number_of_inputs, number_of_states> &K,
      const Eigen::Matrix<double, number_of_inputs, number_of_states> &Kff)
      : K(K), Kff(Kff) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct StateFeedbackHybridPlantCoefficients final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackHybridPlantCoefficients(
      const StateFeedbackHybridPlantCoefficients &other)
      : A_continuous(other.A_continuous),
        B_continuous(other.B_continuous),
        C(other.C),
        D(other.D),
        U_min(other.U_min),
        U_max(other.U_max) {}

  StateFeedbackHybridPlantCoefficients(
      const Eigen::Matrix<double, number_of_states, number_of_states>
          &A_continuous,
      const Eigen::Matrix<double, number_of_states, number_of_inputs>
          &B_continuous,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_max,
      const Eigen::Matrix<double, number_of_inputs, 1> &U_min)
      : A_continuous(A_continuous),
        B_continuous(B_continuous),
        C(C),
        D(D),
        U_min(U_min),
        U_max(U_max) {}

  const Eigen::Matrix<double, number_of_states, number_of_states> A_continuous;
  const Eigen::Matrix<double, number_of_states, number_of_inputs> B_continuous;
  const Eigen::Matrix<double, number_of_outputs, number_of_states> C;
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> D;
  const Eigen::Matrix<double, number_of_inputs, 1> U_min;
  const Eigen::Matrix<double, number_of_inputs, 1> U_max;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackHybridPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackHybridPlant(
      ::std::vector<::std::unique_ptr<StateFeedbackHybridPlantCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>>
          *coefficients)
      : coefficients_(::std::move(*coefficients)), index_(0) {
    Reset();
  }

  StateFeedbackHybridPlant(StateFeedbackHybridPlant &&other)
      : index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
    X_.swap(other.X_);
    Y_.swap(other.Y_);
  }

  virtual ~StateFeedbackHybridPlant() {}

  const Eigen::Matrix<double, number_of_states, number_of_states> &A() const {
    return A_;
  }
  double A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<double, number_of_states, number_of_inputs> &B() const {
    return B_;
  }
  double B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_states> &C() const {
    return coefficients().C;
  }
  double C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D() const {
    return coefficients().D;
  }
  double D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_min() const {
    return coefficients().U_min;
  }
  double U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_max() const {
    return coefficients().U_max;
  }
  double U_max(int i, int j) const { return U_max()(i, j); }

  const Eigen::Matrix<double, number_of_states, 1> &X() const { return X_; }
  double X(int i, int j) const { return X()(i, j); }
  const Eigen::Matrix<double, number_of_outputs, 1> &Y() const { return Y_; }
  double Y(int i, int j) const { return Y()(i, j); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X() { return X_; }
  double &mutable_X(int i, int j) { return mutable_X()(i, j); }
  Eigen::Matrix<double, number_of_outputs, 1> &mutable_Y() { return Y_; }
  double &mutable_Y(int i, int j) { return mutable_Y()(i, j); }

  const StateFeedbackHybridPlantCoefficients<number_of_states, number_of_inputs,
                                             number_of_outputs>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackHybridPlantCoefficients<number_of_states, number_of_inputs,
                                             number_of_outputs>
      &coefficients() const {
    return *coefficients_[index_];
  }

  int index() const { return index_; }
  void set_index(int index) {
    assert(index >= 0);
    assert(index < static_cast<int>(coefficients_.size()));
    index_ = index;
  }

  void Reset() {
    X_.setZero();
    Y_.setZero();
    A_.setZero();
    B_.setZero();
    DelayedU_.setZero();
    UpdateAB(::aos::controls::kLoopFrequency);
  }

  // Assert that U is within the hardware range.
  virtual void CheckU(const Eigen::Matrix<double, number_of_inputs, 1> &U) {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > U_max(i, 0) + 0.00001 || U(i, 0) < U_min(i, 0) - 0.00001) {
        LOG(FATAL, "U out of range\n");
      }
    }
  }

  // Computes the new X and Y given the control input.
  void Update(const Eigen::Matrix<double, number_of_inputs, 1> &U,
              ::std::chrono::nanoseconds dt) {
    // Powers outside of the range are more likely controller bugs than things
    // that the plant should deal with.
    CheckU(U);
    ::aos::robot_state.FetchLatest();

    Eigen::Matrix<double, number_of_inputs, 1> current_U =
        DelayedU_ * (::aos::robot_state.get()
                         ? ::aos::robot_state->voltage_battery / 12.0
                         : 1.0);
    X_ = Update(X(), current_U, dt);
    Y_ = C() * X() + D() * current_U;
    DelayedU_ = U;
  }

  Eigen::Matrix<double, number_of_inputs, 1> DelayedU_;

  Eigen::Matrix<double, number_of_states, 1> Update(
      const Eigen::Matrix<double, number_of_states, 1> X,
      const Eigen::Matrix<double, number_of_inputs, 1> &U,
      ::std::chrono::nanoseconds dt) {
    UpdateAB(dt);
    return A() * X + B() * U;
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  void UpdateAB(::std::chrono::nanoseconds dt) {
    Eigen::Matrix<double, number_of_states + number_of_inputs,
                  number_of_states + number_of_inputs>
        M_state_continuous;
    M_state_continuous.setZero();
    M_state_continuous.template block<number_of_states, number_of_states>(0,
                                                                          0) =
        coefficients().A_continuous *
        ::std::chrono::duration_cast<::std::chrono::duration<double>>(dt)
            .count();
    M_state_continuous.template block<number_of_states, number_of_inputs>(
        0, number_of_states) =
        coefficients().B_continuous *
        ::std::chrono::duration_cast<::std::chrono::duration<double>>(dt)
            .count();

    Eigen::Matrix<double, number_of_states + number_of_inputs,
                  number_of_states + number_of_inputs>
        M_state = M_state_continuous.exp();
    A_ = M_state.template block<number_of_states, number_of_states>(0, 0);
    B_ = M_state.template block<number_of_states, number_of_inputs>(
        0, number_of_states);
  }

  Eigen::Matrix<double, number_of_states, 1> X_;
  Eigen::Matrix<double, number_of_outputs, 1> Y_;

  Eigen::Matrix<double, number_of_states, number_of_states> A_;
  Eigen::Matrix<double, number_of_states, number_of_inputs> B_;


  ::std::vector<::std::unique_ptr<StateFeedbackHybridPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;

  int index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackHybridPlant);
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackController(
      ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>> *controllers)
      : coefficients_(::std::move(*controllers)) {}

  StateFeedbackController(StateFeedbackController &&other)
      : index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  const Eigen::Matrix<double, number_of_inputs, number_of_states> &K() const {
    return coefficients().K;
  }
  double K(int i, int j) const { return K()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, number_of_states> &Kff() const {
    return coefficients().Kff;
  }
  double Kff(int i, int j) const { return Kff()(i, j); }

  void Reset() {}

  // Sets the current controller to be index, clamped to be within range.
  void set_index(int index) {
    if (index < 0) {
      index_ = 0;
    } else if (index >= static_cast<int>(coefficients_.size())) {
      index_ = static_cast<int>(coefficients_.size()) - 1;
    } else {
      index_ = index;
    }
  }

  int index() const { return index_; }

  const StateFeedbackControllerCoefficients<number_of_states, number_of_inputs,
                                            number_of_outputs>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackControllerCoefficients<number_of_states, number_of_inputs,
                                            number_of_outputs>
      &coefficients() const {
    return *coefficients_[index_];
  }

 private:
  int index_ = 0;
  ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;
};

// A container for all the observer coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct StateFeedbackObserverCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_states, number_of_outputs> L;

  StateFeedbackObserverCoefficients(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L)
      : L(L) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackObserver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackObserver(
      ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>> *observers)
      : coefficients_(::std::move(*observers)) {}

  StateFeedbackObserver(StateFeedbackObserver &&other)
      : X_hat_(other.X_hat_), index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  const Eigen::Matrix<double, number_of_states, number_of_outputs> &L() const {
    return coefficients().L;
  }
  double L(int i, int j) const { return L()(i, j); }

  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  Eigen::Matrix<double, number_of_states, 1> &mutable_X_hat() { return X_hat_; }

  void Reset(
      StateFeedbackLoop<number_of_states, number_of_inputs, number_of_outputs,
                        StateFeedbackPlant<number_of_states, number_of_inputs,
                                           number_of_outputs>,
                        StateFeedbackObserver> * /*loop*/) {
    X_hat_.setZero();
  }

  void Predict(
      StateFeedbackLoop<number_of_states, number_of_inputs, number_of_outputs,
                        StateFeedbackPlant<number_of_states, number_of_inputs,
                                           number_of_outputs>,
                        StateFeedbackObserver> *loop,
      const Eigen::Matrix<double, number_of_inputs, 1> &new_u,
      ::std::chrono::nanoseconds /*dt*/) {
    mutable_X_hat() = loop->plant().Update(X_hat(), new_u);
  }

  void Correct(const StateFeedbackLoop<
                   number_of_states, number_of_inputs, number_of_outputs,
                   StateFeedbackPlant<number_of_states, number_of_inputs,
                                      number_of_outputs>,
                   StateFeedbackObserver> &loop,
               const Eigen::Matrix<double, number_of_inputs, 1> &U,
               const Eigen::Matrix<double, number_of_outputs, 1> &Y) {
    mutable_X_hat() += loop.plant().A_inv() * L() *
                       (Y - loop.plant().C() * X_hat() - loop.plant().D() * U);
  }

  // Sets the current controller to be index, clamped to be within range.
  void set_index(int index) {
    if (index < 0) {
      index_ = 0;
    } else if (index >= static_cast<int>(coefficients_.size())) {
      index_ = static_cast<int>(coefficients_.size()) - 1;
    } else {
      index_ = index;
    }
  }

  int index() const { return index_; }

  const StateFeedbackObserverCoefficients<number_of_states, number_of_inputs,
                                          number_of_outputs>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackObserverCoefficients<number_of_states, number_of_inputs,
                                          number_of_outputs>
      &coefficients() const {
    return *coefficients_[index_];
  }

 private:
  // Internal state estimate.
  Eigen::Matrix<double, number_of_states, 1> X_hat_;

  int index_ = 0;
  ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;
};

// A container for all the observer coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
struct HybridKalmanCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_states, number_of_states> Q_continuous;
  const Eigen::Matrix<double, number_of_outputs, number_of_outputs> R_continuous;
  const Eigen::Matrix<double, number_of_states, number_of_states> P_steady_state;

  HybridKalmanCoefficients(
      const Eigen::Matrix<double, number_of_states, number_of_states>
          &Q_continuous,
      const Eigen::Matrix<double, number_of_outputs, number_of_outputs>
          &R_continuous,
      const Eigen::Matrix<double, number_of_states, number_of_states>
          &P_steady_state)
      : Q_continuous(Q_continuous),
        R_continuous(R_continuous),
        P_steady_state(P_steady_state) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class HybridKalman {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit HybridKalman(
      ::std::vector<::std::unique_ptr<HybridKalmanCoefficients<
          number_of_states, number_of_inputs, number_of_outputs>>> *observers)
      : coefficients_(::std::move(*observers)) {}

  HybridKalman(HybridKalman &&other)
      : X_hat_(other.X_hat_), index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  // Getters for Q
  const Eigen::Matrix<double, number_of_states, number_of_states> &Q() const {
    return Q_;
  }
  double Q(int i, int j) const { return Q()(i, j); }
  // Getters for R
  const Eigen::Matrix<double, number_of_outputs, number_of_outputs> &R() const {
    return R_;
  }
  double R(int i, int j) const { return R()(i, j); }

  // Getters for P
  const Eigen::Matrix<double, number_of_states, number_of_states> &P() const {
    return P_;
  }
  double P(int i, int j) const { return P()(i, j); }

  // Getters for X_hat
  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  Eigen::Matrix<double, number_of_states, 1> &mutable_X_hat() { return X_hat_; }

  void Reset(StateFeedbackLoop<
             number_of_states, number_of_inputs, number_of_outputs,
             StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                      number_of_outputs>,
             HybridKalman> *loop) {
    X_hat_.setZero();
    P_ = coefficients().P_steady_state;
    UpdateQR(loop, ::aos::controls::kLoopFrequency);
  }

  void Predict(StateFeedbackLoop<
                   number_of_states, number_of_inputs, number_of_outputs,
                   StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                            number_of_outputs>,
                   HybridKalman> *loop,
               const Eigen::Matrix<double, number_of_inputs, 1> &new_u,
               ::std::chrono::nanoseconds dt) {
    // Trigger the predict step.  This will update A() and B() in the plant.
    mutable_X_hat() = loop->mutable_plant()->Update(X_hat(), new_u, dt);

    UpdateQR(loop, dt);
    P_ = loop->plant().A() * P_ * loop->plant().A().transpose() + Q_;
  }

  void Correct(const StateFeedbackLoop<
                   number_of_states, number_of_inputs, number_of_outputs,
                   StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                            number_of_outputs>,
                   HybridKalman> &loop,
               const Eigen::Matrix<double, number_of_inputs, 1> &U,
               const Eigen::Matrix<double, number_of_outputs, 1> &Y) {
    Eigen::Matrix<double, number_of_outputs, 1> Y_bar =
        Y - (loop.plant().C() * X_hat_ + loop.plant().D() * U);
    Eigen::Matrix<double, number_of_outputs, number_of_outputs> S =
        loop.plant().C() * P_ * loop.plant().C().transpose() + R_;
    Eigen::Matrix<double, number_of_states, number_of_outputs> KalmanGain;
    KalmanGain = (S.transpose().ldlt().solve(
                      (P() * loop.plant().C().transpose()).transpose()))
                     .transpose();
    X_hat_ = X_hat_ + KalmanGain * Y_bar;
    P_ = (loop.plant().coefficients().A_continuous.Identity() -
          KalmanGain * loop.plant().C()) *
         P();
  }

  // Sets the current controller to be index, clamped to be within range.
  void set_index(int index) {
    if (index < 0) {
      index_ = 0;
    } else if (index >= static_cast<int>(coefficients_.size())) {
      index_ = static_cast<int>(coefficients_.size()) - 1;
    } else {
      index_ = index;
    }
  }

  int index() const { return index_; }

  const HybridKalmanCoefficients<number_of_states, number_of_inputs,
                                 number_of_outputs>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const HybridKalmanCoefficients<number_of_states, number_of_inputs,
                                 number_of_outputs>
      &coefficients() const {
    return *coefficients_[index_];
  }

 private:
  void UpdateQR(StateFeedbackLoop<
                    number_of_states, number_of_inputs, number_of_outputs,
                    StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                             number_of_outputs>,
                    HybridKalman> *loop,
                ::std::chrono::nanoseconds dt) {
    // Now, compute the discrete time Q and R coefficients.
    Eigen::Matrix<double, number_of_states, number_of_states> Qtemp =
        (coefficients().Q_continuous +
         coefficients().Q_continuous.transpose()) /
        2.0;
    Eigen::Matrix<double, number_of_outputs, number_of_outputs> Rtemp =
        (coefficients().R_continuous +
         coefficients().R_continuous.transpose()) /
        2.0;

    Eigen::Matrix<double, 2 * number_of_states, 2 * number_of_states> M_gain;
    M_gain.setZero();
    // Set up the matrix M = [[-A, Q], [0, A.T]]
    M_gain.template block<number_of_states, number_of_states>(0, 0) =
        -loop->plant().coefficients().A_continuous;
    M_gain.template block<number_of_states, number_of_states>(
        0, number_of_states) = Qtemp;
    M_gain.template block<number_of_states, number_of_states>(
        number_of_states, number_of_states) =
        loop->plant().coefficients().A_continuous.transpose();

    Eigen::Matrix<double, 2 * number_of_states, 2 *number_of_states> phi =
        (M_gain *
         ::std::chrono::duration_cast<::std::chrono::duration<double>>(dt)
             .count())
            .exp();

    // Phi12 = phi[0:number_of_states, number_of_states:2*number_of_states]
    // Phi22 = phi[number_of_states:2*number_of_states,
    // number_of_states:2*number_of_states]
    Eigen::Matrix<double, number_of_states, number_of_states> phi12 =
        phi.block(0, number_of_states, number_of_states, number_of_states);
    Eigen::Matrix<double, number_of_states, number_of_states> phi22 = phi.block(
        number_of_states, number_of_states, number_of_states, number_of_states);

    Q_ = phi22.transpose() * phi12;
    Q_ = (Q_ + Q_.transpose()) / 2.0;
    R_ = Rtemp /
         ::std::chrono::duration_cast<::std::chrono::duration<double>>(dt)
             .count();
  }

  // Internal state estimate.
  Eigen::Matrix<double, number_of_states, 1> X_hat_;
  // Internal covariance estimate.
  Eigen::Matrix<double, number_of_states, number_of_states> P_;

  // Discretized Q and R for the kalman filter.
  Eigen::Matrix<double, number_of_states, number_of_states> Q_;
  Eigen::Matrix<double, number_of_outputs, number_of_outputs> R_;

  int index_ = 0;
  ::std::vector<::std::unique_ptr<HybridKalmanCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename PlantType = StateFeedbackPlant<
              number_of_states, number_of_inputs, number_of_outputs>,
          typename ObserverType = StateFeedbackObserver<
              number_of_states, number_of_inputs, number_of_outputs>>
class StateFeedbackLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackLoop(
      PlantType &&plant,
      StateFeedbackController<number_of_states, number_of_inputs,
                              number_of_outputs> &&controller,
      ObserverType &&observer)
      : plant_(::std::move(plant)),
        controller_(::std::move(controller)),
        observer_(::std::move(observer)) {
    Reset();
  }

  StateFeedbackLoop(StateFeedbackLoop &&other)
      : plant_(::std::move(other.plant_)),
        controller_(::std::move(other.controller_)),
        observer_(::std::move(other.observer_)) {
    R_.swap(other.R_);
    next_R_.swap(other.next_R_);
    U_.swap(other.U_);
    U_uncapped_.swap(other.U_uncapped_);
    ff_U_.swap(other.ff_U_);
  }

  virtual ~StateFeedbackLoop() {}

  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return observer().X_hat();
  }
  double X_hat(int i, int j) const { return X_hat()(i, j); }
  const Eigen::Matrix<double, number_of_states, 1> &R() const { return R_; }
  double R(int i, int j) const { return R()(i, j); }
  const Eigen::Matrix<double, number_of_states, 1> &next_R() const {
    return next_R_;
  }
  double next_R(int i, int j) const { return next_R()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U() const { return U_; }
  double U(int i, int j) const { return U()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &U_uncapped() const {
    return U_uncapped_;
  }
  double U_uncapped(int i, int j) const { return U_uncapped()(i, j); }
  const Eigen::Matrix<double, number_of_inputs, 1> &ff_U() const {
    return ff_U_;
  }
  double ff_U(int i, int j) const { return ff_U()(i, j); }

  Eigen::Matrix<double, number_of_states, 1> &mutable_X_hat() {
    return observer_.mutable_X_hat();
  }
  double &mutable_X_hat(int i, int j) { return mutable_X_hat()(i, j); }
  Eigen::Matrix<double, number_of_states, 1> &mutable_R() { return R_; }
  double &mutable_R(int i, int j) { return mutable_R()(i, j); }
  Eigen::Matrix<double, number_of_states, 1> &mutable_next_R() {
    return next_R_;
  }
  double &mutable_next_R(int i, int j) { return mutable_next_R()(i, j); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U() { return U_; }
  double &mutable_U(int i, int j) { return mutable_U()(i, j); }
  Eigen::Matrix<double, number_of_inputs, 1> &mutable_U_uncapped() {
    return U_uncapped_;
  }
  double &mutable_U_uncapped(int i, int j) {
    return mutable_U_uncapped()(i, j);
  }

  const PlantType &plant() const { return plant_; }
  PlantType *mutable_plant() { return &plant_; }

  const StateFeedbackController<number_of_states, number_of_inputs,
                                number_of_outputs>
      &controller() const {
    return controller_;
  }

  const ObserverType &observer() const { return observer_; }

  void Reset() {
    R_.setZero();
    next_R_.setZero();
    U_.setZero();
    U_uncapped_.setZero();
    ff_U_.setZero();

    plant_.Reset();
    controller_.Reset();
    observer_.Reset(this);
  }

  // If U is outside the hardware range, limit it before the plant tries to use
  // it.
  virtual void CapU() {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > plant().U_max(i, 0)) {
        U_(i, 0) = plant().U_max(i, 0);
      } else if (U(i, 0) < plant().U_min(i, 0)) {
        U_(i, 0) = plant().U_min(i, 0);
      }
    }
  }

  // Corrects X_hat given the observation in Y.
  void Correct(const Eigen::Matrix<double, number_of_outputs, 1> &Y) {
    observer_.Correct(*this, U(), Y);
  }

  const Eigen::Matrix<double, number_of_states, 1> error() const {
    return R() - X_hat();
  }

  // Returns the calculated controller power.
  virtual const Eigen::Matrix<double, number_of_inputs, 1> ControllerOutput() {
    // TODO(austin): Should this live in StateSpaceController?
    ff_U_ = FeedForward();
    return controller().K() * error() + ff_U_;
  }

  // Calculates the feed forwards power.
  virtual const Eigen::Matrix<double, number_of_inputs, 1> FeedForward() {
    // TODO(austin): Should this live in StateSpaceController?
    return controller().Kff() * (next_R() - plant().A() * R());
  }

  // stop_motors is whether or not to output all 0s.
  void Update(bool stop_motors,
              ::std::chrono::nanoseconds dt = ::std::chrono::milliseconds(5)) {
    if (stop_motors) {
      U_.setZero();
      U_uncapped_.setZero();
      ff_U_.setZero();
    } else {
      U_ = U_uncapped_ = ControllerOutput();
      CapU();
    }

    UpdateObserver(U_, dt);

    UpdateFFReference();
  }

  // Updates R() after any CapU operations happen on U().
  void UpdateFFReference() {
    ff_U_ -= U_uncapped() - U();
    if (!controller().Kff().isZero(0)) {
      R_ = plant().A() * R() + plant().B() * ff_U_;
    }
  }

  void UpdateObserver(const Eigen::Matrix<double, number_of_inputs, 1> &new_u,
                      ::std::chrono::nanoseconds dt) {
    observer_.Predict(this, new_u, dt);
  }

  // Sets the current controller to be index.
  void set_index(int index) {
    plant_.set_index(index);
    controller_.set_index(index);
    observer_.set_index(index);
  }

  int index() const { return plant_.index(); }

 protected:
  PlantType plant_;

  StateFeedbackController<number_of_states, number_of_inputs, number_of_outputs>
      controller_;

  ObserverType observer_;

  // These are accessible from non-templated subclasses.
  static constexpr int kNumStates = number_of_states;
  static constexpr int kNumOutputs = number_of_outputs;
  static constexpr int kNumInputs = number_of_inputs;

  // Portion of U which is based on the feed-forwards.
  Eigen::Matrix<double, number_of_inputs, 1> ff_U_;

 private:
  // Current goal (Used by the feed-back controller).
  Eigen::Matrix<double, number_of_states, 1> R_;
  // Goal to go to in the next cycle (Used by Feed-Forward controller.)
  Eigen::Matrix<double, number_of_states, 1> next_R_;
  // Computed output after being capped.
  Eigen::Matrix<double, number_of_inputs, 1> U_;
  // Computed output before being capped.
  Eigen::Matrix<double, number_of_inputs, 1> U_uncapped_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackLoop);
};

#endif  // FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
