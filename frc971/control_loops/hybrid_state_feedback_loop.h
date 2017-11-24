#ifndef FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_H_

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
#include "frc971/control_loops/state_feedback_loop.h"

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
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
      const Eigen::Matrix<Scalar, number_of_states, number_of_states>
          &A_continuous,
      const Eigen::Matrix<Scalar, number_of_states, number_of_inputs>
          &B_continuous,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_max,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_min)
      : A_continuous(A_continuous),
        B_continuous(B_continuous),
        C(C),
        D(D),
        U_min(U_min),
        U_max(U_max) {}

  const Eigen::Matrix<Scalar, number_of_states, number_of_states> A_continuous;
  const Eigen::Matrix<Scalar, number_of_states, number_of_inputs> B_continuous;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_states> C;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_inputs> D;
  const Eigen::Matrix<Scalar, number_of_inputs, 1> U_min;
  const Eigen::Matrix<Scalar, number_of_inputs, 1> U_max;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
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

  const Eigen::Matrix<Scalar, number_of_states, number_of_states> &A() const {
    return A_;
  }
  Scalar A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<Scalar, number_of_states, number_of_inputs> &B() const {
    return B_;
  }
  Scalar B(int i, int j) const { return B()(i, j); }
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_states> &C() const {
    return coefficients().C;
  }
  Scalar C(int i, int j) const { return C()(i, j); }
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_inputs> &D() const {
    return coefficients().D;
  }
  Scalar D(int i, int j) const { return D()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_min() const {
    return coefficients().U_min;
  }
  Scalar U_min(int i, int j) const { return U_min()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_max() const {
    return coefficients().U_max;
  }
  Scalar U_max(int i, int j) const { return U_max()(i, j); }

  const Eigen::Matrix<Scalar, number_of_states, 1> &X() const { return X_; }
  Scalar X(int i, int j) const { return X()(i, j); }
  const Eigen::Matrix<Scalar, number_of_outputs, 1> &Y() const { return Y_; }
  Scalar Y(int i, int j) const { return Y()(i, j); }

  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_X() { return X_; }
  Scalar &mutable_X(int i, int j) { return mutable_X()(i, j); }
  Eigen::Matrix<Scalar, number_of_outputs, 1> &mutable_Y() { return Y_; }
  Scalar &mutable_Y(int i, int j) { return mutable_Y()(i, j); }

  const StateFeedbackHybridPlantCoefficients<number_of_states, number_of_inputs,
                                             number_of_outputs, Scalar>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackHybridPlantCoefficients<number_of_states, number_of_inputs,
                                             number_of_outputs, Scalar>
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
    UpdateAB(::std::chrono::milliseconds(5));
  }

  // Assert that U is within the hardware range.
  virtual void CheckU(const Eigen::Matrix<Scalar, number_of_inputs, 1> &U) {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > U_max(i, 0) + static_cast<Scalar>(0.00001) ||
          U(i, 0) < U_min(i, 0) - static_cast<Scalar>(0.00001)) {
        LOG(FATAL, "U out of range\n");
      }
    }
  }

  // Computes the new X and Y given the control input.
  void Update(const Eigen::Matrix<Scalar, number_of_inputs, 1> &U,
              ::std::chrono::nanoseconds dt) {
    // Powers outside of the range are more likely controller bugs than things
    // that the plant should deal with.
    CheckU(U);
    ::aos::robot_state.FetchLatest();

    Eigen::Matrix<Scalar, number_of_inputs, 1> current_U =
        DelayedU_ *
        (::aos::robot_state.get()
             ? ::aos::robot_state->voltage_battery / static_cast<Scalar>(12.0)
             : static_cast<Scalar>(1.0));
    X_ = Update(X(), current_U, dt);
    Y_ = C() * X() + D() * current_U;
    DelayedU_ = U;
  }

  Eigen::Matrix<Scalar, number_of_inputs, 1> DelayedU_;

  Eigen::Matrix<Scalar, number_of_states, 1> Update(
      const Eigen::Matrix<Scalar, number_of_states, 1> X,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U,
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
    Eigen::Matrix<Scalar, number_of_states + number_of_inputs,
                  number_of_states + number_of_inputs>
        M_state_continuous;
    M_state_continuous.setZero();
    M_state_continuous.template block<number_of_states, number_of_states>(0,
                                                                          0) =
        coefficients().A_continuous *
        ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt)
            .count();
    M_state_continuous.template block<number_of_states, number_of_inputs>(
        0, number_of_states) =
        coefficients().B_continuous *
        ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt)
            .count();

    Eigen::Matrix<Scalar, number_of_states + number_of_inputs,
                  number_of_states + number_of_inputs>
        M_state = M_state_continuous.exp();
    A_ = M_state.template block<number_of_states, number_of_states>(0, 0);
    B_ = M_state.template block<number_of_states, number_of_inputs>(
        0, number_of_states);
  }

  Eigen::Matrix<Scalar, number_of_states, 1> X_;
  Eigen::Matrix<Scalar, number_of_outputs, 1> Y_;

  Eigen::Matrix<Scalar, number_of_states, number_of_states> A_;
  Eigen::Matrix<Scalar, number_of_states, number_of_inputs> B_;


  ::std::vector<::std::unique_ptr<StateFeedbackHybridPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;

  int index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackHybridPlant);
};


// A container for all the observer coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
struct HybridKalmanCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<Scalar, number_of_states, number_of_states> Q_continuous;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> R_continuous;
  const Eigen::Matrix<Scalar, number_of_states, number_of_states> P_steady_state;

  HybridKalmanCoefficients(
      const Eigen::Matrix<Scalar, number_of_states, number_of_states>
          &Q_continuous,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs>
          &R_continuous,
      const Eigen::Matrix<Scalar, number_of_states, number_of_states>
          &P_steady_state)
      : Q_continuous(Q_continuous),
        R_continuous(R_continuous),
        P_steady_state(P_steady_state) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
class HybridKalman {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit HybridKalman(
      ::std::vector<::std::unique_ptr<HybridKalmanCoefficients<
          number_of_states, number_of_inputs, number_of_outputs, Scalar>>> *observers)
      : coefficients_(::std::move(*observers)) {}

  HybridKalman(HybridKalman &&other)
      : X_hat_(other.X_hat_), index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  // Getters for Q
  const Eigen::Matrix<Scalar, number_of_states, number_of_states> &Q() const {
    return Q_;
  }
  Scalar Q(int i, int j) const { return Q()(i, j); }
  // Getters for R
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> &R() const {
    return R_;
  }
  Scalar R(int i, int j) const { return R()(i, j); }

  // Getters for P
  const Eigen::Matrix<Scalar, number_of_states, number_of_states> &P() const {
    return P_;
  }
  Scalar P(int i, int j) const { return P()(i, j); }

  // Getters for X_hat
  const Eigen::Matrix<Scalar, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_X_hat() { return X_hat_; }

  void Reset(StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                      number_of_outputs> *plant) {
    X_hat_.setZero();
    P_ = coefficients().P_steady_state;
    UpdateQR(plant, ::std::chrono::milliseconds(5));
  }

  void Predict(StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                        number_of_outputs, Scalar> *plant,
               const Eigen::Matrix<Scalar, number_of_inputs, 1> &new_u,
               ::std::chrono::nanoseconds dt) {
    // Trigger the predict step.  This will update A() and B() in the plant.
    mutable_X_hat() = plant->Update(X_hat(), new_u, dt);

    UpdateQR(plant, dt);
    P_ = plant->A() * P_ * plant->A().transpose() + Q_;
  }

  void Correct(
      const StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                     number_of_outputs, Scalar> &plant,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U,
      const Eigen::Matrix<Scalar, number_of_outputs, 1> &Y) {
    Eigen::Matrix<Scalar, number_of_outputs, 1> Y_bar =
        Y - (plant.C() * X_hat_ + plant.D() * U);
    Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> S =
        plant.C() * P_ * plant.C().transpose() + R_;
    Eigen::Matrix<Scalar, number_of_states, number_of_outputs> KalmanGain;
    KalmanGain =
        (S.transpose().ldlt().solve((P() * plant.C().transpose()).transpose()))
            .transpose();
    X_hat_ = X_hat_ + KalmanGain * Y_bar;
    P_ = (plant.coefficients().A_continuous.Identity() -
          KalmanGain * plant.C()) *
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
  void UpdateQR(StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                                         number_of_outputs> *plant,
                ::std::chrono::nanoseconds dt) {
    // Now, compute the discrete time Q and R coefficients.
    Eigen::Matrix<Scalar, number_of_states, number_of_states> Qtemp =
        (coefficients().Q_continuous +
         coefficients().Q_continuous.transpose()) /
        static_cast<Scalar>(2.0);
    Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> Rtemp =
        (coefficients().R_continuous +
         coefficients().R_continuous.transpose()) /
        static_cast<Scalar>(2.0);

    Eigen::Matrix<Scalar, 2 * number_of_states, 2 * number_of_states> M_gain;
    M_gain.setZero();
    // Set up the matrix M = [[-A, Q], [0, A.T]]
    M_gain.template block<number_of_states, number_of_states>(0, 0) =
        -plant->coefficients().A_continuous;
    M_gain.template block<number_of_states, number_of_states>(
        0, number_of_states) = Qtemp;
    M_gain.template block<number_of_states, number_of_states>(
        number_of_states, number_of_states) =
        plant->coefficients().A_continuous.transpose();

    Eigen::Matrix<Scalar, 2 * number_of_states, 2 *number_of_states> phi =
        (M_gain *
         ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt)
             .count())
            .exp();

    // Phi12 = phi[0:number_of_states, number_of_states:2*number_of_states]
    // Phi22 = phi[number_of_states:2*number_of_states,
    // number_of_states:2*number_of_states]
    Eigen::Matrix<Scalar, number_of_states, number_of_states> phi12 =
        phi.block(0, number_of_states, number_of_states, number_of_states);
    Eigen::Matrix<Scalar, number_of_states, number_of_states> phi22 = phi.block(
        number_of_states, number_of_states, number_of_states, number_of_states);

    Q_ = phi22.transpose() * phi12;
    Q_ = (Q_ + Q_.transpose()) / static_cast<Scalar>(2.0);
    R_ = Rtemp /
         ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt)
             .count();
  }

  // Internal state estimate.
  Eigen::Matrix<Scalar, number_of_states, 1> X_hat_;
  // Internal covariance estimate.
  Eigen::Matrix<Scalar, number_of_states, number_of_states> P_;

  // Discretized Q and R for the kalman filter.
  Eigen::Matrix<Scalar, number_of_states, number_of_states> Q_;
  Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> R_;

  int index_ = 0;
  ::std::vector<::std::unique_ptr<HybridKalmanCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      coefficients_;
};

#endif  // FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_H_
