#ifndef FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_
#define FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_

// wikipedia article is <http://en.wikipedia.org/wiki/State_observer>

// Stupid vxworks system headers define it which blows up Eigen...
#undef m_data

#include "Eigen/Dense"

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_states, number_of_states> A;
  const Eigen::Matrix<double, number_of_states, number_of_inputs> B;
  const Eigen::Matrix<double, number_of_outputs, number_of_states> C;
  const Eigen::Matrix<double, number_of_outputs, number_of_inputs> D;
  const Eigen::Matrix<double, number_of_inputs, 1> U_min;
  const Eigen::Matrix<double, number_of_inputs, 1> U_max;

  Eigen::Matrix<double, number_of_states, 1> X;
  Eigen::Matrix<double, number_of_outputs, 1> Y;
  Eigen::Matrix<double, number_of_inputs, 1> U;

  StateFeedbackPlant(const StateFeedbackPlant &other)
      : A(other.A),
        B(other.B),
        C(other.C),
        D(other.D),
        U_min(other.U_min),
        U_max(other.U_max) {
    X.setZero();
    Y.setZero();
    U.setZero();
  }

  StateFeedbackPlant(
      const Eigen::Matrix<double, number_of_states, number_of_states> &A,
      const Eigen::Matrix<double, number_of_states, number_of_inputs> &B,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<double, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<double, number_of_outputs, 1> &U_max,
      const Eigen::Matrix<double, number_of_outputs, 1> &U_min)
      : A(A),
        B(B),
        C(C),
        D(D),
        U_min(U_min),
        U_max(U_max) {
    X.setZero();
    Y.setZero();
    U.setZero();
  }

  virtual ~StateFeedbackPlant() {}

  // If U is outside the hardware range, limit it before the plant tries to use
  // it.
  virtual void CapU() {
    for (int i = 0; i < kNumOutputs; ++i) {
      if (U[i] > U_max[i]) {
        U[i] = U_max[i];
      } else if (U[i] < U_min[i]) {
        U[i] = U_min[i];
      }
    }
  }
  // Computes the new X and Y given the control input.
  void Update() {
    CapU();
    X = A * X + B * U;
    Y = C * X + D * U;
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs>
class StateFeedbackLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<double, number_of_states, number_of_outputs> L;
  const Eigen::Matrix<double, number_of_outputs, number_of_states> K;

  Eigen::Matrix<double, number_of_states, 1> X_hat;
  Eigen::Matrix<double, number_of_states, 1> R;
  Eigen::Matrix<double, number_of_inputs, 1> U;
  Eigen::Matrix<double, number_of_inputs, 1> U_uncapped;
  Eigen::Matrix<double, number_of_outputs, 1> U_ff;
  Eigen::Matrix<double, number_of_outputs, 1> Y;

  StateFeedbackPlant<number_of_states, number_of_inputs,
                     number_of_outputs> plant;

  StateFeedbackLoop(
      const Eigen::Matrix<double, number_of_states, number_of_outputs> &L,
      const Eigen::Matrix<double, number_of_outputs, number_of_states> &K,
      const StateFeedbackPlant<number_of_states, number_of_inputs,
                               number_of_outputs> &plant)
      : L(L),
        K(K),
        plant(plant) {
    X_hat.setZero();
    R.setZero();
    U.setZero();
    U_uncapped.setZero();
    U_ff.setZero();
    Y.setZero();
  }
  virtual ~StateFeedbackLoop() {}

  virtual void FeedForward() {
    for (int i = 0; i < number_of_outputs; ++i) {
      U_ff[i] = 0.0;
    }
  }

  // If U is outside the hardware range, limit it before the plant tries to use
  // it.
  virtual void CapU() {
    for (int i = 0; i < kNumOutputs; ++i) {
      if (U[i] > plant.U_max[i]) {
        U[i] = plant.U_max[i];
      } else if (U[i] < plant.U_min[i]) {
        U[i] = plant.U_min[i];
      }
    }
  }

  // update_observer is whether or not to use the values in Y.
  // stop_motors is whether or not to output all 0s.
  void Update(bool update_observer, bool stop_motors) {
    if (stop_motors) {
      for (int i = 0; i < number_of_outputs; ++i) {
        U[i] = 0.0;
      }
    } else {
      U = U_uncapped = K * (R - X_hat);
      CapU();
    }

    if (update_observer) {
      X_hat = (plant.A - L * plant.C) * X_hat + L * Y + plant.B * U;
    } else {
      X_hat = plant.A * X_hat + plant.B * U;
    }
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;
};

#endif  // FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_
