#ifndef FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_
#define FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_

// wikipedia article is <http://en.wikipedia.org/wiki/State_observer>

#include "Eigen/Dense"

template <int number_of_states, int number_of_outputs>
class StateFeedbackPlant {
 public:
  Eigen::Matrix<double, number_of_states, 1> X;
  Eigen::Matrix<double, number_of_outputs, 1> Y;
  Eigen::Matrix<double, number_of_outputs, 1> U;
  Eigen::Matrix<double, number_of_outputs, 1> U_max;
  Eigen::Matrix<double, number_of_outputs, 1> U_min;
  Eigen::Matrix<double, number_of_states, number_of_states> A;
  Eigen::Matrix<double, number_of_states, number_of_outputs> B;
  Eigen::Matrix<double, number_of_outputs, number_of_states> C;
  Eigen::Matrix<double, number_of_outputs, number_of_outputs> D;
  // TODO(aschuh): These following 2 lines are here because MATRIX_INIT
  // assumes that you have a controller as well as a plant.
  Eigen::Matrix<double, number_of_states, number_of_outputs> L;
  Eigen::Matrix<double, number_of_outputs, number_of_states> K;

  StateFeedbackPlant() {
    X.setZero();
    Y.setZero();
    U.setZero();
  }

  virtual ~StateFeedbackPlant() {}

  // If U is outside the hardware range, limit it before the plant tries to use
  // it.
  virtual void CapU() {
    for (int i = 0; i < number_of_outputs; ++i) {
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
  static const int number_of_states_var = number_of_states;
  static const int number_of_outputs_var = number_of_outputs;
};

template <int number_of_states, int number_of_outputs>
class StateFeedbackLoop {
 public:
  Eigen::Matrix<double, number_of_states, 1> X;
  Eigen::Matrix<double, number_of_states, 1> X_hat;
  Eigen::Matrix<double, number_of_outputs, 1> Y;
  Eigen::Matrix<double, number_of_states, 1> R;
  Eigen::Matrix<double, number_of_outputs, 1> U;
  Eigen::Matrix<double, number_of_outputs, 1> U_max;
  Eigen::Matrix<double, number_of_outputs, 1> U_min;
  Eigen::Matrix<double, number_of_outputs, 1> U_ff;
  Eigen::Matrix<double, number_of_states, number_of_states> A;
  Eigen::Matrix<double, number_of_states, number_of_outputs> B;
  // K in wikipedia article
  Eigen::Matrix<double, number_of_outputs, number_of_states> C;
  Eigen::Matrix<double, number_of_outputs, number_of_outputs> D;
  // B in wikipedia article
  Eigen::Matrix<double, number_of_states, number_of_outputs> L;
  // C in wikipedia article
  Eigen::Matrix<double, number_of_outputs, number_of_states> K;

  StateFeedbackLoop() {
    // You have to initialize all the matrices to 0 or else all their elements
    // are undefined.
    X.setZero();
    X_hat.setZero();
    Y.setZero();
    R.setZero();
    U.setZero();
    U_ff.setZero();
  }
  virtual ~StateFeedbackLoop() {}

  virtual void FeedForward() {
    for (int i = 0; i < number_of_outputs; ++i) {
      U_ff[i] = 0.0;
    }
  }
  // If U is outside the hardware range, limit it before it
  // gets added to the observer.
  virtual void CapU() {
    for (int i = 0; i < number_of_outputs; ++i) {
      if (U[i] > U_max[i]) {
        U[i] = U_max[i];
      } else if (U[i] < U_min[i]) {
        U[i] = U_min[i];
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
      // new power = constant * (goal - current prediction)
      U.noalias() = K * (R - X_hat);
      CapU();
    }

    if (update_observer) {
      X_hat = (A - L * C) * X_hat + L * Y + B * U;
    } else {
      X_hat = A * X_hat + B * U;
    }
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int number_of_states_var = number_of_states;
  static const int number_of_outputs_var = number_of_outputs;
};
#endif  // FRC971_CONTROL_LOOPS_STATEFEEDBACKLOOP_H_
