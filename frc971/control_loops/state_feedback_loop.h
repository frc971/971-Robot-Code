#ifndef FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_

#include <assert.h>

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"

#if defined(__linux__)
#include "aos/logging/logging.h"
#endif
#include "aos/macros.h"

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename PlantType, typename ObserverType, typename Scalar>
class StateFeedbackLoop;

// For everything in this file, "inputs" and "outputs" are defined from the
// perspective of the plant. This means U is an input and Y is an output
// (because you give the plant U (powers) and it gives you back a Y (sensor
// values). This is the opposite of what they mean from the perspective of the
// controller (U is an output because that's what goes to the motors and Y is an
// input because that's what comes back from the sensors).

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
struct StateFeedbackPlantCoefficients final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlantCoefficients(const StateFeedbackPlantCoefficients &other)
      : A(other.A),
        B(other.B),
        C(other.C),
        D(other.D),
        U_min(other.U_min),
        U_max(other.U_max) {}

  StateFeedbackPlantCoefficients(
      const Eigen::Matrix<Scalar, number_of_states, number_of_states> &A,
      const Eigen::Matrix<Scalar, number_of_states, number_of_inputs> &B,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_states> &C,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_inputs> &D,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_max,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_min)
      : A(A), B(B), C(C), D(D), U_min(U_min), U_max(U_max) {}

  const Eigen::Matrix<Scalar, number_of_states, number_of_states> A;
  const Eigen::Matrix<Scalar, number_of_states, number_of_inputs> B;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_states> C;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_inputs> D;
  const Eigen::Matrix<Scalar, number_of_inputs, 1> U_min;
  const Eigen::Matrix<Scalar, number_of_inputs, 1> U_max;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
class StateFeedbackPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  StateFeedbackPlant(
      ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<
          number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
          *coefficients)
      : coefficients_(::std::move(*coefficients)), index_(0) {
    Reset();
  }

  StateFeedbackPlant(StateFeedbackPlant &&other) : index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
    X_.swap(other.X_);
    Y_.swap(other.Y_);
  }

  virtual ~StateFeedbackPlant() {}

  const Eigen::Matrix<Scalar, number_of_states, number_of_states> &A() const {
    return coefficients().A;
  }
  Scalar A(int i, int j) const { return A()(i, j); }
  const Eigen::Matrix<Scalar, number_of_states, number_of_inputs> &B() const {
    return coefficients().B;
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
  Scalar U_max(int i, int j = 0) const { return U_max()(i, j); }

  const Eigen::Matrix<Scalar, number_of_states, 1> &X() const { return X_; }
  Scalar X(int i, int j = 0) const { return X()(i, j); }
  const Eigen::Matrix<Scalar, number_of_outputs, 1> &Y() const { return Y_; }
  Scalar Y(int i, int j = 0) const { return Y()(i, j); }

  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_X() { return X_; }
  Scalar &mutable_X(int i, int j = 0) { return mutable_X()(i, j); }
  Eigen::Matrix<Scalar, number_of_outputs, 1> &mutable_Y() { return Y_; }
  Scalar &mutable_Y(int i, int j = 0) { return mutable_Y()(i, j); }

  size_t coefficients_size() const { return coefficients_.size(); }

  const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                       number_of_outputs, Scalar>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
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
  }

  // Assert that U is within the hardware range.
  virtual void CheckU(const Eigen::Matrix<Scalar, number_of_inputs, 1> &U) {
    for (int i = 0; i < kNumInputs; ++i) {
      if (U(i, 0) > U_max(i, 0) + static_cast<Scalar>(0.00001) ||
          U(i, 0) < U_min(i, 0) - static_cast<Scalar>(0.00001)) {
#if defined(__linux__)
        AOS_LOG(FATAL, "U out of range\n");
#else
        abort();
#endif
      }
    }
  }

  // Computes the new X and Y given the control input.
  void Update(const Eigen::Matrix<Scalar, number_of_inputs, 1> &U) {
    // Powers outside of the range are more likely controller bugs than things
    // that the plant should deal with.
    CheckU(U);
    X_ = Update(X(), U);
    UpdateY(U);
  }

  // Computes the new Y given the control input.
  void UpdateY(const Eigen::Matrix<Scalar, number_of_inputs, 1> &U) {
    Y_ = C() * X() + D() * U;
  }

  Eigen::Matrix<Scalar, number_of_states, 1> Update(
      const Eigen::Matrix<Scalar, number_of_states, 1> X,
      const Eigen::Matrix<Scalar, number_of_inputs, 1> &U) const {
    return A() * X + B() * U;
  }

 protected:
  // these are accessible from non-templated subclasses
  static const int kNumStates = number_of_states;
  static const int kNumOutputs = number_of_outputs;
  static const int kNumInputs = number_of_inputs;

 private:
  Eigen::Matrix<Scalar, number_of_states, 1> X_;
  Eigen::Matrix<Scalar, number_of_outputs, 1> Y_;

  ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
      coefficients_;

  int index_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackPlant);
};

// A container for all the controller coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
struct StateFeedbackControllerCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> K;
  const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> Kff;

  StateFeedbackControllerCoefficients(
      const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> &K,
      const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> &Kff)
      : K(K), Kff(Kff) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
class StateFeedbackController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackController(
      ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<
          number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
          *controllers)
      : coefficients_(::std::move(*controllers)) {}

  StateFeedbackController(StateFeedbackController &&other)
      : index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> &K() const {
    return coefficients().K;
  }
  Scalar K(int i, int j) const { return K()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, number_of_states> &Kff() const {
    return coefficients().Kff;
  }
  Scalar Kff(int i, int j) const { return Kff()(i, j); }

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
                                            number_of_outputs, Scalar>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackControllerCoefficients<number_of_states, number_of_inputs,
                                            number_of_outputs, Scalar>
      &coefficients() const {
    return *coefficients_[index_];
  }

 private:
  int index_ = 0;
  ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
      coefficients_;
};

// A container for all the observer coefficients.
template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
struct StateFeedbackObserverCoefficients final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const Eigen::Matrix<Scalar, number_of_states, number_of_outputs> KalmanGain;
  const Eigen::Matrix<Scalar, number_of_states, number_of_states> Q;
  const Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> R;

  StateFeedbackObserverCoefficients(
      const Eigen::Matrix<Scalar, number_of_states, number_of_outputs>
          &KalmanGain,
      const Eigen::Matrix<Scalar, number_of_states, number_of_states> &Q,
      const Eigen::Matrix<Scalar, number_of_outputs, number_of_outputs> &R)
      : KalmanGain(KalmanGain), Q(Q), R(R) {}
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double>
class StateFeedbackObserver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackObserver(
      ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<
          number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
          *observers)
      : coefficients_(::std::move(*observers)) {}

  StateFeedbackObserver(StateFeedbackObserver &&other)
      : X_hat_(other.X_hat_), index_(other.index_) {
    ::std::swap(coefficients_, other.coefficients_);
  }

  const Eigen::Matrix<Scalar, number_of_states, number_of_outputs> &KalmanGain()
      const {
    return coefficients().KalmanGain;
  }
  Scalar KalmanGain(int i, int j) const { return KalmanGain()(i, j); }

  const Eigen::Matrix<Scalar, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_X_hat() { return X_hat_; }

  void Reset(StateFeedbackPlant<number_of_states, number_of_inputs,
                                number_of_outputs, Scalar> * /*loop*/) {
    X_hat_.setZero();
  }

  void Predict(StateFeedbackPlant<number_of_states, number_of_inputs,
                                  number_of_outputs, Scalar> *plant,
               const Eigen::Matrix<Scalar, number_of_inputs, 1> &new_u,
               ::std::chrono::nanoseconds /*dt*/) {
    mutable_X_hat() = plant->Update(X_hat(), new_u);
  }

  void Correct(const StateFeedbackPlant<number_of_states, number_of_inputs,
                                        number_of_outputs, Scalar> &plant,
               const Eigen::Matrix<Scalar, number_of_inputs, 1> &U,
               const Eigen::Matrix<Scalar, number_of_outputs, 1> &Y) {
    mutable_X_hat() += KalmanGain() * (Y - plant.C() * X_hat() - plant.D() * U);
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
                                          number_of_outputs, Scalar>
      &coefficients(int index) const {
    return *coefficients_[index];
  }

  const StateFeedbackObserverCoefficients<number_of_states, number_of_inputs,
                                          number_of_outputs, Scalar>
      &coefficients() const {
    return *coefficients_[index_];
  }

 private:
  // Internal state estimate.
  Eigen::Matrix<Scalar, number_of_states, 1> X_hat_;

  int index_ = 0;
  ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<
      number_of_states, number_of_inputs, number_of_outputs, Scalar>>>
      coefficients_;
};

template <int number_of_states, int number_of_inputs, int number_of_outputs,
          typename Scalar = double,
          typename PlantType = StateFeedbackPlant<
              number_of_states, number_of_inputs, number_of_outputs, Scalar>,
          typename ObserverType = StateFeedbackObserver<
              number_of_states, number_of_inputs, number_of_outputs, Scalar>>
class StateFeedbackLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit StateFeedbackLoop(
      PlantType &&plant,
      StateFeedbackController<number_of_states, number_of_inputs,
                              number_of_outputs, Scalar> &&controller,
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

  const Eigen::Matrix<Scalar, number_of_states, 1> &X_hat() const {
    return observer().X_hat();
  }
  Scalar X_hat(int i, int j = 0) const { return X_hat()(i, j); }
  const Eigen::Matrix<Scalar, number_of_states, 1> &R() const { return R_; }
  Scalar R(int i, int j) const { return R()(i, j); }
  const Eigen::Matrix<Scalar, number_of_states, 1> &next_R() const {
    return next_R_;
  }
  Scalar next_R(int i, int j) const { return next_R()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, 1> &U() const { return U_; }
  Scalar U(int i, int j) const { return U()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, 1> &U_uncapped() const {
    return U_uncapped_;
  }
  Scalar U_uncapped(int i, int j) const { return U_uncapped()(i, j); }
  const Eigen::Matrix<Scalar, number_of_inputs, 1> &ff_U() const {
    return ff_U_;
  }
  Scalar ff_U(int i, int j) const { return ff_U()(i, j); }

  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_X_hat() {
    return observer_.mutable_X_hat();
  }
  Scalar &mutable_X_hat(int i, int j = 0) { return mutable_X_hat()(i, j); }
  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_R() { return R_; }
  Scalar &mutable_R(int i, int j) { return mutable_R()(i, j); }
  Eigen::Matrix<Scalar, number_of_states, 1> &mutable_next_R() {
    return next_R_;
  }
  Scalar &mutable_next_R(int i, int j) { return mutable_next_R()(i, j); }
  Eigen::Matrix<Scalar, number_of_inputs, 1> &mutable_U() { return U_; }
  Scalar &mutable_U(int i, int j) { return mutable_U()(i, j); }
  Eigen::Matrix<Scalar, number_of_inputs, 1> &mutable_U_uncapped() {
    return U_uncapped_;
  }
  Scalar &mutable_U_uncapped(int i, int j) {
    return mutable_U_uncapped()(i, j);
  }

  const PlantType &plant() const { return plant_; }
  PlantType *mutable_plant() { return &plant_; }

  const StateFeedbackController<number_of_states, number_of_inputs,
                                number_of_outputs, Scalar>
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
    observer_.Reset(this->mutable_plant());
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
  void Correct(const Eigen::Matrix<Scalar, number_of_outputs, 1> &Y) {
    observer_.Correct(this->plant(), U(), Y);
  }

  const Eigen::Matrix<Scalar, number_of_states, 1> error() const {
    return R() - X_hat();
  }

  // Returns the calculated controller power.
  virtual const Eigen::Matrix<Scalar, number_of_inputs, 1> ControllerOutput() {
    // TODO(austin): Should this live in StateSpaceController?
    ff_U_ = FeedForward();
    return controller().K() * error() + ff_U_;
  }

  // Calculates the feed forwards power.
  virtual const Eigen::Matrix<Scalar, number_of_inputs, 1> FeedForward() {
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

  void UpdateObserver(const Eigen::Matrix<Scalar, number_of_inputs, 1> &new_u,
                      ::std::chrono::nanoseconds dt) {
    observer_.Predict(this->mutable_plant(), new_u, dt);
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

  StateFeedbackController<number_of_states, number_of_inputs, number_of_outputs,
                          Scalar>
      controller_;

  ObserverType observer_;

  // These are accessible from non-templated subclasses.
  static constexpr int kNumStates = number_of_states;
  static constexpr int kNumOutputs = number_of_outputs;
  static constexpr int kNumInputs = number_of_inputs;

  // Portion of U which is based on the feed-forwards.
  Eigen::Matrix<Scalar, number_of_inputs, 1> ff_U_;

 private:
  // Current goal (Used by the feed-back controller).
  Eigen::Matrix<Scalar, number_of_states, 1> R_;
  // Goal to go to in the next cycle (Used by Feed-Forward controller.)
  Eigen::Matrix<Scalar, number_of_states, 1> next_R_;
  // Computed output after being capped.
  Eigen::Matrix<Scalar, number_of_inputs, 1> U_;
  // Computed output before being capped.
  Eigen::Matrix<Scalar, number_of_inputs, 1> U_uncapped_;

  DISALLOW_COPY_AND_ASSIGN(StateFeedbackLoop);
};

#endif  // FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_H_
