#ifndef FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_CONVERTERS_H_
#define FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_CONVERTERS_H_

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop_static.h"
#include "frc971/math/flatbuffers_matrix.h"

namespace frc971::control_loops {
// These converters are used for constructing a "default" StateFeedbackLoop
// object from a flatbuffer.
// The only method that should typically be called by users is
// MakeStateFeedbackLoop().
// Use hybrid_state_feedback_loop_converters.h for control loops that use the
// hybrid kalman filters.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackPlantCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackPlantCoefficients(
    const fbs::StateFeedbackPlantCoefficients &coefficients) {
  CHECK(coefficients.a() != nullptr);
  CHECK(coefficients.b() != nullptr);
  CHECK(coefficients.c() != nullptr);
  CHECK(coefficients.d() != nullptr);
  CHECK(coefficients.u_max() != nullptr);
  CHECK(coefficients.u_min() != nullptr);
  CHECK(coefficients.u_limit_coefficient() != nullptr);
  CHECK(coefficients.u_limit_constant() != nullptr);

  return std::make_unique<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_states>(*coefficients.a()),
      ToEigenOrDie<number_of_states, number_of_inputs>(*coefficients.b()),
      ToEigenOrDie<number_of_outputs, number_of_states>(*coefficients.c()),
      ToEigenOrDie<number_of_outputs, number_of_inputs>(*coefficients.d()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_max()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_min()),
      ToEigenOrDie<number_of_inputs, number_of_states>(
          *coefficients.u_limit_coefficient()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_limit_constant()),
      std::chrono::nanoseconds(coefficients.dt()), coefficients.delayed_u(),
      ToEigenOrDie<number_of_outputs, 1>(*coefficients.wrap_point()));
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackControllerCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackControllerCoefficients(
    const fbs::StateFeedbackControllerCoefficients &coefficients) {
  CHECK(coefficients.k() != nullptr);
  CHECK(coefficients.kff() != nullptr);
  return std::make_unique<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_inputs, number_of_states>(*coefficients.k()),
      ToEigenOrDie<number_of_inputs, number_of_states>(*coefficients.kff()));
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackObserverCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackObserverCoefficients(
    const fbs::StateFeedbackObserverCoefficients &coefficients) {
  CHECK(coefficients.kalman_gain() != nullptr);
  CHECK(coefficients.q() != nullptr);
  CHECK(coefficients.r() != nullptr);
  return std::make_unique<StateFeedbackObserverCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_outputs>(
          *coefficients.kalman_gain()),
      ToEigenOrDie<number_of_states, number_of_states>(*coefficients.q()),
      ToEigenOrDie<number_of_outputs, number_of_outputs>(*coefficients.r()),
      coefficients.delayed_u());
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
StateFeedbackLoop<number_of_states, number_of_inputs, number_of_outputs>
MakeStateFeedbackLoop(const flatbuffers::Vector<flatbuffers::Offset<
                          fbs::StateFeedbackLoopCoefficients>> &coefficients) {
  CHECK_LE(1u, coefficients.size());
  std::vector<std::unique_ptr<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      plant_coefficients;
  std::vector<std::unique_ptr<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      controller_coefficients;
  std::vector<std::unique_ptr<StateFeedbackObserverCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      observer_coefficients;
  for (const fbs::StateFeedbackLoopCoefficients *loop : coefficients) {
    CHECK(loop->has_plant());
    CHECK(loop->has_controller());
    CHECK(loop->has_observer());
    plant_coefficients.emplace_back(
        MakeStateFeedbackPlantCoefficients<number_of_states, number_of_inputs,
                                           number_of_outputs>(*loop->plant()));
    controller_coefficients.emplace_back(
        MakeStateFeedbackControllerCoefficients<
            number_of_states, number_of_inputs, number_of_outputs>(
            *loop->controller()));
    observer_coefficients.emplace_back(
        MakeStateFeedbackObserverCoefficients<
            number_of_states, number_of_inputs, number_of_outputs>(
            *loop->observer()));
  }

  return StateFeedbackLoop<number_of_states, number_of_inputs,
                           number_of_outputs>(
      StateFeedbackPlant<number_of_states, number_of_inputs, number_of_outputs>(
          std::move(plant_coefficients)),
      StateFeedbackController<number_of_states, number_of_inputs,
                              number_of_outputs>(
          std::move(controller_coefficients)),
      StateFeedbackObserver<number_of_states, number_of_inputs,
                            number_of_outputs>(
          std::move(observer_coefficients)));
}

}  // namespace frc971::control_loops

#endif  // FRC971_CONTROL_LOOPS_STATE_FEEDBACK_LOOP_CONVERTERS_H_
