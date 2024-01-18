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
  return std::make_unique<StateFeedbackPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_states>(
          *CHECK_NOTNULL(coefficients.a())),
      ToEigenOrDie<number_of_states, number_of_inputs>(
          *CHECK_NOTNULL(coefficients.b())),
      ToEigenOrDie<number_of_outputs, number_of_states>(
          *CHECK_NOTNULL(coefficients.c())),
      ToEigenOrDie<number_of_outputs, number_of_inputs>(
          *CHECK_NOTNULL(coefficients.d())),
      ToEigenOrDie<number_of_inputs, 1>(*CHECK_NOTNULL(coefficients.u_max())),
      ToEigenOrDie<number_of_inputs, 1>(*CHECK_NOTNULL(coefficients.u_min())),
      ToEigenOrDie<number_of_inputs, number_of_states>(
          *CHECK_NOTNULL(coefficients.u_limit_coefficient())),
      ToEigenOrDie<number_of_inputs, 1>(
          *CHECK_NOTNULL(coefficients.u_limit_constant())),
      std::chrono::nanoseconds(coefficients.dt()), coefficients.delayed_u());
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackControllerCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackControllerCoefficients(
    const fbs::StateFeedbackControllerCoefficients &coefficients) {
  return std::make_unique<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_inputs, number_of_states>(
          *CHECK_NOTNULL(coefficients.k())),
      ToEigenOrDie<number_of_inputs, number_of_states>(
          *CHECK_NOTNULL(coefficients.kff())));
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackObserverCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackObserverCoefficients(
    const fbs::StateFeedbackObserverCoefficients &coefficients) {
  return std::make_unique<StateFeedbackObserverCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_outputs>(
          *CHECK_NOTNULL(coefficients.kalman_gain())),
      ToEigenOrDie<number_of_states, number_of_states>(
          *CHECK_NOTNULL(coefficients.q())),
      ToEigenOrDie<number_of_outputs, number_of_outputs>(
          *CHECK_NOTNULL(coefficients.r())),
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
