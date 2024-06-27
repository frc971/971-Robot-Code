#ifndef FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_CONVERTERS_H_
#define FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_CONVERTERS_H_

#include "frc971/control_loops/hybrid_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop_converters.h"
#include "frc971/control_loops/state_feedback_loop_static.h"

namespace frc971::control_loops {
// These converters are used for constructing a hybrid StateFeedbackLoop
// object from a flatbuffer.
// The only method that should typically be called by users is
// MakeHybridStateFeedbackLoop().
template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<StateFeedbackHybridPlantCoefficients<
    number_of_states, number_of_inputs, number_of_outputs>>
MakeStateFeedbackHybridPlantCoefficients(
    const fbs::StateFeedbackHybridPlantCoefficients &coefficients) {
  CHECK(coefficients.a_continuous() != nullptr);
  CHECK(coefficients.b_continuous() != nullptr);
  CHECK(coefficients.c() != nullptr);
  CHECK(coefficients.d() != nullptr);
  CHECK(coefficients.u_max() != nullptr);
  CHECK(coefficients.u_min() != nullptr);
  CHECK(coefficients.u_limit_coefficient() != nullptr);
  CHECK(coefficients.u_limit_constant() != nullptr);

  return std::make_unique<StateFeedbackHybridPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_states>(
          *coefficients.a_continuous()),
      ToEigenOrDie<number_of_states, number_of_inputs>(
          *coefficients.b_continuous()),
      ToEigenOrDie<number_of_outputs, number_of_states>(*coefficients.c()),
      ToEigenOrDie<number_of_outputs, number_of_inputs>(*coefficients.d()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_max()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_min()),
      ToEigenOrDie<number_of_inputs, number_of_states>(
          *coefficients.u_limit_coefficient()),
      ToEigenOrDie<number_of_inputs, 1>(*coefficients.u_limit_constant()),
      coefficients.delayed_u());
}

template <int number_of_states, int number_of_inputs, int number_of_outputs>
std::unique_ptr<HybridKalmanCoefficients<number_of_states, number_of_inputs,
                                         number_of_outputs>>
MakeHybridKalmanCoefficients(
    const fbs::HybridKalmanCoefficients &coefficients) {
  CHECK(coefficients.q_continuous() != nullptr);
  CHECK(coefficients.r_continuous() != nullptr);
  CHECK(coefficients.p_steady_state() != nullptr);
  return std::make_unique<HybridKalmanCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>(
      ToEigenOrDie<number_of_states, number_of_states>(
          *coefficients.q_continuous()),
      ToEigenOrDie<number_of_outputs, number_of_outputs>(
          *coefficients.r_continuous()),
      ToEigenOrDie<number_of_states, number_of_states>(
          *coefficients.p_steady_state()),
      coefficients.delayed_u());
}
template <int number_of_states, int number_of_inputs, int number_of_outputs>
StateFeedbackLoop<
    number_of_states, number_of_inputs, number_of_outputs, double,
    StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                             number_of_outputs>,
    HybridKalman<number_of_states, number_of_inputs, number_of_outputs>>
MakeHybridStateFeedbackLoop(
    const flatbuffers::Vector<
        flatbuffers::Offset<fbs::StateFeedbackHybridLoopCoefficients>>
        &coefficients) {
  CHECK_LE(1u, coefficients.size());
  std::vector<std::unique_ptr<StateFeedbackHybridPlantCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      plant_coefficients;
  std::vector<std::unique_ptr<StateFeedbackControllerCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      controller_coefficients;
  std::vector<std::unique_ptr<HybridKalmanCoefficients<
      number_of_states, number_of_inputs, number_of_outputs>>>
      observer_coefficients;
  for (const fbs::StateFeedbackHybridLoopCoefficients *loop : coefficients) {
    CHECK(loop->has_plant());
    CHECK(loop->has_controller());
    CHECK(loop->has_observer());
    plant_coefficients.emplace_back(
        MakeStateFeedbackHybridPlantCoefficients<
            number_of_states, number_of_inputs, number_of_outputs>(
            *loop->plant()));
    controller_coefficients.emplace_back(
        MakeStateFeedbackControllerCoefficients<
            number_of_states, number_of_inputs, number_of_outputs>(
            *loop->controller()));
    observer_coefficients.emplace_back(
        MakeHybridKalmanCoefficients<number_of_states, number_of_inputs,
                                     number_of_outputs>(*loop->observer()));
  }

  return StateFeedbackLoop<
      number_of_states, number_of_inputs, number_of_outputs, double,
      StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                               number_of_outputs>,
      HybridKalman<number_of_states, number_of_inputs, number_of_outputs>>(
      StateFeedbackHybridPlant<number_of_states, number_of_inputs,
                               number_of_outputs>(
          std::move(plant_coefficients)),
      StateFeedbackController<number_of_states, number_of_inputs,
                              number_of_outputs>(
          std::move(controller_coefficients)),
      HybridKalman<number_of_states, number_of_inputs, number_of_outputs>(
          std::move(observer_coefficients)));
}
}  // namespace frc971::control_loops

#endif  // FRC971_CONTROL_LOOPS_HYBRID_STATE_FEEDBACK_LOOP_CONVERTERS_H_
