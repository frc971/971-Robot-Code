#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2020/control_loops/superstructure/shooter/shooter_tuning_params_generated.h"

ABSL_FLAG(double, velocity_initial_finisher, 300.0,
          "Initial finisher velocity");
ABSL_FLAG(double, velocity_final_finisher, 500.0, "Final finisher velocity");
ABSL_FLAG(double, velocity_finisher_increment, 25.0,
          "Finisher velocity increment");

ABSL_FLAG(double, velocity_initial_accelerator, 180.0,
          "Initial accelerator velocity");
ABSL_FLAG(double, velocity_final_accelerator, 250.0,
          "Final accelerator velocity");
ABSL_FLAG(double, velocity_accelerator_increment, 20.0,
          "Accelerator velocity increment");

ABSL_FLAG(int32_t, balls_per_iteration, 10,
          "Balls to shoot per iteration in the velocity sweep");

namespace shooter = y2020::control_loops::superstructure::shooter;

flatbuffers::Offset<shooter::FlywheelTuningParams> BuildFlywheelTuningParams(
    aos::Sender<shooter::TuningParams>::Builder &builder,
    double velocity_initial, double velocity_final, double velocity_increment) {
  auto flywheel_tuning_params_builder =
      builder.MakeBuilder<shooter::FlywheelTuningParams>();
  flywheel_tuning_params_builder.add_velocity_initial(velocity_initial);
  flywheel_tuning_params_builder.add_velocity_final(velocity_final);
  flywheel_tuning_params_builder.add_velocity_increment(velocity_increment);
  return flywheel_tuning_params_builder.Finish();
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  aos::ShmEventLoop event_loop(&config.message());

  auto sender = event_loop.MakeSender<shooter::TuningParams>("/superstructure");
  auto builder = sender.MakeBuilder();

  auto finisher_params = BuildFlywheelTuningParams(
      builder, absl::GetFlag(FLAGS_velocity_initial_finisher),
      absl::GetFlag(FLAGS_velocity_final_finisher),
      absl::GetFlag(FLAGS_velocity_finisher_increment));
  auto accelerator_params = BuildFlywheelTuningParams(
      builder, absl::GetFlag(FLAGS_velocity_initial_accelerator),
      absl::GetFlag(FLAGS_velocity_final_accelerator),
      absl::GetFlag(FLAGS_velocity_accelerator_increment));

  auto tuning_params_builder = builder.MakeBuilder<shooter::TuningParams>();
  tuning_params_builder.add_finisher(finisher_params);
  tuning_params_builder.add_accelerator(accelerator_params);
  tuning_params_builder.add_balls_per_iteration(
      absl::GetFlag(FLAGS_balls_per_iteration));
  builder.CheckOk(builder.Send(tuning_params_builder.Finish()));

  return 0;
}
