#include "absl/flags/flag.h"
#include "ctre/phoenix/cci/Diagnostics_CCI.h"

#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_static.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"
#include "frc971/wpilib/swerve/swerve_module.h"
#include "frc971/wpilib/talonfx.h"
#include "y2025/constants.h"
#include "y2025/constants/constants_generated.h"

using frc971::control_loops::swerve::SwerveModuleCanPositionStatic;
using frc971::wpilib::CANSensorReader;
using frc971::wpilib::TalonFX;
using frc971::wpilib::TalonFXParams;
using frc971::wpilib::swerve::DrivetrainWriter;
using frc971::wpilib::swerve::SwerveModule;
using frc971::wpilib::swerve::SwerveModules;
ABSL_FLAG(bool, ctre_diag_server, false,
          "If true, enable the diagnostics server for interacting with "
          "devices on the CAN bus using Phoenix Tuner");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  std::vector<aos::ShmEventLoop *> loops;

  // Thread 1

  frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

  ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());

  frc971::constants::ConstantsFetcher<y2025::Constants> constants_fetcher(
      &constant_fetcher_event_loop);

  const y2025::Constants *robot_constants = &constants_fetcher.constants();

  loops.push_back(&constant_fetcher_event_loop);

  std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;
  std::vector<std::shared_ptr<TalonFX>> falcons;

  const y2025::CurrentLimits *current_limits =
      robot_constants->common()->current_limits();

  SwerveModules modules{
      .front_left = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{13, true},
          frc971::wpilib::TalonFXParams{0, false}, "Drivetrain Bus",
          &signals_registry, current_limits->drivetrain_stator_current_limit(),
          current_limits->drivetrain_supply_current_limit(),
          current_limits->steer_stator_current_limit(),
          current_limits->steer_supply_current_limit()),
      .front_right = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{20, true},
          frc971::wpilib::TalonFXParams{2, true}, "Drivetrain Bus",
          &signals_registry, current_limits->drivetrain_stator_current_limit(),
          current_limits->drivetrain_supply_current_limit(),
          current_limits->steer_stator_current_limit(),
          current_limits->steer_supply_current_limit()),
      .back_left = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{19, true},
          frc971::wpilib::TalonFXParams{3, false}, "Drivetrain Bus",
          &signals_registry, current_limits->drivetrain_stator_current_limit(),
          current_limits->drivetrain_supply_current_limit(),
          current_limits->steer_stator_current_limit(),
          current_limits->steer_supply_current_limit()),
      .back_right = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{7, true},
          frc971::wpilib::TalonFXParams{1, true}, "Drivetrain Bus",
          &signals_registry, current_limits->drivetrain_stator_current_limit(),
          current_limits->drivetrain_supply_current_limit(),
          current_limits->steer_stator_current_limit(),
          current_limits->steer_supply_current_limit())};

  // Thread 2
  aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());

  can_sensor_reader_event_loop.set_name("CANSensorReader");

  modules.PopulateFalconsVector(&falcons);

  aos::Sender<frc971::control_loops::swerve::CanPositionStatic>
      can_position_sender =
          can_sensor_reader_event_loop
              .MakeSender<frc971::control_loops::swerve::CanPositionStatic>(
                  "/drivetrain");

  CANSensorReader can_sensor_reader(
      &can_sensor_reader_event_loop, std::move(signals_registry), falcons,
      [falcons, modules,
       &can_position_sender](ctre::phoenix::StatusCode status) {
        // TODO(max): use status properly in the flatbuffer.
        (void)status;

        aos::Sender<frc971::control_loops::swerve::CanPositionStatic>::
            StaticBuilder builder = can_position_sender.MakeStaticBuilder();
        for (auto falcon : falcons) {
          falcon->RefreshNontimesyncedSignals();
        }

        const SwerveModule::ModuleGearRatios gear_ratios{
            .rotation = y2025::constants::Values::kRotationModuleRatio,
            .translation = y2025::constants::Values::kTranslationModuleRatio()};

        modules.front_left->PopulateCanPosition(builder->add_front_left(),
                                                gear_ratios);
        modules.front_right->PopulateCanPosition(builder->add_front_right(),
                                                 gear_ratios);
        modules.back_left->PopulateCanPosition(builder->add_back_left(),
                                               gear_ratios);
        modules.back_right->PopulateCanPosition(builder->add_back_right(),
                                                gear_ratios);

        builder.CheckOk(builder.Send());
      });

  loops.push_back(&can_sensor_reader_event_loop);

  // Thread 3
  // Setup CAN
  if (!absl::GetFlag(FLAGS_ctre_diag_server)) {
    c_Phoenix_Diagnostics_SetSecondsToStart(-1);
    c_Phoenix_Diagnostics_Dispose();
  }

  ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
      y2025::constants::Values::kDrivetrainRxPriority, true, "Drivetrain Bus");
  ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
      y2025::constants::Values::kDrivetrainTxPriority, true, "Drivetrain Bus");

  aos::ShmEventLoop drivetrain_writer_event_loop(&config.message());

  drivetrain_writer_event_loop.set_name("DrivetrainWriter");

  DrivetrainWriter drivetrain_writer(
      &drivetrain_writer_event_loop,
      y2025::constants::Values::kDrivetrainWriterPriority, 12);

  drivetrain_writer.set_talonfxs(modules);

  loops.push_back(&drivetrain_writer_event_loop);

  std::vector<std::thread> threads;

  for (aos::ShmEventLoop *event_loop : loops) {
    threads.emplace_back([event_loop]() {
      LOG(INFO) << "Starting event loop " << event_loop->name();
      event_loop->Run();
    });
  }

  for (std::thread &thread : threads) {
    thread.join();
  }
  return 0;
};
