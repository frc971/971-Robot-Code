#include <chrono>
#include <memory>

#include "frc971/constants.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"

namespace frc971::control_loops {

template <typename SubsystemStatus, typename SubsystemState,
          typename SubsystemConstants>

// Class used for simulating a single degree of freedom subsystem in test.
// Simulates the state of the subsystem as a voltage is applied
class SubsystemSimulator {
 public:
  SubsystemSimulator(CappedTestPlant *plant, PositionSensorSimulator encoder,
                     const SubsystemConstants subsystem_constants,
                     const frc971::constants::Range range,
                     double encoder_offset, const std::chrono::nanoseconds dt)
      : plant_(plant),
        encoder_(encoder),
        subsystem_constants_(subsystem_constants),
        range_(range),
        encoder_offset_(encoder_offset),
        dt_(dt) {}

  void InitializePosition(double start_pos) {
    plant_->mutable_X(0, 0) = start_pos;
    plant_->mutable_X(1, 0) = 0.0;

    encoder_.Initialize(start_pos, 0.0, encoder_offset_);
  }

  // Simulates the superstructure for a single timestep.
  void Simulate(double voltage, const SubsystemStatus *status) {
    double last_velocity = plant_->X(1, 0);

    const double voltage_check =
        (static_cast<SubsystemState>(status->state()) ==
         SubsystemState::RUNNING)
            ? subsystem_constants_.subsystem_params.operating_voltage
            : subsystem_constants_.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(voltage, 0.0, voltage_check);

    ::Eigen::Matrix<double, 1, 1> U;
    U << voltage + plant_->voltage_offset();
    plant_->Update(U);

    const double position = plant_->Y(0, 0);

    encoder_.MoveTo(position);

    EXPECT_GE(position, range_.lower_hard);
    EXPECT_LE(position, range_.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);

    const double velocity = plant_->X(1, 0);
    const double acceleration = (velocity - last_velocity) / loop_time;

    EXPECT_GE(peak_acceleration_, acceleration);
    EXPECT_LE(-peak_acceleration_, acceleration);
    EXPECT_GE(peak_velocity_, velocity);
    EXPECT_LE(-peak_velocity_, velocity);
  }

  void set_peak_acceleration(double value) { peak_acceleration_ = value; }
  void set_peak_velocity(double value) { peak_velocity_ = value; }

  void set_controller_index(size_t index) { plant_->set_index(index); }

  PositionSensorSimulator *encoder() { return &encoder_; }

 private:
  std::unique_ptr<CappedTestPlant> plant_;
  PositionSensorSimulator encoder_;
  const SubsystemConstants subsystem_constants_;
  const frc971::constants::Range range_;

  double encoder_offset_ = 0.0;

  double peak_velocity_ = std::numeric_limits<double>::infinity();
  double peak_acceleration_ = std::numeric_limits<double>::infinity();

  const std::chrono::nanoseconds dt_;
};

}  // namespace frc971::control_loops