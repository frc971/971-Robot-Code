#include "y2019/control_loops/superstructure/vacuum.h"

#include <chrono>

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_output_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

namespace chrono = ::std::chrono;

constexpr double Vacuum::kPumpVoltage;
constexpr double Vacuum::kPumpHasPieceVoltage;
constexpr aos::monotonic_clock::duration Vacuum::kTimeAtHigherVoltage;
constexpr aos::monotonic_clock::duration Vacuum::kReleaseTime;

void Vacuum::Iterate(const SuctionGoal *unsafe_goal, float suction_pressure,
                     OutputT *output, bool *has_piece,
                     aos::EventLoop *event_loop) {
  auto monotonic_now = event_loop->monotonic_now();
  bool low_pump_voltage = false;

  // implement a simple low-pass filter on the pressure
  filtered_pressure_ = kSuctionAlpha * suction_pressure +
                       (1 - kSuctionAlpha) * filtered_pressure_;

  const bool new_has_piece =
      filtered_pressure_ < (filtered_had_piece_near_disabled_
                                ? kVacuumOffThreshold
                                : kVacuumOnThreshold);

  if (new_has_piece && !had_piece_) {
    time_at_last_acquisition_ = monotonic_now;
  }
  *has_piece =
      monotonic_now > time_at_last_acquisition_ + kTimeAtHigherVoltage &&
      new_has_piece;

  if (!output && *has_piece) {
    last_disable_has_piece_time_ = monotonic_now;
  }


  // If we've had the piece for enough time, go to lower pump_voltage
  low_pump_voltage = *has_piece && filtered_pressure_ < kVacuumOnThreshold;

  if (unsafe_goal && output) {
    const bool release = !unsafe_goal->grab_piece();

    if (release) {
      last_release_time_ = monotonic_now;
    }

    // Once the vacuum evacuates, the pump speeds up because there is no
    // resistance.  So, we want to turn it down to save the pump from
    // overheating.
    output->pump_voltage =
        release ? 0 : (low_pump_voltage ? kPumpHasPieceVoltage : kPumpVoltage);

    if (unsafe_goal->grab_piece() && unsafe_goal->gamepiece_mode() == 0) {
      output->intake_suction_top = false;
      output->intake_suction_bottom = true;
    } else if (unsafe_goal->grab_piece() && unsafe_goal->gamepiece_mode() == 1) {
      output->intake_suction_top = true;
      output->intake_suction_bottom = true;
    } else {
      output->intake_suction_top = false;
      output->intake_suction_bottom = false;
    }

    // If we intend to release, or recently released, set has_piece to false so
    // that we give the part of the vacuum circuit with the pressure sensor time
    // to equilibrate with the rest of the suction cup.
    if (release || monotonic_now < last_release_time_ + kReleaseTime) {
      *has_piece = false;
    }
  }
  had_piece_ = new_has_piece;

  filtered_had_piece_near_disabled_ =
      *has_piece &&
      monotonic_now < last_disable_has_piece_time_ + chrono::milliseconds(250);
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
