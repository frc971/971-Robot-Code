#include "y2019/control_loops/superstructure/vacuum.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

constexpr double Vacuum::kPumpVoltage;
constexpr double Vacuum::kPumpHasPieceVoltage;
constexpr aos::monotonic_clock::duration Vacuum::kTimeAtHigherVoltage;
constexpr aos::monotonic_clock::duration Vacuum::kTimeToKeepPumpRunning;

void Vacuum::Iterate(const SuctionGoal *unsafe_goal, float suction_pressure,
                     SuperstructureQueue::Output *output, bool *has_piece,
                     aos::EventLoop *event_loop) {
  auto monotonic_now = event_loop->monotonic_now();
  bool low_pump_voltage = false;
  bool no_goal_for_a_bit = false;

  // implement a simple low-pass filter on the pressure
  filtered_pressure_ = kSuctionAlpha * suction_pressure +
                       (1 - kSuctionAlpha) * filtered_pressure_;

  *has_piece = filtered_pressure_ < kVacuumThreshold;

  if (*has_piece && !had_piece_) {
    time_at_last_acquisition_ = monotonic_now;
  }

  // if we've had the piece for enought time, go to lower pump_voltage
  low_pump_voltage =
      *has_piece &&
      monotonic_now > time_at_last_acquisition_ + kTimeAtHigherVoltage;
  no_goal_for_a_bit =
      monotonic_now > time_at_last_evacuate_goal_ + kTimeToKeepPumpRunning;

  if (unsafe_goal && output) {
    const bool evacuate = unsafe_goal->top || unsafe_goal->bottom;
    if (evacuate) {
      time_at_last_evacuate_goal_ = monotonic_now;
    }

    // Once the vacuum evacuates, the pump speeds up because there is no
    // resistance.  So, we want to turn it down to save the pump from
    // overheating.
    output->pump_voltage =
        (no_goal_for_a_bit) ? 0 : (low_pump_voltage ? kPumpHasPieceVoltage
                                                    : kPumpVoltage);

    output->intake_suction_top = unsafe_goal->top;
    output->intake_suction_bottom = unsafe_goal->bottom;
  }
  had_piece_ = *has_piece;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
