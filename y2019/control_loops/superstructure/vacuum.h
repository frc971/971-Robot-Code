#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_VACUUM_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_VACUUM_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_output_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

class Vacuum {
 public:
  Vacuum() {}
  void Iterate(const SuctionGoal *unsafe_goal, float suction_pressure,
               OutputT *output, bool *has_piece, aos::EventLoop *event_loop);

  // Voltage to the vaccum pump when we are attempting to acquire a piece
  static constexpr double kPumpVoltage = 8.0;

  // Voltage to the vaccum pump when we have a piece
  static constexpr double kPumpHasPieceVoltage = 2.25;

  // Time to continue at the higher pump voltage after getting a gamepiece
  static constexpr aos::monotonic_clock::duration kTimeAtHigherVoltage =
      std::chrono::milliseconds(100);

  // Time required for the game piece to be released from a vacuum
  static constexpr aos::monotonic_clock::duration kReleaseTime =
      std::chrono::milliseconds(250);

 private:
  bool had_piece_ = false;
  aos::monotonic_clock::time_point last_release_time_ =
      aos::monotonic_clock::epoch();
  // Time since the last time we had a game piece while disabled.
  aos::monotonic_clock::time_point last_disable_has_piece_time_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point time_at_last_acquisition_ =
      aos::monotonic_clock::epoch();
  double filtered_pressure_ = 1.0;

  bool filtered_had_piece_near_disabled_ = false;

  static constexpr double kVacuumOnThreshold = 0.70;
  static constexpr double kVacuumOffThreshold = 0.85;

  static constexpr double kFilterTimeConstant = 0.1;
  static constexpr double dt = .00505;
  static constexpr double kSuctionAlpha =
      dt * (1 - kFilterTimeConstant) / (kFilterTimeConstant);

  DISALLOW_COPY_AND_ASSIGN(Vacuum);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
