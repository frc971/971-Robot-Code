#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_VACUUM_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_VACUUM_H_

#include "y2019/control_loops/superstructure/superstructure.q.h"
#include "aos/controls/control_loop.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

class Vacuum {
 public:
  Vacuum() {}
  void Iterate(const SuctionGoal *unsafe_goal, float suction_pressure,
               SuperstructureQueue::Output *output, bool *has_piece,
               aos::EventLoop *event_loop);


  // Voltage to the vaccum pump when we are attempting to acquire a piece
  static constexpr double kPumpVoltage = 8.0;

  // Voltage to the vaccum pump when we have a piece
  static constexpr double kPumpHasPieceVoltage = 2.0;

  // Time to continue at the higher pump voltage after getting a gamepiece
  static constexpr aos::monotonic_clock::duration kTimeAtHigherVoltage =
      std::chrono::milliseconds(100);

  // Time to continue the pump after getting a no suck goal
  static constexpr aos::monotonic_clock::duration kTimeToKeepPumpRunning =
      std::chrono::milliseconds(750);

 private:
  bool had_piece_ = false;
  aos::monotonic_clock::time_point time_at_last_evacuate_goal_ =
      aos::monotonic_clock::epoch();
  aos::monotonic_clock::time_point time_at_last_acquisition_ =
      aos::monotonic_clock::epoch();
  double filtered_pressure_ = 1.0;

  static constexpr double kVacuumThreshold = 0.70;

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
