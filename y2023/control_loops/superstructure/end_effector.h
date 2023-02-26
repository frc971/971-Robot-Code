#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {

class EndEffector {
 public:
  static constexpr double kRollerConeSuckVoltage() { return 12.0; }
  static constexpr double kRollerCubeSuckVoltage() { return -5.0; }
  static constexpr double kRollerSpitVoltage() { return -9.0; }

  EndEffector();
  void RunIteration(const ::aos::monotonic_clock::time_point timestamp,
                    RollerGoal roller_goal, double falcon_current,
                    double cone_position, bool beambreak,
                    double *intake_roller_voltage);
  EndEffectorState state() const { return state_; }
  GamePiece game_piece() const { return game_piece_; }
  void Reset();

 private:
  EndEffectorState state_;
  GamePiece game_piece_;

  aos::monotonic_clock::time_point timer_;

  bool beambreak_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
