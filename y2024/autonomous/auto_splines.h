#ifndef Y2024_AUTONOMOUS_AUTO_SPLINES_H_
#define Y2024_AUTONOMOUS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/input/joystick_state_generated.h"
/*

  The cooridinate system for the autonomous splines is the same as the spline
  python generator and drivetrain spline systems.

*/

namespace y2024::autonomous {

class AutonomousSplines {
 public:
  AutonomousSplines()
      : test_spline_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/test_spline.json")),
        mobility_and_shoot_spline_(
            aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
                "splines/mobilityandshoot.0.json")),
        four_piece_spline_1_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/five_note.0.json")),
        four_piece_spline_2_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/five_note.1.json")),
        four_piece_spline_3_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/five_note.2.json")),
        four_piece_spline_4_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/five_note.3.json")),
        four_piece_spline_5_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/five_note.4.json")),
        two_piece_steal_spline_1_(
            aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
                "splines/five_note.0.json")),
        two_piece_steal_spline_2_(
            aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
                "splines/five_note.1.json")),
        two_piece_steal_spline_3_(
            aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
                "splines/five_note.2.json")),
        two_piece_steal_spline_4_(
            aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
                "splines/five_note.3.json")) {}
  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);

  flatbuffers::Offset<frc971::MultiSpline> TestSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> MobilityAndShootSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> FourPieceSpline1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> FourPieceSpline2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> FourPieceSpline3(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> FourPieceSpline4(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> FourPieceSpline5(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);

  flatbuffers::Offset<frc971::MultiSpline> TwoPieceStealSpline1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> TwoPieceStealSpline2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> TwoPieceStealSpline3(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> TwoPieceStealSpline4(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);

 private:
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> test_spline_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> mobility_and_shoot_spline_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> four_piece_spline_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> four_piece_spline_2_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> four_piece_spline_3_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> four_piece_spline_4_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> four_piece_spline_5_;

  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> two_piece_steal_spline_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> two_piece_steal_spline_2_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> two_piece_steal_spline_3_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> two_piece_steal_spline_4_;
};

}  // namespace y2024::autonomous

#endif  // Y2024_AUTONOMOUS_AUTO_SPLINES_H_
