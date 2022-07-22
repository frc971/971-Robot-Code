#include "y2022_bot3/actors/auto_splines.h"

#include "frc971/control_loops/control_loops_generated.h"

namespace y2022_bot3 {
namespace actors {

void MaybeFlipSpline(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset,
    bool is_left) {
  flatbuffers::Vector<float> *spline_y =
      GetMutableTemporaryPointer(*builder->fbb(), spline_y_offset);

  if (!is_left) {
    for (size_t i = 0; i < spline_y->size(); i++) {
      spline_y->Mutate(i, -spline_y->Get(i));
    }
  }
}

flatbuffers::Offset<frc971::MultiSpline> FixSpline(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    flatbuffers::Offset<frc971::MultiSpline> spline_offset,
    aos::Alliance alliance) {
  frc971::MultiSpline *spline =
      GetMutableTemporaryPointer(*builder->fbb(), spline_offset);
  flatbuffers::Vector<float> *spline_x = spline->mutable_spline_x();
  flatbuffers::Vector<float> *spline_y = spline->mutable_spline_y();

  if (alliance == aos::Alliance::kBlue) {
    for (size_t ii = 0; ii < spline_x->size(); ++ii) {
      spline_x->Mutate(ii, -spline_x->Get(ii));
    }
    for (size_t ii = 0; ii < spline_y->size(); ++ii) {
      spline_y->Mutate(ii, -spline_y->Get(ii));
    }
  }
  return spline_offset;
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::TestSpline(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  return FixSpline(
      builder,
      aos::CopyFlatBuffer<frc971::MultiSpline>(test_spline_, builder->fbb()),
      alliance);
}

}  // namespace actors
}  // namespace y2022_bot3
