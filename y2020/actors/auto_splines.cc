#include "y2020/actors/auto_splines.h"

#include "frc971/control_loops/control_loops_generated.h"

namespace y2020 {
namespace actors {

constexpr double kFieldLength = 16.4592;
constexpr double kFieldWidth = 8.2296;

void MaybeFlipSpline(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
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

  for (size_t ii = 0; ii < spline_x->size(); ++ii) {
    spline_x->Mutate(ii, spline_x->Get(ii) - kFieldLength / 2.0);
  }
  for (size_t ii = 0; ii < spline_y->size(); ++ii) {
    spline_y->Mutate(ii, kFieldWidth / 2.0 - spline_y->Get(ii));
  }

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

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::BasicSSpline(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(1.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder lateral_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    lateral_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LATERAL_ACCELERATION);
    lateral_constraint_builder.add_value(1.0);
    lateral_constraint_offset = lateral_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(2.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset});

  const float startx = 0.0;
  const float starty = 0.0;
  std::vector<float> x_pos{0.0f + startx, 0.4f + startx, 0.4f + startx,
                           0.6f + startx, 0.6f + startx, 1.0f + startx};
  std::vector<float> y_pos{starty + 0.0f, starty + 0.0f,  starty + 0.05f,
                           starty + 0.1f, starty + 0.15f, starty + 0.15f};
  if (alliance == aos::Alliance::kRed) {
    for (size_t ii = 0; ii < x_pos.size(); ++ii) {
      x_pos[ii] *= -1;
      y_pos[ii] *= -1;
    }
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>(x_pos);
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>(y_pos);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::TargetAligned1(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  return FixSpline(builder,
                   aos::CopyFlatBuffer<frc971::MultiSpline>(target_aligned_1_,
                                                            builder->fbb()),
                   alliance);
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::TargetAligned2(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  return FixSpline(builder,
                   aos::CopyFlatBuffer<frc971::MultiSpline>(target_aligned_2_,
                                                            builder->fbb()),
                   alliance);
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::TargetAligned3(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  return FixSpline(builder,
                   aos::CopyFlatBuffer<frc971::MultiSpline>(target_aligned_3_,
                                                            builder->fbb()),
                   alliance);
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::FarSideFender(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder,
    aos::Alliance alliance) {
  // I drew the spline on the wrong side of the field.
  if (alliance == aos::Alliance::kBlue) {
    alliance = aos::Alliance::kRed;
  } else {
    alliance = aos::Alliance::kBlue;
  }
  return FixSpline(builder,
                   aos::CopyFlatBuffer<frc971::MultiSpline>(far_side_fender_,
                                                            builder->fbb()),
                   alliance);
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::StraightLine(
    aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
        *builder) {
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>(
          {-12.3, -11.9, -11.5, -11.1, -10.6, -10.0});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({1.25, 1.25, 1.25, 1.25, 1.25, 1.25});

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

}  // namespace actors
}  // namespace y2020
