#include "y2022/actors/auto_splines.h"

#include "frc971/control_loops/control_loops_generated.h"

namespace y2022 {
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

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::BasicSSpline(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder) {
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
    voltage_constraint_builder.add_value(6.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset});

  const float startx = 0.4;
  const float starty = 3.4;
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({0.0f + startx, 0.6f + startx,
                                           0.6f + startx, 0.4f + startx,
                                           0.4f + startx, 1.0f + startx});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({starty - 0.0f, starty - 0.0f,
                                           starty - 0.3f, starty - 0.7f,
                                           starty - 1.0f, starty - 1.0f});

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::StraightLine(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder) {
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
}  // namespace y2022
