#include "y2019/actors/auto_splines.h"

#include "frc971/control_loops/control_loops_generated.h"

namespace y2019 {
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

// Path off of level 2 to the far side of the rocket with a panel
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HABToFarRocket(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(2.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder lateral_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    lateral_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LATERAL_ACCELERATION);
    lateral_constraint_builder.add_value(2.0);
    lateral_constraint_offset = lateral_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(4.0);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({1.14763818102, 1.66, 3.10, 4.05,
                                           4.45, 5.11, 5.77, 6.71, 7.27, 7.19,
                                           6.57});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({1.30261224364, 1.30217320136, 1.39,
                                           1.47, 1.56346705393, 1.69, 1.81,
                                           1.97, 2.18, 2.84, 3.33});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(2);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from the far side of the rocket to the loading station to pickup
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::FarRocketToHP(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(3.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder lateral_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    lateral_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LATERAL_ACCELERATION);
    lateral_constraint_builder.add_value(2.0);
    lateral_constraint_offset = lateral_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(4.5);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>(
          {6.6, 7.511, 6.332, 4.590, 1.561, 0.179});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>(
          {3.391, 2.826, 1.384, 3.395 - 0.20, 3.429 - 0.20, 3.434 - 0.20});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from the human player station to the far side of the rocket with a panel
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HPToFarRocket(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(3.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder lateral_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    lateral_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LATERAL_ACCELERATION);
    lateral_constraint_builder.add_value(3.0);
    lateral_constraint_offset = lateral_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(4.0);
    velocity_constraint_builder.add_start_distance(7.0);
    velocity_constraint_builder.add_end_distance(15.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({0.895115737979, 2.9155615909,
                                           5.02361983866, 6.40346237218,
                                           7.1260656844, 7.83907559509});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({3.43030859063, 3.44230565037,
                                           2.8824369646, 2.81000389973,
                                           3.08853311072, 2.6933085577});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from the far side of the rocket to close to the loading station
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::FarRocketToNearHP(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  // TODO(theo): Add some real constraints.
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({6.51652191988, 6.83156293562,
                                           5.74513904409, 2.2337653586,
                                           1.94766705864, 0.727526876557});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({3.2465107468, 2.88277456846,
                                           1.93458779243, 3.44064777429,
                                           3.44377880106, 3.43326367284});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from level 2 to 2nd cargo ship bay with a hatch panel
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HABToSecondCargoShipBay(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(2.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(10.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(1.6);
    velocity_constraint_builder.add_start_distance(4.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, voltage_constraint_offset,
               velocity_constraint_offset});

  constexpr double kLess = 0.06;
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>(
          {1.0, 2.53944573074, 5.75526086906, 6.52583747973 - kLess,
           7.12318661548 - kLess, 7.22595029399 - kLess});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({1.5, 1.48, 2.05178220103,
                                           2.56666687655, 1.79340280288,
                                           1.16170693058});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from 2nd cargo ship bay to loading station
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::SecondCargoShipBayToHP(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(4.0);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({7.22595029399, 7.1892447864,
                                           6.5373977907, 5.55997590982,
                                           1.22953437637, 0.32521840905});
  constexpr double kYShift = 0.1;
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>(
          {1.2, 1.44543230529, 2.00646674662, 3.43762336271 - kYShift,
           3.44125430793 - kYShift, 3.4360348159 - kYShift});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from loading station to 3rd cargo ship bay with a hatch panel
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HPToThirdCargoShipBay(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint2_offset;

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(10.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(3.5);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint2_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint2_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint2_builder.add_value(2.0);
    velocity_constraint2_builder.add_start_distance(6.0);
    velocity_constraint2_builder.add_end_distance(10.0);
    velocity_constraint2_offset = velocity_constraint2_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {voltage_constraint_offset, velocity_constraint_offset,
               velocity_constraint2_offset});

  constexpr double kEndMove = 0.25;
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({0.75, 1.112, 5.576, 7.497 - kEndMove,
                                           7.675 - kEndMove, 7.768 - kEndMove});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>(
          {3.431, 3.434, 2.712, 2.874, 1.786, 1.168});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

// Path from 3rd cargo ship bay to near the loading station
flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::ThirdCargoShipBayToNearHP(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(0.5);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({7.75823205276, 7.58356294646,
                                           5.95536035287, 2.12377989323,
                                           1.29347361128, 0.598613577531});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({1.16791407107, 1.94564064915,
                                           2.54565614767, 3.43728005786,
                                           3.43775494434, 3.43119598027});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HabToFarRocketTest(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
    bool is_left) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(2.0);
    longitudinal_constraint_offset = longitudinal_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder lateral_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    lateral_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LATERAL_ACCELERATION);
    lateral_constraint_builder.add_value(2.0);
    lateral_constraint_offset = lateral_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder voltage_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    voltage_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VOLTAGE);
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(1.7);
    velocity_constraint_builder.add_start_distance(0.0);
    velocity_constraint_builder.add_end_distance(0.8);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({1.14763818102, 2.53944573074,
                                           3.74586892131, 5.22352745444,
                                           6.70255737219, 7.35784750785});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({1.30261224364, 1.28295363394,
                                           1.27450357714, 2.89953366429,
                                           3.10734391012, 2.90125929705});
  MaybeFlipSpline(builder, spline_y_offset, is_left);

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::FarRocketToHPTest(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

  {
    frc971::Constraint::Builder longitudinal_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    longitudinal_constraint_builder.add_constraint_type(
        frc971::ConstraintType::LONGITUDINAL_ACCELERATION);
    longitudinal_constraint_builder.add_value(1.5);
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
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(0.5);
    velocity_constraint_builder.add_start_distance(9.5);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({6.53, 7.8, 7.8, 4.0, 2.0, 0.4});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({3.47, 3.0, 1.5, 3.0, 3.4, 3.4});

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
}

flatbuffers::Offset<frc971::MultiSpline> AutonomousSplines::HPToNearRocketTest(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder) {
  flatbuffers::Offset<frc971::Constraint> longitudinal_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> lateral_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> voltage_constraint_offset;
  flatbuffers::Offset<frc971::Constraint> velocity_constraint_offset;

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
    voltage_constraint_builder.add_value(11.0);
    voltage_constraint_offset = voltage_constraint_builder.Finish();
  }

  {
    frc971::Constraint::Builder velocity_constraint_builder =
        builder->MakeBuilder<frc971::Constraint>();
    velocity_constraint_builder.add_constraint_type(
        frc971::ConstraintType::VELOCITY);
    velocity_constraint_builder.add_value(0.5);
    velocity_constraint_builder.add_start_distance(2.7);
    velocity_constraint_builder.add_end_distance(10.0);
    velocity_constraint_offset = velocity_constraint_builder.Finish();
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<frc971::Constraint>>>
      constraints_offset =
          builder->fbb()->CreateVector<flatbuffers::Offset<frc971::Constraint>>(
              {longitudinal_constraint_offset, lateral_constraint_offset,
               voltage_constraint_offset, velocity_constraint_offset});

  flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      builder->fbb()->CreateVector<float>({1.5, 2.0, 3.0, 4.0, 4.5, 5.12});
  flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      builder->fbb()->CreateVector<float>({3.4, 3.4, 3.4, 3.0, 3.0, 3.43});

  frc971::MultiSpline::Builder multispline_builder =
      builder->MakeBuilder<frc971::MultiSpline>();

  multispline_builder.add_spline_count(1);
  multispline_builder.add_constraints(constraints_offset);
  multispline_builder.add_spline_x(spline_x_offset);
  multispline_builder.add_spline_y(spline_y_offset);

  return multispline_builder.Finish();
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
}  // namespace y2019
