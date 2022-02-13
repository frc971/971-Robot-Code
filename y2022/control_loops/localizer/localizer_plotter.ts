// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {ImuMessageHandler} from 'org_frc971/frc971/wpilib/imu_plot_utils';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotLocalizer(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const position = aosPlotter.addMessageSource("/drivetrain",
      "frc971.control_loops.drivetrain.Position");
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Status');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Output');
  const localizer = aosPlotter.addMessageSource(
      '/localizer', 'frc971.controls.LocalizerStatus');
  const imu = aosPlotter.addRawMessageSource(
      '/drivetrain', 'frc971.IMUValuesBatch',
      new ImuMessageHandler(conn.getSchema('frc971.IMUValuesBatch')));

  // Drivetrain Status estimated relative position
  const positionPlot = aosPlotter.addPlot(element);
  positionPlot.plot.getAxisLabels().setTitle("Estimated Relative Position " +
                                             "of the Drivetrain");
  positionPlot.plot.getAxisLabels().setXLabel(TIME);
  positionPlot.plot.getAxisLabels().setYLabel("Relative Position (m)");
  const leftPosition =
      positionPlot.addMessageLine(status, ["estimated_left_position"]);
  leftPosition.setColor(RED);
  const rightPosition =
      positionPlot.addMessageLine(status, ["estimated_right_position"]);
  rightPosition.setColor(GREEN);
  positionPlot
      .addMessageLine(localizer, ['model_based', 'model_state', 'left_encoder'])
      .setColor(BROWN)
      .setPointSize(0.0);
  positionPlot
      .addMessageLine(localizer, ['model_based', 'model_state', 'right_encoder'])
      .setColor(CYAN)
      .setPointSize(0.0);
  positionPlot.addMessageLine(position, ['left_encoder'])
      .setColor(BROWN)
      .setDrawLine(false);
  positionPlot.addMessageLine(position, ['right_encoder'])
      .setColor(CYAN)
      .setDrawLine(false);


  // Drivetrain Velocities
  const velocityPlot = aosPlotter.addPlot(element);
  velocityPlot.plot.getAxisLabels().setTitle('Velocity Plots');
  velocityPlot.plot.getAxisLabels().setXLabel(TIME);
  velocityPlot.plot.getAxisLabels().setYLabel('Wheel Velocity (m/s)');

  const leftVelocity =
      velocityPlot.addMessageLine(status, ['estimated_left_velocity']);
  leftVelocity.setColor(RED);
  const rightVelocity =
      velocityPlot.addMessageLine(status, ['estimated_right_velocity']);
  rightVelocity.setColor(GREEN);

  const leftSpeed = velocityPlot.addMessageLine(position, ["left_speed"]);
  leftSpeed.setColor(BLUE);
  const rightSpeed = velocityPlot.addMessageLine(position, ["right_speed"]);
  rightSpeed.setColor(BROWN);

  const ekfLeftVelocity = velocityPlot.addMessageLine(
      localizer, ['model_based', 'model_state', 'left_velocity']);
  ekfLeftVelocity.setColor(RED);
  ekfLeftVelocity.setPointSize(0.0);
  const ekfRightVelocity = velocityPlot.addMessageLine(
      localizer, ['model_based', 'model_state', 'right_velocity']);
  ekfRightVelocity.setColor(GREEN);
  ekfRightVelocity.setPointSize(0.0);

  velocityPlot
      .addMessageLine(
          localizer, ['model_based', 'oldest_model_state', 'left_velocity'])
      .setColor(RED)
      .setDrawLine(false);

  velocityPlot
      .addMessageLine(
          localizer, ['model_based', 'oldest_model_state', 'right_velocity'])
      .setColor(GREEN)
      .setDrawLine(false);

  const splineVelocityOffset =
      velocityPlot
          .addMessageLine(status, ['localizer', 'longitudinal_velocity_offset'])
          .setColor(BROWN)
          .setPointSize(0.0);

  const splineLateralVelocity =
      velocityPlot.addMessageLine(status, ['localizer', 'lateral_velocity'])
          .setColor(PINK)
          .setPointSize(0.0);

  // Drivetrain Voltage
  const voltagePlot = aosPlotter.addPlot(element);
  voltagePlot.plot.getAxisLabels().setTitle('Voltage Plots');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Voltage (V)')

  voltagePlot.addMessageLine(localizer, ['model_based', 'model_state', 'left_voltage_error'])
      .setColor(RED)
      .setDrawLine(false);
  voltagePlot.addMessageLine(localizer, ['model_based', 'model_state', 'right_voltage_error'])
      .setColor(GREEN)
      .setDrawLine(false);
  voltagePlot.addMessageLine(output, ['left_voltage'])
      .setColor(RED)
      .setPointSize(0);
  voltagePlot.addMessageLine(output, ['right_voltage'])
      .setColor(GREEN)
      .setPointSize(0);

  // Heading
  const yawPlot = aosPlotter.addPlot(element);
  yawPlot.plot.getAxisLabels().setTitle('Robot Yaw');
  yawPlot.plot.getAxisLabels().setXLabel(TIME);
  yawPlot.plot.getAxisLabels().setYLabel('Yaw (rad)');

  yawPlot.addMessageLine(status, ['localizer', 'theta']).setColor(GREEN);

  yawPlot.addMessageLine(status, ['down_estimator', 'yaw']).setColor(BLUE);

  yawPlot.addMessageLine(localizer, ['model_based', 'theta']).setColor(RED);

  // Pitch/Roll
  const orientationPlot = aosPlotter.addPlot(element);
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setXLabel(TIME);
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  orientationPlot.addMessageLine(localizer, ['model_based', 'down_estimator', 'lateral_pitch'])
      .setColor(RED)
      .setLabel('roll');
  orientationPlot
      .addMessageLine(localizer, ['model_based', 'down_estimator', 'longitudinal_pitch'])
      .setColor(GREEN)
      .setLabel('pitch');

  const stillPlot = aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 3]);
  stillPlot.plot.getAxisLabels().setTitle('Still Plot');
  stillPlot.plot.getAxisLabels().setXLabel(TIME);
  stillPlot.plot.getAxisLabels().setYLabel('bool, g\'s');
  stillPlot.plot.setDefaultYRange([-0.1, 1.1]);

  stillPlot.addMessageLine(localizer, ['model_based', 'down_estimator', 'gravity_magnitude'])
      .setColor(WHITE)
      .setDrawLine(false);
  stillPlot.addMessageLine(localizer, ['model_based', 'using_model'])
      .setColor(PINK)
      .setDrawLine(false);

  // Accelerometer/Gravity
  const accelPlot = aosPlotter.addPlot(element);
  accelPlot.plot.getAxisLabels().setTitle('Absoluate Velocities')
  accelPlot.plot.getAxisLabels().setYLabel('Velocity (m/s)');
  accelPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');

  accelPlot.addMessageLine(localizer, ['no_wheel_status', 'velocity_x'])
      .setColor(PINK);
  accelPlot.addMessageLine(localizer, ['no_wheel_status', 'velocity_y'])
      .setColor(GREEN);
  accelPlot.addMessageLine(localizer, ['no_wheel_status', 'velocity_z'])
      .setColor(BLUE);

  accelPlot.addMessageLine(localizer, ['model_based', 'accel_state', 'velocity_x'])
      .setColor(RED)
      .setDrawLine(false);
  accelPlot.addMessageLine(localizer, ['model_based', 'accel_state', 'velocity_y'])
      .setColor(GREEN)
      .setDrawLine(false);

  accelPlot.addMessageLine(localizer, ['model_based', 'oldest_accel_state', 'velocity_x'])
      .setColor(RED)
      .setPointSize(0);
  accelPlot.addMessageLine(localizer, ['model_based', 'oldest_accel_state', 'velocity_y'])
      .setColor(GREEN)
      .setPointSize(0);

  // Absolute X Position
  const xPositionPlot = aosPlotter.addPlot(element);
  xPositionPlot.plot.getAxisLabels().setTitle('X Position');
  xPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  xPositionPlot.plot.getAxisLabels().setYLabel('X Position (m)');

  xPositionPlot.addMessageLine(status, ['x']).setColor(RED);
  xPositionPlot.addMessageLine(status, ['down_estimator', 'position_x'])
      .setColor(BLUE);
  xPositionPlot.addMessageLine(localizer, ['no_wheel_status', 'x']).setColor(GREEN);
  xPositionPlot.addMessageLine(localizer, ['model_based', 'x']).setColor(CYAN);

  xPositionPlot.plot.setDefaultYRange([0.0, 0.5]);

  // Absolute Y Position
  const yPositionPlot = aosPlotter.addPlot(element);
  yPositionPlot.plot.getAxisLabels().setTitle('Y Position');
  yPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  yPositionPlot.plot.getAxisLabels().setYLabel('Y Position (m)');

  const localizerY = yPositionPlot.addMessageLine(status, ['y']);
  localizerY.setColor(RED);
  yPositionPlot.addMessageLine(status, ['down_estimator', 'position_y'])
      .setColor(BLUE);
  yPositionPlot.addMessageLine(localizer, ['no_wheel_status', 'y']).setColor(GREEN);
  yPositionPlot.addMessageLine(localizer, ['model_based', 'y']).setColor(CYAN);

  // Gyro
  const gyroPlot = aosPlotter.addPlot(element);
  gyroPlot.plot.getAxisLabels().setTitle('Gyro Readings');
  gyroPlot.plot.getAxisLabels().setYLabel('Angular Velocity (rad / sec)');
  gyroPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const gyroX = gyroPlot.addMessageLine(imu, ['gyro_x']);
  gyroX.setColor(RED);
  const gyroY = gyroPlot.addMessageLine(imu, ['gyro_y']);
  gyroY.setColor(GREEN);
  const gyroZ = gyroPlot.addMessageLine(imu, ['gyro_z']);
  gyroZ.setColor(BLUE);

  const impliedAccelPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  impliedAccelPlot.plot.getAxisLabels().setTitle('Implied Accelerations');
  impliedAccelPlot.plot.getAxisLabels().setXLabel(TIME);

  impliedAccelPlot.addMessageLine(localizer, ['model_based', 'implied_accel_z'])
      .setColor(BLUE);
  impliedAccelPlot.addMessageLine(localizer, ['model_based', 'implied_accel_y'])
      .setColor(GREEN);
  impliedAccelPlot.addMessageLine(localizer, ['model_based', 'implied_accel_x'])
      .setColor(RED);

  const costPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  costPlot.plot.getAxisLabels().setTitle('Costs');
  costPlot.plot.getAxisLabels().setXLabel(TIME);

  costPlot.addMessageLine(localizer, ['model_based', 'residual'])
      .setColor(RED)
      .setPointSize(0);

  costPlot.addMessageLine(localizer, ['model_based', 'filtered_residual'])
      .setColor(BLUE)
      .setPointSize(0);

  costPlot.addMessageLine(localizer, ['model_based', 'velocity_residual'])
      .setColor(GREEN)
      .setPointSize(0);

  costPlot.addMessageLine(localizer, ['model_based', 'theta_rate_residual'])
      .setColor(BROWN)
      .setPointSize(0);

  costPlot.addMessageLine(localizer, ['model_based', 'accel_residual'])
      .setColor(CYAN)
      .setPointSize(0);
}
