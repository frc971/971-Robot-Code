// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from '../../aos/network/www/aos_plotter';
import {ImuMessageHandler} from '../../frc971/wpilib/imu_plot_utils';
import * as proxy from '../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../aos/network/www/colors';

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
      '/localizer', 'y2023.localizer.Status');
  const imu = aosPlotter.addRawMessageSource(
      '/localizer', 'frc971.IMUValuesBatch',
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
      .addMessageLine(localizer, ['imu', 'left_encoder'])
      .setColor(BROWN)
      .setPointSize(0.0);
  positionPlot
      .addMessageLine(localizer, ['imu', 'right_encoder'])
      .setColor(CYAN)
      .setPointSize(0.0);
  positionPlot.addMessageLine(position, ['left_encoder'])
      .setColor(BROWN)
      .setDrawLine(false);
  positionPlot.addMessageLine(imu, ['left_encoder'])
      .setColor(BROWN)
      .setDrawLine(false);
  positionPlot.addMessageLine(position, ['right_encoder'])
      .setColor(CYAN)
      .setDrawLine(false);
  positionPlot.addMessageLine(imu, ['right_encoder'])
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
      localizer, ['state', 'left_velocity']);
  ekfLeftVelocity.setColor(RED);
  ekfLeftVelocity.setPointSize(0.0);
  const ekfRightVelocity = velocityPlot.addMessageLine(
      localizer, ['state', 'right_velocity']);
  ekfRightVelocity.setColor(GREEN);
  ekfRightVelocity.setPointSize(0.0);

  // Lateral velocity
  const lateralPlot = aosPlotter.addPlot(element);
  lateralPlot.plot.getAxisLabels().setTitle('Lateral Velocity');
  lateralPlot.plot.getAxisLabels().setXLabel(TIME);
  lateralPlot.plot.getAxisLabels().setYLabel('Velocity (m/s)');

  lateralPlot.addMessageLine(localizer, ['state', 'lateral_velocity']).setColor(CYAN);

  // Drivetrain Voltage
  const voltagePlot = aosPlotter.addPlot(element);
  voltagePlot.plot.getAxisLabels().setTitle('Voltage Plots');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Voltage (V)')

  voltagePlot.addMessageLine(localizer, ['state', 'left_voltage_error'])
      .setColor(RED)
      .setDrawLine(false);
  voltagePlot.addMessageLine(localizer, ['state', 'right_voltage_error'])
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

  yawPlot.addMessageLine(localizer, ['down_estimator', 'yaw']).setColor(BLUE);

  yawPlot.addMessageLine(localizer, ['state', 'theta']).setColor(RED);

  // Pitch/Roll
  const orientationPlot = aosPlotter.addPlot(element);
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setXLabel(TIME);
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  orientationPlot.addMessageLine(localizer, ['down_estimator', 'lateral_pitch'])
      .setColor(RED)
      .setLabel('roll');
  orientationPlot
      .addMessageLine(localizer, ['down_estimator', 'longitudinal_pitch'])
      .setColor(GREEN)
      .setLabel('pitch');

  const stillPlot = aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 3]);
  stillPlot.plot.getAxisLabels().setTitle('Still Plot');
  stillPlot.plot.getAxisLabels().setXLabel(TIME);
  stillPlot.plot.getAxisLabels().setYLabel('bool, g\'s');
  stillPlot.plot.setDefaultYRange([-0.1, 1.1]);

  stillPlot.addMessageLine(localizer, ['down_estimator', 'gravity_magnitude'])
      .setColor(WHITE)
      .setDrawLine(false);

  // Absolute X Position
  const xPositionPlot = aosPlotter.addPlot(element);
  xPositionPlot.plot.getAxisLabels().setTitle('X Position');
  xPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  xPositionPlot.plot.getAxisLabels().setYLabel('X Position (m)');

  xPositionPlot.addMessageLine(status, ['x']).setColor(RED);
  xPositionPlot.addMessageLine(localizer, ['down_estimator', 'position_x'])
      .setColor(BLUE);
  xPositionPlot.addMessageLine(localizer, ['state', 'x']).setColor(CYAN);

  // Absolute Y Position
  const yPositionPlot = aosPlotter.addPlot(element);
  yPositionPlot.plot.getAxisLabels().setTitle('Y Position');
  yPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  yPositionPlot.plot.getAxisLabels().setYLabel('Y Position (m)');

  const localizerY = yPositionPlot.addMessageLine(status, ['y']);
  localizerY.setColor(RED);
  yPositionPlot.addMessageLine(localizer, ['down_estimator', 'position_y'])
      .setColor(BLUE);
  yPositionPlot.addMessageLine(localizer, ['state', 'y']).setColor(CYAN);

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


  const timingPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  timingPlot.plot.getAxisLabels().setTitle('Fault Counting');
  timingPlot.plot.getAxisLabels().setXLabel(TIME);

  timingPlot
      .addMessageLine(
          localizer, ['imu', 'imu_failures', 'imu_to_pico_checksum_mismatch'])
      .setColor(BLUE)
      .setDrawLine(false);

  timingPlot
      .addMessageLine(
          localizer, ['imu', 'imu_failures', 'pico_to_pi_checksum_mismatch'])
      .setColor(RED)
      .setDrawLine(false);

  timingPlot
      .addMessageLine(
          localizer, ['imu', 'imu_failures', 'other_zeroing_faults'])
      .setColor(CYAN)
      .setDrawLine(false);

  timingPlot
      .addMessageLine(
          localizer, ['imu', 'imu_failures', 'missed_messages'])
      .setColor(PINK)
      .setDrawLine(false);
}
