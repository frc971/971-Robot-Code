// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import {ImuMessageHandler} from '../../../frc971/wpilib/imu_plot_utils';
import * as proxy from '../../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotDrivetrain(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const goal = aosPlotter.addMessageSource('/drivetrain', 'frc971.control_loops.drivetrain.Goal');
  const position = aosPlotter.addMessageSource("/drivetrain",
      "frc971.control_loops.drivetrain.Position");
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Status');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Output');
  const gyroReading = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.sensors.GyroReading');
  const imu = aosPlotter.addRawMessageSource(
      '/drivetrain', 'frc971.IMUValuesBatch',
      new ImuMessageHandler(conn.getSchema('frc971.IMUValuesBatch')));

  // Polydrivetrain (teleop control) plots
  const teleopPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  teleopPlot.plot.getAxisLabels().setTitle('Drivetrain Teleop Goals');
  teleopPlot.plot.getAxisLabels().setXLabel(TIME);
  teleopPlot.plot.getAxisLabels().setYLabel('bool, throttle/wheel values');
  teleopPlot.plot.setDefaultYRange([-1.1, 1.1]);

  const quickTurn = teleopPlot.addMessageLine(goal, ['quickturn']);
  quickTurn.setColor(RED);
  const throttle = teleopPlot.addMessageLine(goal, ['throttle']);
  throttle.setColor(GREEN);
  const wheel = teleopPlot.addMessageLine(goal, ['wheel']);
  wheel.setColor(BLUE);

  // Drivetrain Control Mode
  const modePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  // TODO(james): Actually add enum support.
  modePlot.plot.getAxisLabels().setTitle(
      'Drivetrain Mode [POLYDRIVE, MOTION_PROFILE, ' +
      'SPLINE_FOLLOWER, LINE_FOLLOWER]');
  modePlot.plot.getAxisLabels().setXLabel(TIME);
  modePlot.plot.getAxisLabels().setYLabel('ControllerType');
  modePlot.plot.setDefaultYRange([-0.1, 3.1]);

  const controllerType = modePlot.addMessageLine(goal, ['controller_type']);
  controllerType.setDrawLine(false);

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
  const leftPositionGoal =
      positionPlot.addMessageLine(status, ["profiled_left_position_goal"]);
  leftPositionGoal.setColor(BLUE);
  const rightPositionGoal =
      positionPlot.addMessageLine(status, ["profiled_right_position_goal"]);
  rightPositionGoal.setColor(PINK);
  const leftEncoder = positionPlot.addMessageLine(position, ["left_encoder"]);
  leftEncoder.setColor(BROWN);
  const rightEncoder = positionPlot.addMessageLine(position, ["right_encoder"]);
  rightEncoder.setColor(CYAN);

  // Drivetrain Output Voltage
  const outputPlot = aosPlotter.addPlot(element);
  outputPlot.plot.getAxisLabels().setTitle('Drivetrain Output');
  outputPlot.plot.getAxisLabels().setXLabel(TIME);
  outputPlot.plot.getAxisLabels().setYLabel('Voltage (V)');

  const leftVoltage = outputPlot.addMessageLine(output, ['left_voltage']);
  leftVoltage.setColor(RED);
  const rightVoltage = outputPlot.addMessageLine(output, ['right_voltage']);
  rightVoltage.setColor(GREEN);

  // Voltage Errors
  const voltageErrors = aosPlotter.addPlot(element);
  voltageErrors.plot.getAxisLabels().setTitle('Voltage Errors');
  voltageErrors.plot.getAxisLabels().setXLabel(TIME);
  voltageErrors.plot.getAxisLabels().setYLabel('Voltage (V)');

  const leftVoltageError =
      voltageErrors.addMessageLine(status, ['left_voltage_error']);
  leftVoltageError.setColor(RED);
  const rightVoltageError =
      voltageErrors.addMessageLine(status, ['right_voltage_error']);
  rightVoltageError.setColor(GREEN);

  const ekfLeftVoltageError =
      voltageErrors.addMessageLine(status, ['localizer', 'left_voltage_error']);
  ekfLeftVoltageError.setColor(PINK);
  const ekfRightVoltageError = voltageErrors.addMessageLine(
      status, ['localizer', 'right_voltage_error']);
  ekfRightVoltageError.setColor(CYAN);

  // Sundry components of the output voltages
  const otherVoltages = aosPlotter.addPlot(element);
  otherVoltages.plot.getAxisLabels().setTitle('Other Voltage Components');
  otherVoltages.plot.getAxisLabels().setXLabel(TIME);
  otherVoltages.plot.getAxisLabels().setYLabel('Voltage (V)');

  const ffLeftVoltage = otherVoltages.addMessageLine(
      status, ['poly_drive_logging', 'ff_left_voltage']);
  ffLeftVoltage.setColor(RED);
  ffLeftVoltage.setPointSize(0);
  const ffRightVoltage = otherVoltages.addMessageLine(
      status, ['poly_drive_logging', 'ff_right_voltage']);
  ffRightVoltage.setColor(GREEN);
  ffRightVoltage.setPointSize(0);

  const uncappedLeftVoltage =
      otherVoltages.addMessageLine(status, ['uncapped_left_voltage']);
  uncappedLeftVoltage.setColor(RED);
  uncappedLeftVoltage.setDrawLine(false);
  const uncappedRightVoltage =
      otherVoltages.addMessageLine(status, ['uncapped_right_voltage']);
  uncappedRightVoltage.setColor(GREEN);
  uncappedRightVoltage.setDrawLine(false);

  // Drivetrain Velocities
  const velocityPlot = aosPlotter.addPlot(element);
  velocityPlot.plot.getAxisLabels().setTitle('Velocity Plots');
  velocityPlot.plot.getAxisLabels().setXLabel(TIME);
  velocityPlot.plot.getAxisLabels().setYLabel('Wheel Velocity (m/s)');

  const ssLeftVelocityGoal =
      velocityPlot.addMessageLine(status, ['profiled_left_velocity_goal']);
  ssLeftVelocityGoal.setColor(PINK);
  ssLeftVelocityGoal.setPointSize(0.0);
  const ssRightVelocityGoal =
      velocityPlot.addMessageLine(status, ['profiled_right_velocity_goal']);
  ssRightVelocityGoal.setColor(CYAN);
  ssRightVelocityGoal.setPointSize(0.0);

  const polyLeftVelocity = velocityPlot.addMessageLine(
      status, ['poly_drive_logging', 'goal_left_velocity']);
  polyLeftVelocity.setColor(PINK);
  polyLeftVelocity.setDrawLine(false);

  const polyRightVelocity = velocityPlot.addMessageLine(
      status, ['poly_drive_logging', 'goal_right_velocity']);
  polyRightVelocity.setColor(CYAN);
  polyRightVelocity.setDrawLine(false);

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

  // Drivetrain trajectory and localizer velocities
  const velocityPlot2 = aosPlotter.addPlot(element);
  velocityPlot2.plot.getAxisLabels().setTitle(
      "Trajectory and Localizer Velocity Plots");
  velocityPlot2.plot.getAxisLabels().setXLabel(TIME);
  velocityPlot2.plot.getAxisLabels().setYLabel('Wheel Velocity (m/s)');

  const splineLeftVelocity = velocityPlot2.addMessageLine(
      status, ['trajectory_logging', 'left_velocity']);
  splineLeftVelocity.setColor(RED);
  splineLeftVelocity.setDrawLine(false);

  const splineRightVelocity = velocityPlot2.addMessageLine(
      status, ['trajectory_logging', 'right_velocity']);
  splineRightVelocity.setColor(GREEN);
  splineRightVelocity.setDrawLine(false);

  const ekfLeftVelocity =
      velocityPlot2.addMessageLine(status, ['localizer', 'left_velocity']);
  ekfLeftVelocity.setColor(RED);
  ekfLeftVelocity.setPointSize(0.0);
  const ekfRightVelocity =
      velocityPlot2.addMessageLine(status, ['localizer', 'right_velocity']);
  ekfRightVelocity.setColor(GREEN);
  ekfRightVelocity.setPointSize(0.0);

  const splineVelocityOffset = velocityPlot2.addMessageLine(
      status, ['localizer', 'longitudinal_velocity_offset']);
  splineVelocityOffset.setColor(BROWN);
  splineVelocityOffset.setPointSize(0.0);

  const splineLateralVelocity = velocityPlot2.addMessageLine(
      status, ['localizer', 'lateral_velocity']);
  splineLateralVelocity.setColor(PINK);
  splineLateralVelocity.setPointSize(0.0);

  // Heading
  const yawPlot = aosPlotter.addPlot(element);
  yawPlot.plot.getAxisLabels().setTitle('Robot Yaw');
  yawPlot.plot.getAxisLabels().setXLabel(TIME);
  yawPlot.plot.getAxisLabels().setYLabel('Yaw (rad)');

  const splineYaw =
      yawPlot.addMessageLine(status, ['trajectory_logging', 'theta']);
  splineYaw.setDrawLine(false);
  splineYaw.setColor(RED);

  const ekfYaw = yawPlot.addMessageLine(status, ['localizer', 'theta']);
  ekfYaw.setColor(GREEN);

  const downEstimatorYaw =
      yawPlot.addMessageLine(status, ['down_estimator', 'yaw']);
  downEstimatorYaw.setColor(BLUE);

  // Pitch/Roll
  const orientationPlot = aosPlotter.addPlot(element);
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setXLabel(TIME);
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  const roll = orientationPlot.addMessageLine(
      status, ['down_estimator', 'lateral_pitch']);
  roll.setColor(RED);
  roll.setLabel('roll');
  const pitch = orientationPlot.addMessageLine(
      status, ['down_estimator', 'longitudinal_pitch']);
  pitch.setColor(GREEN);
  pitch.setLabel('pitch');

  // Accelerometer/Gravity
  const accelPlot = aosPlotter.addPlot(element);
  accelPlot.plot.getAxisLabels().setTitle('Accelerometer Readings');
  accelPlot.plot.getAxisLabels().setYLabel('Acceleration (g)');
  accelPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const expectedAccelX =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_x']);
  expectedAccelX.setColor(RED);
  expectedAccelX.setPointSize(0);
  const expectedAccelY =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_y']);
  expectedAccelY.setColor(GREEN);
  expectedAccelY.setPointSize(0);
  const expectedAccelZ =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_z']);
  expectedAccelZ.setColor(BLUE);
  expectedAccelZ.setPointSize(0);

  const gravity_magnitude =
      accelPlot.addMessageLine(status, ['down_estimator', 'gravity_magnitude']);
  gravity_magnitude.setColor(WHITE);
  gravity_magnitude.setPointSize(0);

  const accelX = accelPlot.addMessageLine(imu, ['accelerometer_x']);
  accelX.setColor(RED);
  accelX.setDrawLine(false);
  const accelY = accelPlot.addMessageLine(imu, ['accelerometer_y']);
  accelY.setColor(GREEN);
  accelY.setDrawLine(false);
  const accelZ = accelPlot.addMessageLine(imu, ['accelerometer_z']);
  accelZ.setColor(BLUE);
  accelZ.setDrawLine(false);

  // Absolute X Position
  const xPositionPlot = aosPlotter.addPlot(element);
  xPositionPlot.plot.getAxisLabels().setTitle('X Position');
  xPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  xPositionPlot.plot.getAxisLabels().setYLabel('X Position (m)');

  const localizerX = xPositionPlot.addMessageLine(status, ['x']);
  localizerX.setColor(RED);
  const splineX =
      xPositionPlot.addMessageLine(status, ['trajectory_logging', 'x']);
  splineX.setColor(GREEN);

  // Absolute Y Position
  const yPositionPlot = aosPlotter.addPlot(element);
  yPositionPlot.plot.getAxisLabels().setTitle('Y Position');
  yPositionPlot.plot.getAxisLabels().setXLabel(TIME);
  yPositionPlot.plot.getAxisLabels().setYLabel('Y Position (m)');

  const localizerY = yPositionPlot.addMessageLine(status, ['y']);
  localizerY.setColor(RED);
  const splineY =
      yPositionPlot.addMessageLine(status, ['trajectory_logging', 'y']);
  splineY.setColor(GREEN);

  // Gyro
  const gyroPlot = aosPlotter.addPlot(element);
  gyroPlot.plot.getAxisLabels().setTitle('Gyro Readings');
  gyroPlot.plot.getAxisLabels().setYLabel('Angular Velocity (rad / sec)');
  gyroPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const gyroZeroX =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_x_average']);
  gyroZeroX.setColor(RED);
  gyroZeroX.setPointSize(0);
  gyroZeroX.setLabel('Gyro X Zero');
  const gyroZeroY =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_y_average']);
  gyroZeroY.setColor(GREEN);
  gyroZeroY.setPointSize(0);
  gyroZeroY.setLabel('Gyro Y Zero');
  const gyroZeroZ =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_z_average']);
  gyroZeroZ.setColor(BLUE);
  gyroZeroZ.setPointSize(0);
  gyroZeroZ.setLabel('Gyro Z Zero');

  const gyroX = gyroPlot.addMessageLine(imu, ['gyro_x']);
  gyroX.setColor(RED);
  const gyroY = gyroPlot.addMessageLine(imu, ['gyro_y']);
  gyroY.setColor(GREEN);
  const gyroZ = gyroPlot.addMessageLine(imu, ['gyro_z']);
  gyroZ.setColor(BLUE);
  gyroPlot.addMessageLine(gyroReading, ['velocity']).setColor(BLUE);

  // IMU States
  const imuStatePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  imuStatePlot.plot.getAxisLabels().setTitle('IMU State');
  imuStatePlot.plot.getAxisLabels().setXLabel(TIME);
  imuStatePlot.plot.setDefaultYRange([-0.1, 1.1]);

  const zeroedLine = imuStatePlot.addMessageLine(status, ['zeroing', 'zeroed']);
  zeroedLine.setColor(RED);
  zeroedLine.setDrawLine(false);
  zeroedLine.setLabel('IMU Zeroed');
  const faultedLine =
      imuStatePlot.addMessageLine(status, ['zeroing', 'faulted']);
  faultedLine.setColor(GREEN);
  faultedLine.setPointSize(0);
  faultedLine.setLabel('IMU Faulted');
}
