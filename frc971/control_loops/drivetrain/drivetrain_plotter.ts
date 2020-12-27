// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {ImuMessageHandler} from 'org_frc971/frc971/wpilib/imu_plot_utils';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;

const kRed = [1, 0, 0];
const kGreen = [0, 1, 0];
const kBlue = [0, 0, 1];
const kBrown = [0.6, 0.3, 0];
const kPink = [1, 0.3, 1];
const kCyan = [0.3, 1, 1];
const kWhite = [1, 1, 1];

export function plotDrivetrain(conn: Connection, element: Element): void {
  const width = 900;
  const height = 400;
  const aosPlotter = new AosPlotter(conn);

  const joystickState = aosPlotter.addMessageSource('/aos', 'aos.JoystickState');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');
  const goal = aosPlotter.addMessageSource('/drivetrain', 'frc971.control_loops.drivetrain.Goal');
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Status');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Output');
  const imu = aosPlotter.addRawMessageSource(
      '/drivetrain', 'frc971.IMUValuesBatch',
      new ImuMessageHandler(conn.getSchema('frc971.IMUValuesBatch')));

  let currentTop = 0;

  // Robot Enabled/Disabled and Mode
  const robotStatePlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height / 2]);
  currentTop += height / 2;
  robotStatePlot.plot.getAxisLabels().setTitle('Robot State');
  robotStatePlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  robotStatePlot.plot.getAxisLabels().setYLabel('bool');
  robotStatePlot.plot.setDefaultYRange([-0.1, 1.1]);

  const testMode = robotStatePlot.addMessageLine(joystickState, ['test_mode']);
  testMode.setColor(kBlue);
  testMode.setPointSize(0.0);
  const autoMode = robotStatePlot.addMessageLine(joystickState, ['autonomous']);
  autoMode.setColor(kRed);
  autoMode.setPointSize(0.0);

  const brownOut = robotStatePlot.addMessageLine(robotState, ['browned_out']);
  brownOut.setColor(kBrown);
  brownOut.setDrawLine(false);
  const enabled = robotStatePlot.addMessageLine(joystickState, ['enabled']);
  enabled.setColor(kGreen);
  enabled.setDrawLine(false);

  // Battery Voltage
  const batteryPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height / 2]);
  currentTop += height / 2;
  batteryPlot.plot.getAxisLabels().setTitle('Battery Voltage');
  batteryPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  batteryPlot.plot.getAxisLabels().setYLabel('Voltage (V)');

  batteryPlot.addMessageLine(robotState, ['voltage_battery']);

  // Polydrivetrain (teleop control) plots
  const teleopPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height / 2]);
  currentTop += height / 2;
  teleopPlot.plot.getAxisLabels().setTitle('Drivetrain Teleop Goals');
  teleopPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  teleopPlot.plot.getAxisLabels().setYLabel('bool, throttle/wheel values');
  teleopPlot.plot.setDefaultYRange([-1.1, 1.1]);

  const quickTurn = teleopPlot.addMessageLine(goal, ['quickturn']);
  quickTurn.setColor(kRed);
  const throttle = teleopPlot.addMessageLine(goal, ['throttle']);
  throttle.setColor(kGreen);
  const wheel = teleopPlot.addMessageLine(goal, ['wheel']);
  wheel.setColor(kBlue);

  // Drivetrain Control Mode
  const modePlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height / 2]);
  currentTop += height / 2;
  // TODO(james): Actually add enum support.
  modePlot.plot.getAxisLabels().setTitle(
      'Drivetrain Mode [POLYDRIVE, MOTION_PROFILE, ' +
      'SPLINE_FOLLOWER, LINE_FOLLOWER]');
  modePlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  modePlot.plot.getAxisLabels().setYLabel('ControllerType');
  modePlot.plot.setDefaultYRange([-0.1, 3.1]);

  const controllerType = modePlot.addMessageLine(goal, ['controller_type']);
  controllerType.setDrawLine(false);

  // Drivetrain Output Voltage
  const outputPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  outputPlot.plot.getAxisLabels().setTitle('Drivetrain Output');
  outputPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  outputPlot.plot.getAxisLabels().setYLabel('Voltage (V)');

  const leftVoltage = outputPlot.addMessageLine(output, ['left_voltage']);
  leftVoltage.setColor(kRed);
  const rightVoltage = outputPlot.addMessageLine(output, ['right_voltage']);
  rightVoltage.setColor(kGreen);

  // Voltage Errors
  const voltageErrors =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  voltageErrors.plot.getAxisLabels().setTitle('Voltage Errors');
  voltageErrors.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  voltageErrors.plot.getAxisLabels().setYLabel('Voltage (V)');

  const leftVoltageError =
      voltageErrors.addMessageLine(status, ['left_voltage_error']);
  leftVoltageError.setColor(kRed);
  const rightVoltageError =
      voltageErrors.addMessageLine(status, ['right_voltage_error']);
  rightVoltageError.setColor(kGreen);

  const ekfLeftVoltageError =
      voltageErrors.addMessageLine(status, ['localizer', 'left_voltage_error']);
  ekfLeftVoltageError.setColor(kPink);
  const ekfRightVoltageError = voltageErrors.addMessageLine(
      status, ['localizer', 'right_voltage_error']);
  ekfRightVoltageError.setColor(kCyan);

  // Sundry components of the output voltages
  const otherVoltages =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  otherVoltages.plot.getAxisLabels().setTitle('Other Voltage Components');
  otherVoltages.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  otherVoltages.plot.getAxisLabels().setYLabel('Voltage (V)');

  const ffLeftVoltage = otherVoltages.addMessageLine(
      status, ['poly_drive_logging', 'ff_left_voltage']);
  ffLeftVoltage.setColor(kRed);
  ffLeftVoltage.setPointSize(0);
  const ffRightVoltage = otherVoltages.addMessageLine(
      status, ['poly_drive_logging', 'ff_right_voltage']);
  ffRightVoltage.setColor(kGreen);
  ffRightVoltage.setPointSize(0);

  const uncappedLeftVoltage =
      otherVoltages.addMessageLine(status, ['uncapped_left_voltage']);
  uncappedLeftVoltage.setColor(kRed);
  uncappedLeftVoltage.setDrawLine(false);
  const uncappedRightVoltage =
      otherVoltages.addMessageLine(status, ['uncapped_right_voltage']);
  uncappedRightVoltage.setColor(kGreen);
  uncappedRightVoltage.setDrawLine(false);

  // Drivetrain Velocities
  const velocityPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  velocityPlot.plot.getAxisLabels().setTitle('Velocity Plots');
  velocityPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  velocityPlot.plot.getAxisLabels().setYLabel('Wheel Velocity (m/s)');

  const ssLeftVelocityGoal =
      velocityPlot.addMessageLine(status, ['profiled_left_velocity_goal']);
  ssLeftVelocityGoal.setColor(kPink);
  ssLeftVelocityGoal.setPointSize(0.0);
  const ssRightVelocityGoal =
      velocityPlot.addMessageLine(status, ['profiled_right_velocity_goal']);
  ssRightVelocityGoal.setColor(kCyan);
  ssRightVelocityGoal.setPointSize(0.0);

  const polyLeftVelocity = velocityPlot.addMessageLine(
      status, ['poly_drive_logging', 'goal_left_velocity']);
  polyLeftVelocity.setColor(kPink);
  polyLeftVelocity.setDrawLine(false);

  const polyRightVelocity = velocityPlot.addMessageLine(
      status, ['poly_drive_logging', 'goal_right_velocity']);
  polyRightVelocity.setColor(kCyan);
  polyRightVelocity.setDrawLine(false);

  const splineLeftVelocity = velocityPlot.addMessageLine(
      status, ['trajectory_logging', 'left_velocity']);
  splineLeftVelocity.setColor(kRed);
  splineLeftVelocity.setDrawLine(false);

  const splineRightVelocity = velocityPlot.addMessageLine(
      status, ['trajectory_logging', 'right_velocity']);
  splineRightVelocity.setColor(kGreen);
  splineRightVelocity.setDrawLine(false);

  const leftVelocity =
      velocityPlot.addMessageLine(status, ['estimated_left_velocity']);
  leftVelocity.setColor(kRed);
  const rightVelocity =
      velocityPlot.addMessageLine(status, ['estimated_right_velocity']);
  rightVelocity.setColor(kGreen);

  const ekfLeftVelocity =
      velocityPlot.addMessageLine(status, ['localizer', 'left_velocity']);
  ekfLeftVelocity.setColor(kRed);
  ekfLeftVelocity.setPointSize(0.0);
  const ekfRightVelocity =
      velocityPlot.addMessageLine(status, ['localizer', 'right_velocity']);
  ekfRightVelocity.setColor(kGreen);
  ekfRightVelocity.setPointSize(0.0);

  // Heading
  const yawPlot = aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  yawPlot.plot.getAxisLabels().setTitle('Robot Yaw');
  yawPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  yawPlot.plot.getAxisLabels().setYLabel('Yaw (rad)');

  const splineYaw =
      yawPlot.addMessageLine(status, ['trajectory_logging', 'theta']);
  splineYaw.setDrawLine(false);
  splineYaw.setColor(kRed);

  const ekfYaw = yawPlot.addMessageLine(status, ['localizer', 'theta']);
  ekfYaw.setColor(kGreen);

  const downEstimatorYaw =
      yawPlot.addMessageLine(status, ['down_estimator', 'yaw']);
  downEstimatorYaw.setColor(kBlue);

  // Pitch/Roll
  const orientationPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  const roll = orientationPlot.addMessageLine(
      status, ['down_estimator', 'lateral_pitch']);
  roll.setColor(kRed);
  roll.setLabel('roll');
  const pitch = orientationPlot.addMessageLine(
      status, ['down_estimator', 'longitudinal_pitch']);
  pitch.setColor(kGreen);
  pitch.setLabel('pitch');

  // Accelerometer/Gravity
  const accelPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  accelPlot.plot.getAxisLabels().setTitle('Accelerometer Readings');
  accelPlot.plot.getAxisLabels().setYLabel('Acceleration (g)');
  accelPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const expectedAccelX =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_x']);
  expectedAccelX.setColor(kRed);
  expectedAccelX.setPointSize(0);
  const expectedAccelY =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_y']);
  expectedAccelY.setColor(kGreen);
  expectedAccelY.setPointSize(0);
  const expectedAccelZ =
      accelPlot.addMessageLine(status, ['down_estimator', 'expected_accel_z']);
  expectedAccelZ.setColor(kBlue);
  expectedAccelZ.setPointSize(0);

  const gravity_magnitude =
      accelPlot.addMessageLine(status, ['down_estimator', 'gravity_magnitude']);
  gravity_magnitude.setColor(kWhite);
  gravity_magnitude.setPointSize(0);

  const accelX = accelPlot.addMessageLine(imu, ['accelerometer_x']);
  accelX.setColor(kRed);
  accelX.setDrawLine(false);
  const accelY = accelPlot.addMessageLine(imu, ['accelerometer_y']);
  accelY.setColor(kGreen);
  accelY.setDrawLine(false);
  const accelZ = accelPlot.addMessageLine(imu, ['accelerometer_z']);
  accelZ.setColor(kBlue);
  accelZ.setDrawLine(false);

  // Absolute X Position
  const xPositionPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  xPositionPlot.plot.getAxisLabels().setTitle('X Position');
  xPositionPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  xPositionPlot.plot.getAxisLabels().setYLabel('X Position (m)');

  const localizerX = xPositionPlot.addMessageLine(status, ['x']);
  localizerX.setColor(kRed);
  const splineX =
      xPositionPlot.addMessageLine(status, ['trajectory_logging', 'x']);
  splineX.setColor(kGreen);

  // Absolute Y Position
  const yPositionPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  yPositionPlot.plot.getAxisLabels().setTitle('Y Position');
  yPositionPlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  yPositionPlot.plot.getAxisLabels().setYLabel('Y Position (m)');

  const localizerY = yPositionPlot.addMessageLine(status, ['y']);
  localizerY.setColor(kRed);
  const splineY =
      yPositionPlot.addMessageLine(status, ['trajectory_logging', 'y']);
  splineY.setColor(kGreen);

  // Gyro
  const gyroPlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height]);
  currentTop += height;
  gyroPlot.plot.getAxisLabels().setTitle('Gyro Readings');
  gyroPlot.plot.getAxisLabels().setYLabel('Angular Velocity (rad / sec)');
  gyroPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const gyroZeroX =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_x_average']);
  gyroZeroX.setColor(kRed);
  gyroZeroX.setPointSize(0);
  gyroZeroX.setLabel('Gyro X Zero');
  const gyroZeroY =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_y_average']);
  gyroZeroY.setColor(kGreen);
  gyroZeroY.setPointSize(0);
  gyroZeroY.setLabel('Gyro Y Zero');
  const gyroZeroZ =
      gyroPlot.addMessageLine(status, ['zeroing', 'gyro_z_average']);
  gyroZeroZ.setColor(kBlue);
  gyroZeroZ.setPointSize(0);
  gyroZeroZ.setLabel('Gyro Z Zero');

  const gyroX = gyroPlot.addMessageLine(imu, ['gyro_x']);
  gyroX.setColor(kRed);
  const gyroY = gyroPlot.addMessageLine(imu, ['gyro_y']);
  gyroY.setColor(kGreen);
  const gyroZ = gyroPlot.addMessageLine(imu, ['gyro_z']);
  gyroZ.setColor(kBlue);

  // IMU States
  const imuStatePlot =
      aosPlotter.addPlot(element, [0, currentTop], [width, height / 2]);
  currentTop += height / 2;
  imuStatePlot.plot.getAxisLabels().setTitle('IMU State');
  imuStatePlot.plot.getAxisLabels().setXLabel('Monotonic Time (sec)');
  imuStatePlot.plot.setDefaultYRange([-0.1, 1.1]);

  const zeroedLine = imuStatePlot.addMessageLine(status, ['zeroing', 'zeroed']);
  zeroedLine.setColor(kRed);
  zeroedLine.setDrawLine(false);
  zeroedLine.setLabel('IMU Zeroed');
  const faultedLine =
      imuStatePlot.addMessageLine(status, ['zeroing', 'faulted']);
  faultedLine.setColor(kGreen);
  faultedLine.setPointSize(0);
  faultedLine.setLabel('IMU Faulted');
}
