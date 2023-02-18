// Provides a basic plot for debugging IMU-related issues on a robot.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import {ImuMessageHandler} from '../../../frc971/wpilib/imu_plot_utils';
import * as proxy from '../../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';

import Connection = proxy.Connection;

export function plotDownEstimator(conn: Connection, element: Element): void {
  const width = 900;
  const height = 400;
  const aosPlotter = new AosPlotter(conn);

  const status = aosPlotter.addMessageSource(
      '/localizer', 'frc971.controls.LocalizerStatus');

  const imu = aosPlotter.addRawMessageSource(
      '/localizer', 'frc971.IMUValuesBatch',
      new ImuMessageHandler(conn.getSchema('frc971.IMUValuesBatch')));

  const accelPlot = aosPlotter.addPlot(element, [width, height]);
  accelPlot.plot.getAxisLabels().setTitle(
      'Estimated Accelerations (x = forward, y = lateral, z = vertical)');
  accelPlot.plot.getAxisLabels().setYLabel('Acceleration (m/s/s)');
  accelPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const accelX = accelPlot.addMessageLine(status, ['model_based', 'down_estimator', 'accel_x']);
  accelX.setColor(RED);
  const accelY = accelPlot.addMessageLine(status, ['model_based', 'down_estimator', 'accel_y']);
  accelY.setColor(GREEN);
  const accelZ = accelPlot.addMessageLine(status, ['model_based', 'down_estimator', 'accel_z']);
  accelZ.setColor(BLUE);

  const velPlot = aosPlotter.addPlot(element, [width, height]);
  velPlot.plot.getAxisLabels().setTitle('Raw IMU Integrated Velocity');
  velPlot.plot.getAxisLabels().setYLabel('Velocity (m/s)');
  velPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const velX = velPlot.addMessageLine(status, ['model_based', 'down_estimator', 'velocity_x']);
  velX.setColor(RED);
  const velY = velPlot.addMessageLine(status, ['model_based', 'down_estimator', 'velocity_y']);
  velY.setColor(GREEN);
  const velZ = velPlot.addMessageLine(status, ['model_based', 'down_estimator', 'velocity_z']);
  velZ.setColor(BLUE);

  const gravityPlot = aosPlotter.addPlot(element, [width, height]);
  gravityPlot.plot.getAxisLabels().setTitle('Accelerometer Magnitudes');
  gravityPlot.plot.getAxisLabels().setXLabel('Monotonic Sent Time (sec)');
  gravityPlot.plot.setDefaultYRange([0.95, 1.05]);
  const gravityLine =
      gravityPlot.addMessageLine(status, ['model_based', 'down_estimator', 'gravity_magnitude']);
  gravityLine.setColor(RED);
  gravityLine.setDrawLine(false);
  const accelMagnitudeLine =
      gravityPlot.addMessageLine(imu, ['acceleration_magnitude_filtered']);
  accelMagnitudeLine.setColor(BLUE);
  accelMagnitudeLine.setLabel('Filtered Accel Magnitude (0.1 sec)');
  accelMagnitudeLine.setDrawLine(false);

  const orientationPlot =
      aosPlotter.addPlot(element, [width, height]);
  orientationPlot.plot.getAxisLabels().setTitle('Orientation');
  orientationPlot.plot.getAxisLabels().setYLabel('Angle (rad)');

  const roll = orientationPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'lateral_pitch']);
  roll.setColor(RED);
  roll.setLabel('roll');
  const pitch = orientationPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'longitudinal_pitch']);
  pitch.setColor(GREEN);
  pitch.setLabel('pitch');
  const yaw = orientationPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'yaw']);
  yaw.setColor(BLUE);
  yaw.setLabel('yaw');

  const imuAccelPlot = aosPlotter.addPlot(element, [width, height]);
  imuAccelPlot.plot.getAxisLabels().setTitle('Filtered Accelerometer Readings');
  imuAccelPlot.plot.getAxisLabels().setYLabel('Acceleration (g)');
  imuAccelPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const imuAccelX = imuAccelPlot.addMessageLine(imu, ['accelerometer_x']);
  imuAccelX.setColor(RED);
  imuAccelX.setDrawLine(false);
  const imuAccelY = imuAccelPlot.addMessageLine(imu, ['accelerometer_y']);
  imuAccelY.setColor(GREEN);
  imuAccelY.setDrawLine(false);
  const imuAccelZ = imuAccelPlot.addMessageLine(imu, ['accelerometer_z']);
  imuAccelZ.setColor(BLUE);
  imuAccelZ.setDrawLine(false);

  const imuAccelXFiltered = imuAccelPlot.addMessageLine(imu, ['accelerometer_x_filtered']);
  imuAccelXFiltered.setColor(RED);
  imuAccelXFiltered.setPointSize(0);
  const imuAccelYFiltered = imuAccelPlot.addMessageLine(imu, ['accelerometer_y_filtered']);
  imuAccelYFiltered.setColor(GREEN);
  imuAccelYFiltered.setPointSize(0);
  const imuAccelZFiltered = imuAccelPlot.addMessageLine(imu, ['accelerometer_z_filtered']);
  imuAccelZFiltered.setColor(BLUE);
  imuAccelZFiltered.setPointSize(0);

  const expectedAccelX = imuAccelPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'expected_accel_x']);
  expectedAccelX.setColor(RED);
  expectedAccelX.setPointSize(0);
  const expectedAccelY = imuAccelPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'expected_accel_y']);
  expectedAccelY.setColor(GREEN);
  expectedAccelY.setPointSize(0);
  const expectedAccelZ = imuAccelPlot.addMessageLine(
      status, ['model_based', 'down_estimator', 'expected_accel_z']);
  expectedAccelZ.setColor(BLUE);
  expectedAccelZ.setPointSize(0);

  const gyroPlot = aosPlotter.addPlot(element, [width, height]);
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

  const statePlot = aosPlotter.addPlot(element, [width, height / 2]);
  statePlot.plot.getAxisLabels().setTitle('IMU State');
  statePlot.plot.getAxisLabels().setXLabel('Monotonic Sent Time (sec)');

  const zeroedLine =
      statePlot.addMessageLine(status, ['zeroing', 'zeroed']);
  zeroedLine.setColor(RED);
  zeroedLine.setDrawLine(false);
  const consecutiveStill =
      statePlot.addMessageLine(status, ['model_based', 'down_estimator', 'consecutive_still']);
  consecutiveStill.setColor(BLUE);
  consecutiveStill.setPointSize(0);
  const faultedLine =
  statePlot.addMessageLine(status, ['zeroing', 'faulted']);
  faultedLine.setColor(GREEN);
  faultedLine.setPointSize(0);
}
