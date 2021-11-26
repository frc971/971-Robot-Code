// Provides a basic plot for debugging IMU-related issues on a robot.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {ImuMessageHandler} from 'org_frc971/frc971/wpilib/imu_plot_utils';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;

export function plotImu(conn: Connection, element: Element): void {
  const width = 900;
  const height = 400;
  const aosPlotter = new AosPlotter(conn);

  const accelPlot = aosPlotter.addPlot(element, [width, height]);
  accelPlot.plot.getAxisLabels().setTitle('Accelerometer Readings');
  accelPlot.plot.getAxisLabels().setYLabel('Acceleration (g)');
  accelPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const drivetrainStatus = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Status');

  const imu = aosPlotter.addRawMessageSource(
      '/drivetrain', 'frc971.IMUValuesBatch',
      new ImuMessageHandler(conn.getSchema('frc971.IMUValuesBatch')));

  const accelX = accelPlot.addMessageLine(imu, ['accelerometer_x']);
  accelX.setColor([1, 0, 0]);
  const accelY = accelPlot.addMessageLine(imu, ['accelerometer_y']);
  accelY.setColor([0, 1, 0]);
  const accelZ = accelPlot.addMessageLine(imu, ['accelerometer_z']);
  accelZ.setColor([0, 0, 1]);

  const gyroPlot = aosPlotter.addPlot(element, [width, height]);
  gyroPlot.plot.getAxisLabels().setTitle('Gyro Readings');
  gyroPlot.plot.getAxisLabels().setYLabel('Angular Velocity (rad / sec)');
  gyroPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  const gyroZeroX =
      gyroPlot.addMessageLine(drivetrainStatus, ['zeroing', 'gyro_x_average']);
  gyroZeroX.setColor([1, 0, 0]);
  gyroZeroX.setPointSize(0);
  gyroZeroX.setLabel('Gyro X Zero');
  const gyroZeroY =
      gyroPlot.addMessageLine(drivetrainStatus, ['zeroing', 'gyro_y_average']);
  gyroZeroY.setColor([0, 1, 0]);
  gyroZeroY.setPointSize(0);
  gyroZeroY.setLabel('Gyro Y Zero');
  const gyroZeroZ =
      gyroPlot.addMessageLine(drivetrainStatus, ['zeroing', 'gyro_z_average']);
  gyroZeroZ.setColor([0, 0, 1]);
  gyroZeroZ.setPointSize(0);
  gyroZeroZ.setLabel('Gyro Z Zero');

  const gyroX = gyroPlot.addMessageLine(imu, ['gyro_x']);
  gyroX.setColor([1, 0, 0]);
  const gyroY = gyroPlot.addMessageLine(imu, ['gyro_y']);
  gyroY.setColor([0, 1, 0]);
  const gyroZ = gyroPlot.addMessageLine(imu, ['gyro_z']);
  gyroZ.setColor([0, 0, 1]);

  const tempPlot = aosPlotter.addPlot(element, [width, height / 2]);
  tempPlot.plot.getAxisLabels().setTitle('IMU Temperature');
  tempPlot.plot.getAxisLabels().setYLabel('Temperature (deg C)');
  tempPlot.plot.getAxisLabels().setXLabel('Monotonic Reading Time (sec)');

  tempPlot.addMessageLine(imu, ['temperature']);

  const statePlot = aosPlotter.addPlot(element, [width, height / 2]);
  statePlot.plot.getAxisLabels().setTitle('IMU State');
  statePlot.plot.getAxisLabels().setXLabel('Monotonic Sent Time (sec)');
  statePlot.plot.setDefaultYRange([-0.1, 1.1]);

  const zeroedLine =
      statePlot.addMessageLine(drivetrainStatus, ['zeroing', 'zeroed']);
  zeroedLine.setColor([1, 0, 0]);
  zeroedLine.setDrawLine(false);
  const faultedLine =
  statePlot.addMessageLine(drivetrainStatus, ['zeroing', 'faulted']);
  faultedLine.setColor([0, 1, 0]);
  faultedLine.setPointSize(0);
}
