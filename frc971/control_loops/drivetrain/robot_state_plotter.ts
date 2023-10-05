// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import * as proxy from '../../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotRobotState(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');
  const joystickState = aosPlotter.addMessageSource('/aos', 'aos.JoystickState');

  var currentTop = 0;

  // Robot Enabled/Disabled and Mode
  const robotStatePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  robotStatePlot.plot.getAxisLabels().setTitle('Robot State');
  robotStatePlot.plot.getAxisLabels().setXLabel(TIME);
  robotStatePlot.plot.getAxisLabels().setYLabel('bool');
  robotStatePlot.plot.setDefaultYRange([-0.1, 1.1]);

  const testMode = robotStatePlot.addMessageLine(joystickState, ['test_mode']);
  testMode.setColor(BLUE);
  testMode.setPointSize(0.0);
  const autoMode = robotStatePlot.addMessageLine(joystickState, ['autonomous']);
  autoMode.setColor(RED);
  autoMode.setPointSize(0.0);

  const brownOut = robotStatePlot.addMessageLine(robotState, ['browned_out']);
  brownOut.setColor(BROWN);
  brownOut.setDrawLine(false);
  robotStatePlot.addMessageLine(robotState, ['outputs_enabled'])
      .setColor(CYAN)
      .setDrawLine(false);
  const enabled = robotStatePlot.addMessageLine(joystickState, ['enabled']);
  enabled.setColor(GREEN);
  enabled.setDrawLine(false);

  // Battery Voltage
  const batteryPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  batteryPlot.plot.getAxisLabels().setTitle('Battery Voltage');
  batteryPlot.plot.getAxisLabels().setXLabel(TIME);
  batteryPlot.plot.getAxisLabels().setYLabel('Voltage (V)');

  batteryPlot.addMessageLine(robotState, ['voltage_battery']);

  // PID of process reading sensors
  const readerPidPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  readerPidPlot.plot.getAxisLabels().setTitle("PID of Process Reading Sensors");
  readerPidPlot.plot.getAxisLabels().setXLabel(TIME);
  readerPidPlot.plot.getAxisLabels().setYLabel("wpilib_interface Process ID");
  readerPidPlot.addMessageLine(robotState, ["reader_pid"]);
}
