// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import * as proxy from '../../../aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotHood(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Status');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  var currentTop = 0;

  // Robot Enabled/Disabled and Mode
  const positionPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  positionPlot.plot.getAxisLabels().setTitle('Position');
  positionPlot.plot.getAxisLabels().setXLabel(TIME);
  positionPlot.plot.getAxisLabels().setYLabel('rad');
  positionPlot.plot.setDefaultYRange([-0.1, 0.7]);

  positionPlot.addMessageLine(goal, ['hood', 'unsafe_goal']).setColor(BLUE).setPointSize(0.0);
  positionPlot.addMessageLine(status, ['hood', 'goal_position']).setColor(RED).setPointSize(0.0);
  positionPlot.addMessageLine(status, ['hood', 'position']).setColor(GREEN).setPointSize(0.0);
  positionPlot.addMessageLine(status, ['hood', 'velocity']).setColor(PINK).setPointSize(0.0);
  positionPlot.addMessageLine(status, ['hood', 'calculated_velocity']).setColor(BROWN).setPointSize(0.0);
  positionPlot.addMessageLine(status, ['hood', 'estimator_state', 'position']).setColor(CYAN).setPointSize(0.0);

  const voltagePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  voltagePlot.plot.getAxisLabels().setTitle('Voltage');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Volts');
  voltagePlot.plot.setDefaultYRange([-4.0, 16.0]);

  voltagePlot.addMessageLine(output, ['hood_voltage']).setColor(BLUE).setPointSize(0.0);
  voltagePlot.addMessageLine(status, ['hood', 'voltage_error']).setColor(RED).setPointSize(0.0);
  voltagePlot.addMessageLine(robotState, ['voltage_battery']).setColor(GREEN).setPointSize(0.0);
}
