// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE, ORANGE} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH * 5 / 2;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotCatapult(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Position');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  // Robot Enabled/Disabled and Mode
  const positionPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  positionPlot.plot.getAxisLabels().setTitle('Position');
  positionPlot.plot.getAxisLabels().setXLabel(TIME);
  positionPlot.plot.getAxisLabels().setYLabel('rad');
  positionPlot.plot.setDefaultYRange([-1.0, 2.0]);

  positionPlot.addMessageLine(goal, ['catapult', 'return_position', 'unsafe_goal']).setColor(BLUE).setPointSize(1.0);
  positionPlot.addMessageLine(goal, ['catapult', 'fire']).setColor(WHITE).setPointSize(1.0);
  positionPlot.addMessageLine(status, ['mpc_horizon'])
      .setColor(WHITE)
      .setPointSize(3.0);
  positionPlot.addMessageLine(status, ['catapult', 'goal_position']).setColor(RED).setPointSize(4.0);
  positionPlot.addMessageLine(status, ['catapult', 'goal_velocity']).setColor(ORANGE).setPointSize(4.0);
  positionPlot.addMessageLine(status, ['catapult', 'position']).setColor(GREEN).setPointSize(4.0);
  positionPlot.addMessageLine(status, ['catapult', 'velocity']).setColor(PINK).setPointSize(1.0);
  positionPlot.addMessageLine(status, ['catapult', 'calculated_velocity']).setColor(BROWN).setPointSize(1.0);
  positionPlot.addMessageLine(position, ['catapult', 'pot']).setColor(WHITE).setPointSize(1.0);
  positionPlot.addMessageLine(status, ['catapult', 'estimator_state', 'position']).setColor(CYAN).setPointSize(1.0);

  const voltagePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  voltagePlot.plot.getAxisLabels().setTitle('Voltage');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Volts');
  voltagePlot.plot.setDefaultYRange([-4.0, 14.0]);

  voltagePlot.addMessageLine(output, ['catapult_voltage']).setColor(BLUE).setPointSize(4.0);
  voltagePlot.addMessageLine(status, ['catapult', 'voltage_error']).setColor(RED).setPointSize(1.0);
  voltagePlot.addMessageLine(status, ['catapult', 'position_power']).setColor(BROWN).setPointSize(1.0);
  voltagePlot.addMessageLine(status, ['catapult', 'velocity_power']).setColor(CYAN).setPointSize(1.0);
  voltagePlot.addMessageLine(robotState, ['voltage_battery']).setColor(GREEN).setPointSize(1.0);

  const solvePlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  solvePlot.plot.getAxisLabels().setTitle('Solve Time');
  solvePlot.plot.getAxisLabels().setXLabel(TIME);
  solvePlot.plot.getAxisLabels().setYLabel('Seconds');
  solvePlot.plot.setDefaultYRange([-0.0005, 0.001]);
  solvePlot.addMessageLine(status, ['solve_time']).setColor(WHITE).setPointSize(4.0);
}
