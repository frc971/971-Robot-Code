// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE, ORANGE} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH * 5 / 2;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotIntake(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Status');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  // Robot Enabled/Disabled and Mode
  const positionPlotFront =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  positionPlotFront.plot.getAxisLabels().setTitle('Position');
  positionPlotFront.plot.getAxisLabels().setXLabel(TIME);
  positionPlotFront.plot.getAxisLabels().setYLabel('rad');
  positionPlotFront.plot.setDefaultYRange([-1.0, 2.0]);

  positionPlotFront.addMessageLine(status, ['intake_front', 'position']).setColor(GREEN).setPointSize(4.0);
  positionPlotFront.addMessageLine(status, ['intake_front', 'velocity']).setColor(PINK).setPointSize(1.0);
  positionPlotFront.addMessageLine(status, ['intake_front', 'goal_position']).setColor(RED).setPointSize(4.0);
  positionPlotFront.addMessageLine(status, ['intake_front', 'goal_velocity']).setColor(ORANGE).setPointSize(4.0);
  positionPlotFront.addMessageLine(status, ['intake_front', 'estimator_state', 'position']).setColor(CYAN).setPointSize(1.0);

  const positionPlotBack =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  positionPlotBack.plot.getAxisLabels().setTitle('Position');
  positionPlotBack.plot.getAxisLabels().setXLabel(TIME);
  positionPlotBack.plot.getAxisLabels().setYLabel('rad');
  positionPlotBack.plot.setDefaultYRange([-1.0, 2.0]);

  positionPlotBack.addMessageLine(status, ['intake_back', 'position']).setColor(GREEN).setPointSize(4.0);
  positionPlotBack.addMessageLine(status, ['intake_back', 'velocity']).setColor(PINK).setPointSize(1.0);
  positionPlotBack.addMessageLine(status, ['intake_back', 'goal_position']).setColor(RED).setPointSize(4.0);
  positionPlotBack.addMessageLine(status, ['intake_back', 'goal_velocity']).setColor(ORANGE).setPointSize(4.0);
  positionPlotBack.addMessageLine(status, ['intake_back', 'estimator_state', 'position']).setColor(CYAN).setPointSize(1.0);

  const voltagePlotFront =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  voltagePlotFront.plot.getAxisLabels().setTitle('Voltage');
  voltagePlotFront.plot.getAxisLabels().setXLabel(TIME);
  voltagePlotFront.plot.getAxisLabels().setYLabel('Volts');
  voltagePlotFront.plot.setDefaultYRange([-4.0, 14.0]);

  voltagePlotFront.addMessageLine(output, ['intake_front_voltage']).setColor(BLUE).setPointSize(4.0);
  voltagePlotFront.addMessageLine(status, ['intake_front', 'voltage_error']).setColor(RED).setPointSize(1.0);
  voltagePlotFront.addMessageLine(status, ['intake_front', 'position_power']).setColor(BROWN).setPointSize(1.0);
  voltagePlotFront.addMessageLine(status, ['intake_front', 'velocity_power']).setColor(CYAN).setPointSize(1.0);
  voltagePlotFront.addMessageLine(robotState, ['voltage_battery']).setColor(GREEN).setPointSize(1.0);


  const voltagePlotBack =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  voltagePlotBack.plot.getAxisLabels().setTitle('Voltage');
  voltagePlotBack.plot.getAxisLabels().setXLabel(TIME);
  voltagePlotBack.plot.getAxisLabels().setYLabel('Volts');
  voltagePlotBack.plot.setDefaultYRange([-4.0, 14.0]);

  voltagePlotBack.addMessageLine(output, ['intake_back_voltage']).setColor(BLUE).setPointSize(4.0);
  voltagePlotBack.addMessageLine(status, ['intake_back', 'voltage_error']).setColor(RED).setPointSize(1.0);
  voltagePlotBack.addMessageLine(status, ['intake_back', 'position_power']).setColor(BROWN).setPointSize(1.0);
  voltagePlotBack.addMessageLine(status, ['intake_back', 'velocity_power']).setColor(CYAN).setPointSize(1.0);
  voltagePlotBack.addMessageLine(robotState, ['voltage_battery']).setColor(GREEN).setPointSize(1.0);
}
