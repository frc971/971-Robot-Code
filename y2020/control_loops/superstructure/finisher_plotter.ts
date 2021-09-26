// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE, BLACK} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotFinisher(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Status');
  const pdpValues = aosPlotter.addMessageSource('/roborio/aos', 'frc971.PDPValues');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  var currentTop = 0;

  // Robot Enabled/Disabled and Mode
  const velocityPlot =
      aosPlotter.addPlot(element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  velocityPlot.plot.getAxisLabels().setTitle('Velocity');
  velocityPlot.plot.getAxisLabels().setXLabel(TIME);
  velocityPlot.plot.getAxisLabels().setYLabel('rad/s');
  velocityPlot.plot.setDefaultYRange([0.0, 600.0]);

  velocityPlot.addMessageLine(goal, ['shooter', 'velocity_finisher']).setColor(BLUE).setPointSize(0.0);
  velocityPlot.addMessageLine(status, ['shooter', 'finisher', 'avg_angular_velocity']).setColor(RED).setPointSize(0.0);
  velocityPlot.addMessageLine(status, ['shooter', 'finisher', 'angular_velocity']).setColor(GREEN).setPointSize(0.0);
  velocityPlot.addMessageLine(status, ['shooter', 'ready']).setColor(BLACK).setPointSize(0.0);
  velocityPlot.addMessageLine(status, ['shooter', 'finisher', 'dt_angular_velocity']).setColor(PINK).setPointSize(0.0);

  const voltagePlot =
      aosPlotter.addPlot(element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  voltagePlot.plot.getAxisLabels().setTitle('Voltage');
  voltagePlot.plot.getAxisLabels().setXLabel(TIME);
  voltagePlot.plot.getAxisLabels().setYLabel('Volts');
  voltagePlot.plot.setDefaultYRange([-4.0, 16.0]);

  voltagePlot.addMessageLine(output, ['finisher_voltage']).setColor(BLUE).setPointSize(0.0);
  voltagePlot.addMessageLine(status, ['shooter', 'finisher', 'voltage_error']).setColor(RED).setPointSize(0.0);
  voltagePlot.addMessageLine(robotState, ['voltage_battery']).setColor(GREEN).setPointSize(0.0);


  const currentPlot =
      aosPlotter.addPlot(element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  currentPlot.plot.getAxisLabels().setTitle('Current');
  currentPlot.plot.getAxisLabels().setXLabel(TIME);
  currentPlot.plot.getAxisLabels().setYLabel('Amps');
  currentPlot.plot.setDefaultYRange([0.0, 80.0]);

  currentPlot.addMessageLine(pdpValues, ['currents[12]']).setColor(GREEN).setPointSize(0.0);
  currentPlot.addMessageLine(pdpValues, ['currents[13]']).setColor(BLUE).setPointSize(0.0);
  currentPlot.addMessageLine(status, ['shooter', 'finisher', 'commanded_current']).setColor(RED).setPointSize(0.0);
}
