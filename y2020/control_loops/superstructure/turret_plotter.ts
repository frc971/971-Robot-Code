// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotTurret(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource(
      '/superstructure', 'y2020.control_loops.superstructure.Status');
  const pdpValues =
      aosPlotter.addMessageSource('/roborio/aos', 'frc971.PDPValues');

  var currentTop = 0;

  const turretPosPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretPosPlot.plot.getAxisLabels().setTitle('Turret Goal Position');
  turretPosPlot.plot.getAxisLabels().setXLabel(TIME);
  turretPosPlot.plot.getAxisLabels().setYLabel('rad');

  turretPosPlot.addMessageLine(status, ['aimer', 'turret_position'])
      .setColor(RED)
      .setPointSize(0.0);
  turretPosPlot.addMessageLine(status, ['turret', 'position'])
      .setColor(GREEN)
      .setPointSize(0.0);
  turretPosPlot.addMessageLine(status, ['turret', 'unprofiled_goal_position'])
      .setColor(BLUE)
      .setPointSize(0.0);

  const turretVoltagePlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretVoltagePlot.plot.getAxisLabels().setTitle('Turret Voltage');
  turretVoltagePlot.plot.getAxisLabels().setXLabel(TIME);
  turretVoltagePlot.plot.getAxisLabels().setYLabel('V');

  turretVoltagePlot.addMessageLine(output, ['turret_voltage'])
      .setColor(RED)
      .setPointSize(0.0);

  const currentPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  currentPlot.plot.getAxisLabels().setTitle('Current');
  currentPlot.plot.getAxisLabels().setXLabel(TIME);
  currentPlot.plot.getAxisLabels().setYLabel('Amps');
  currentPlot.plot.setDefaultYRange([0.0, 80.0]);

  currentPlot.addMessageLine(pdpValues, ['currents[6]'])
      .setColor(GREEN)
      .setPointSize(0.0);


  const targetDistancePlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  targetDistancePlot.plot.getAxisLabels().setTitle('Target distance');
  targetDistancePlot.plot.getAxisLabels().setXLabel(TIME);
  targetDistancePlot.plot.getAxisLabels().setYLabel('m');

  targetDistancePlot.addMessageLine(status, ['aimer', 'target_distance'])
      .setColor(RED)
      .setPointSize(0.0);

  const targetChoicePlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  targetChoicePlot.plot.getAxisLabels().setTitle('Target choice');
  targetChoicePlot.plot.getAxisLabels().setXLabel(TIME);
  targetChoicePlot.plot.getAxisLabels().setYLabel('[bool]');
  targetChoicePlot.plot.setDefaultYRange([-0.05, 1.05]);

  targetChoicePlot.addMessageLine(status, ['aimer', 'aiming_for_inner_port'])
      .setColor(RED)
      .setPointSize(0.0);
}
