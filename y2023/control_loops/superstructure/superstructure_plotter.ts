// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from '../../../aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';
import * as proxy from '../../../aos/network/www/proxy';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH * 2;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotSuperstructure(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  //const goal = aosPlotter.addMessageSource(
  //    '/superstructure', 'y2023.control_loops.superstructure.Goal');
  //const output = aosPlotter.addMessageSource(
  //    '/superstructure', 'y2023.control_loops.superstructure.Output');
  //const status = aosPlotter.addMessageSource(
  //    '/superstructure', 'y2023.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource(
      '/superstructure', 'y2023.control_loops.superstructure.Position');
  //const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  const positionPlot =
      aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  positionPlot.plot.getAxisLabels().setTitle('States');
  positionPlot.plot.getAxisLabels().setXLabel(TIME);
  positionPlot.plot.getAxisLabels().setYLabel('wonky state units');
  positionPlot.plot.setDefaultYRange([-1.0, 2.0]);

  positionPlot.addMessageLine(position, ['arm', 'distal', 'pot'])
      .setColor(RED)
      .setPointSize(4.0);
  positionPlot.addMessageLine(position, ['arm', 'distal', 'absolute_encoder'])
      .setColor(BLUE)
      .setPointSize(4.0);
  positionPlot.addMessageLine(position, ['arm', 'distal', 'encoder'])
      .setColor(GREEN)
      .setPointSize(4.0);
}
