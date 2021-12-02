// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 3;

export function plotSuperstructure(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2021_bot3.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2021_bot3.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2021_bot3.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource('/superstructure', 'y2021_bot3.control_loops.superstructure.Position');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  var currentTop = 0;

  const intakePlot =
      aosPlotter.addPlot(element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  intakePlot.plot.getAxisLabels().setTitle('Intake');
  intakePlot.plot.getAxisLabels().setXLabel(TIME);
  intakePlot.plot.getAxisLabels().setYLabel('Volts');
  intakePlot.plot.setDefaultYRange([-20.0, 20.0]);

  intakePlot.addMessageLine(output, ['intake_volts']).setColor(BLUE);
  intakePlot.addMessageLine(goal, ['intake_speed']).setColor(GREEN);
  intakePlot.addMessageLine(status, ['intake_speed']).setColor(RED);

  const outtakePlot =
      aosPlotter.addPlot(element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  outtakePlot.plot.getAxisLabels().setTitle('Outtake');
  outtakePlot.plot.getAxisLabels().setXLabel(TIME);
  outtakePlot.plot.getAxisLabels().setYLabel('Volts');
  outtakePlot.plot.setDefaultYRange([-20.0, 20.0]);

  outtakePlot.addMessageLine(output, ['outtake_volts']).setColor(BLUE);
  outtakePlot.addMessageLine(goal, ['outtake_speed']).setColor(GREEN);
  outtakePlot.addMessageLine(status, ['outtake_speed']).setColor(RED);
}
