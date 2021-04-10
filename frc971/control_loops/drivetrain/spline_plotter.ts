// Provides a plot for debugging drivetrain-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

export function plotSpline(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const goal = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Goal');
  const position = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Position');
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Status');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.drivetrain.Output');

  let currentTop = 0;

  // Polydrivetrain (teleop control) plots
  const longitudinalPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  longitudinalPlot.plot.getAxisLabels().setTitle('Longitudinal Distance');
  longitudinalPlot.plot.getAxisLabels().setXLabel(TIME);
  longitudinalPlot.plot.getAxisLabels().setYLabel('meters');

  longitudinalPlot.addMessageLine(
      status, ['trajectory_logging', 'distance_remaining']);

  const boolPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
  currentTop += DEFAULT_HEIGHT / 2;
  boolPlot.plot.getAxisLabels().setTitle('Bool Flags');
  boolPlot.plot.getAxisLabels().setXLabel(TIME);
  boolPlot.plot.getAxisLabels().setYLabel('boolean');

  boolPlot.addMessageLine(status, ['trajectory_logging', 'is_executing'])
      .setColor(RED);
  boolPlot.addMessageLine(status, ['trajectory_logging', 'is_executed'])
      .setColor(BLUE);

  const handlePlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  handlePlot.plot.getAxisLabels().setTitle('Spline Handles');
  handlePlot.plot.getAxisLabels().setXLabel(TIME);
  handlePlot.plot.getAxisLabels().setYLabel('handle number');

  handlePlot
      .addMessageLine(status, ['trajectory_logging', 'available_splines[]'])
      .setColor(RED)
      .setDrawLine(false);
  handlePlot
      .addMessageLine(status, ['trajectory_logging', 'goal_spline_handle'])
      .setColor(BLUE)
      .setPointSize(0.0);
  handlePlot
      .addMessageLine(status, ['trajectory_logging', 'current_spline_idx'])
      .setColor(GREEN)
      .setPointSize(0.0);
}
