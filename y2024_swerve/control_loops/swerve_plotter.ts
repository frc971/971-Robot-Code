// Provides a plot for debugging robot state-related issues.
import {AosPlotter, MessageHandler} from '../../aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../aos/network/www/colors';
import * as proxy from '../../aos/network/www/proxy';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH * 2;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 1;

function plotModule(
    name: string, plotter: AosPlotter, element: Element, position: MessageHandler, positionName: string[],
    status: MessageHandler, statusName: string[], output: MessageHandler, outputName: string[]): void {
  {
    const positionPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    positionPlot.plot.getAxisLabels().setTitle(name + ' Position');
    positionPlot.plot.getAxisLabels().setXLabel(TIME);
    positionPlot.plot.getAxisLabels().setYLabel('Position [rad,m]');
    positionPlot.addMessageLine(position, positionName .concat(  ['encoder']))
        .setColor(RED);
    positionPlot.addMessageLine(position, positionName .concat( ['absolute_encoder']))
        .setColor(GREEN);
    positionPlot
        .addMessageLine(status, statusName .concat( [ 'estimator_state', 'position']))
        .setColor(BROWN);
    positionPlot.addMessageLine(status, statusName .concat( [ 'position']))
        .setColor(WHITE);
  }
  {
    const statesPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
    statesPlot.plot.getAxisLabels().setTitle(name + ' State');
    statesPlot.plot.getAxisLabels().setXLabel(TIME);
    statesPlot.plot.getAxisLabels().setYLabel('[bool,ZeroingError]');
    statesPlot.addMessageLine(status, statusName .concat( [ 'estopped'])).setColor(RED);
    statesPlot.addMessageLine(
        status, statusName.concat(['zeroed'])).setColor(GREEN);
    statesPlot
        .addMessageLine(status, statusName .concat( [ 'estimator_state', 'errors[]']))
        .setColor(BLUE)
        .setDrawLine(false);
  }
  {
    const positionConvergencePlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    positionConvergencePlot.plot.getAxisLabels().setTitle(name + ' Position Goals');
    positionConvergencePlot.plot.getAxisLabels().setXLabel(TIME);
    positionConvergencePlot.plot.getAxisLabels().setYLabel('[rad,m]');
    positionConvergencePlot.addMessageLine(status, statusName .concat( [ 'position']))
        .setColor(RED);
    positionConvergencePlot.addMessageLine(status, statusName .concat( [ 'goal_position']))
        .setColor(GREEN);
    positionConvergencePlot
        .addMessageLine(status, statusName .concat( [ 'unprofiled_goal_position']))
        .setColor(BROWN);
  }
  {
    const velocityConvergencePlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    velocityConvergencePlot.plot.getAxisLabels().setTitle(name + ' Velocity Goals');
    velocityConvergencePlot.plot.getAxisLabels().setXLabel(TIME);
    velocityConvergencePlot.plot.getAxisLabels().setYLabel('[rad,m]');
    velocityConvergencePlot.addMessageLine(status, statusName .concat( [ 'velocity']))
        .setColor(RED);
    velocityConvergencePlot.addMessageLine(status, statusName .concat( [ 'calculated_velocity']))
        .setColor(RED).setDrawLine(false);
    velocityConvergencePlot.addMessageLine(status, statusName .concat( [ 'goal_velocity']))
        .setColor(GREEN);
    velocityConvergencePlot
        .addMessageLine(status, statusName .concat( [ 'unprofiled_goal_velocity']))
        .setColor(BROWN);
  }
  {
    const outputPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    outputPlot.plot.getAxisLabels().setTitle(name + ' Outputs');
    outputPlot.plot.getAxisLabels().setXLabel(TIME);
    outputPlot.plot.getAxisLabels().setYLabel('[volts]');
    outputPlot.addMessageLine(output, outputName)
        .setColor(RED);
    outputPlot.addMessageLine(status, statusName .concat( [ 'voltage_error']))
        .setColor(GREEN);
    outputPlot.addMessageLine(status, statusName .concat( [ 'position_power']))
        .setColor(BLUE);
    outputPlot.addMessageLine(status, statusName .concat( [ 'velocity_power']))
        .setColor(BROWN);
    outputPlot.addMessageLine(status, statusName .concat( [ 'feedforwards_power']))
        .setColor(WHITE);
  }
}

export function plotSwerve(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  {
    const robotStatePlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    robotStatePlot.plot.getAxisLabels().setTitle('Robot State Plot');
    robotStatePlot.plot.getAxisLabels().setXLabel(TIME);
    robotStatePlot.plot.getAxisLabels().setYLabel('[bool]');
    robotStatePlot.addMessageLine(robotState, ['outputs_enabled'])
        .setColor(RED);
  }

  const goal = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.swerve.Goal');
  const output = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.swerve.Output');
  const status = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.swerve.Status');
  const position = aosPlotter.addMessageSource(
      '/drivetrain', 'frc971.control_loops.swerve.Position');

  plotModule(
      'Front Left', aosPlotter, element, position, ['front_left', 'rotation_position'], status, ['front_left_status', 'rotation'],
      output, ['front_left_output', 'rotation_current']);
  plotModule(
      'Front Right', aosPlotter, element, position, ['front_right', 'rotation_position'], status, ['front_right_status', 'rotation'],
      output, ['front_right_output', 'rotation_current']);
  plotModule(
      'Back Left', aosPlotter, element, position, ['back_left', 'rotation_position'], status, ['back_left_status', 'rotation'],
      output, ['back_left_output', 'rotation_current']);
  // TODO(james): Chrome runs out of WebGL canvasses when attempting to plot the final module....
//  plotModule(
//      'Back Right', aosPlotter, element, position, ['back_right', 'rotation_position'], status, ['back_right_status', 'rotation'],
//      output, ['back_right_output', 'rotation_current']);
}
