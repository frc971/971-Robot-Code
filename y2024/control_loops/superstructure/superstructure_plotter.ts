// Provides a plot for debugging robot state-related issues.
import {AosPlotter, MessageHandler} from '../../../aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../../aos/network/www/colors';
import * as proxy from '../../../aos/network/www/proxy';

import Connection = proxy.Connection;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH * 2;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT * 1;

function plotSzsdofSubsystem(
    name: string, plotter: AosPlotter, element: Element, position: MessageHandler, positionName: string,
    status: MessageHandler, statusName: string, output: MessageHandler, outputName: string, hasPot:boolean = true): void {
  {
    const positionPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    positionPlot.plot.getAxisLabels().setTitle(name + ' Position');
    positionPlot.plot.getAxisLabels().setXLabel(TIME);
    positionPlot.plot.getAxisLabels().setYLabel('Position [rad,m]');
    positionPlot.addMessageLine(position, [positionName, 'encoder'])
        .setColor(RED);
    positionPlot.addMessageLine(position, [positionName, 'absolute_encoder'])
        .setColor(GREEN);
    if (hasPot) {
      positionPlot.addMessageLine(position, [positionName, 'pot'])
          .setColor(BLUE);
    }
    positionPlot
        .addMessageLine(status, [statusName, 'estimator_state', 'position'])
        .setColor(BROWN);
    positionPlot.addMessageLine(status, [statusName, 'position'])
        .setColor(WHITE);
  }
  {
    const statesPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT / 2]);
    statesPlot.plot.getAxisLabels().setTitle(name + ' State');
    statesPlot.plot.getAxisLabels().setXLabel(TIME);
    statesPlot.plot.getAxisLabels().setYLabel('[bool,ZeroingError]');
    statesPlot.addMessageLine(status, [statusName, 'estopped']).setColor(RED);
    statesPlot.addMessageLine(status, [statusName, 'zeroed']).setColor(GREEN);
    statesPlot
        .addMessageLine(status, [statusName, 'estimator_state', 'errors[]'])
        .setColor(BLUE)
        .setDrawLine(false);
  }
  {
    const positionConvergencePlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    positionConvergencePlot.plot.getAxisLabels().setTitle(name + ' Position Goals');
    positionConvergencePlot.plot.getAxisLabels().setXLabel(TIME);
    positionConvergencePlot.plot.getAxisLabels().setYLabel('[rad,m]');
    positionConvergencePlot.addMessageLine(status, [statusName, 'position'])
        .setColor(RED);
    positionConvergencePlot.addMessageLine(status, [statusName, 'goal_position'])
        .setColor(GREEN);
    positionConvergencePlot
        .addMessageLine(status, [statusName, 'unprofiled_goal_position'])
        .setColor(BROWN);
  }
  {
    const velocityConvergencePlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    velocityConvergencePlot.plot.getAxisLabels().setTitle(name + ' Velocity Goals');
    velocityConvergencePlot.plot.getAxisLabels().setXLabel(TIME);
    velocityConvergencePlot.plot.getAxisLabels().setYLabel('[rad,m]');
    velocityConvergencePlot.addMessageLine(status, [statusName, 'velocity'])
        .setColor(RED);
    velocityConvergencePlot.addMessageLine(status, [statusName, 'calculated_velocity'])
        .setColor(RED).setDrawLine(false);
    velocityConvergencePlot.addMessageLine(status, [statusName, 'goal_velocity'])
        .setColor(GREEN);
    velocityConvergencePlot
        .addMessageLine(status, [statusName, 'unprofiled_goal_velocity'])
        .setColor(BROWN);
  }
  {
    const outputPlot =
        plotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    outputPlot.plot.getAxisLabels().setTitle(name + ' Outputs');
    outputPlot.plot.getAxisLabels().setXLabel(TIME);
    outputPlot.plot.getAxisLabels().setYLabel('[volts]');
    outputPlot.addMessageLine(output, [outputName])
        .setColor(RED);
    outputPlot.addMessageLine(status, [statusName, 'voltage_error'])
        .setColor(GREEN);
    outputPlot.addMessageLine(status, [statusName, 'position_power'])
        .setColor(BLUE);
    outputPlot.addMessageLine(status, [statusName, 'velocity_power'])
        .setColor(BROWN);
    outputPlot.addMessageLine(status, [statusName, 'feedforwards_power'])
        .setColor(WHITE);
  }
}

export function plotSuperstructure(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  const status = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Status');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');

  {
    const robotStatePlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    robotStatePlot.plot.getAxisLabels().setTitle('Robot State Plot');
    robotStatePlot.plot.getAxisLabels().setXLabel(TIME);
    robotStatePlot.plot.getAxisLabels().setYLabel('[bool]');
    robotStatePlot.addMessageLine(robotState, ['outputs_enabled'])
        .setColor(RED);
    robotStatePlot.addMessageLine(status, ['zeroed'])
        .setColor(GREEN);
    robotStatePlot.addMessageLine(status, ['estopped'])
        .setColor(BLUE);
  }
}

export function plotClimber(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Position');
  {
    const goalPlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    goalPlot.plot.getAxisLabels().setTitle('Climber Goal');
    goalPlot.plot.getAxisLabels().setXLabel(TIME);
    goalPlot.plot.getAxisLabels().setYLabel('[enum]');
    goalPlot.addMessageLine(goal, ['climber_goal']).setColor(RED);
  }

  plotSzsdofSubsystem(
      'Climber', aosPlotter, element, position, 'climber', status, 'climber',
      output, 'climber_voltage');
}

export function plotIntake(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Position');

  {
    const goalPlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    goalPlot.plot.getAxisLabels().setTitle('Intake Goal');
    goalPlot.plot.getAxisLabels().setXLabel(TIME);
    goalPlot.plot.getAxisLabels().setYLabel('[enum]');
    goalPlot.addMessageLine(goal, ['intake_goal']).setColor(RED);
  }
  {
    const rollerPlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    rollerPlot.plot.getAxisLabels().setTitle('Intake Rollers');
    rollerPlot.plot.getAxisLabels().setXLabel(TIME);
    rollerPlot.plot.getAxisLabels().setYLabel('[enum,voltage]');
    rollerPlot.addMessageLine(status, ['intake_roller']).setColor(RED);
    rollerPlot.addMessageLine(status, ['transfer_roller']).setColor(BLUE);
    rollerPlot.addMessageLine(output, ['intake_roller_voltage'])
        .setColor(RED)
        .setPointSize(0);
    rollerPlot.addMessageLine(output, ['transfer_roller_voltage'])
        .setColor(BLUE)
        .setPointSize(0);
  }

  plotSzsdofSubsystem(
      'Intake', aosPlotter, element, position, 'intake_pivot', status, 'intake_pivot',
      output, 'intake_pivot_voltage', false);
}

export function plotExtend(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource(
      '/superstructure', 'y2024.control_loops.superstructure.Position');

  {
    const goalPlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    goalPlot.plot.getAxisLabels().setTitle('Extend Goal');
    goalPlot.plot.getAxisLabels().setXLabel(TIME);
    goalPlot.plot.getAxisLabels().setYLabel('[enum]');
    goalPlot.addMessageLine(goal, ['intake_goal']).setColor(RED);
  }
  {
    const rollerPlot =
        aosPlotter.addPlot(element, [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
    rollerPlot.plot.getAxisLabels().setTitle('Extend Rollers');
    rollerPlot.plot.getAxisLabels().setXLabel(TIME);
    rollerPlot.plot.getAxisLabels().setYLabel('[enum,voltage]');
    rollerPlot.addMessageLine(status, ['intake_roller']).setColor(RED);
    rollerPlot.addMessageLine(status, ['transfer_roller']).setColor(BLUE);
    rollerPlot.addMessageLine(output, ['intake_roller_voltage'])
        .setColor(RED)
        .setPointSize(0);
    rollerPlot.addMessageLine(output, ['transfer_roller_voltage'])
        .setColor(BLUE)
        .setPointSize(0);
  }

  plotSzsdofSubsystem(
      'Extend', aosPlotter, element, position, 'extend', status, 'extend',
      output, 'extend_voltage');
}
