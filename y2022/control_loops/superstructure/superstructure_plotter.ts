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
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Output');
  const status = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Status');
  const position = aosPlotter.addMessageSource('/superstructure', 'y2022.control_loops.superstructure.Position');
  const robotState = aosPlotter.addMessageSource('/aos', 'aos.RobotState');
}
