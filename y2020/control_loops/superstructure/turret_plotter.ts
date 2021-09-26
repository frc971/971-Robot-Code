// Provides a plot for debugging robot state-related issues.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import * as configuration from 'org_frc971/aos/configuration_generated';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';
import {MessageHandler, TimestampedMessage} from 'org_frc971/aos/network/www/aos_plotter';
import {Point} from 'org_frc971/aos/network/www/plotter';
import {Table} from 'org_frc971/aos/network/www/reflection';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';

import Connection = proxy.Connection;
import Schema = configuration.reflection.Schema;

const TIME = AosPlotter.TIME;
const DEFAULT_WIDTH = AosPlotter.DEFAULT_WIDTH;
const DEFAULT_HEIGHT = AosPlotter.DEFAULT_HEIGHT;

class DerivativeMessageHandler extends MessageHandler {
  // Calculated magnitude of the measured acceleration from the IMU.
  private acceleration_magnitudes: Point[] = [];
  constructor(private readonly schema: Schema) {
    super(schema);
  }
  private readScalar(table: Table, fieldName: string): number {
    return this.parser.readScalar(table, fieldName);
  }

  // Computes a numerical derivative for a given input.
  private derivative(input: Point[]): Point[] {
    const num_measurements = input.length;
    const results = [];
    for (let ii = 0; ii < num_measurements - 1; ++ii) {
      const x0 = input[ii].x;
      const x1 = input[ii + 1].x;
      const y0 = input[ii].y;
      const y1 = input[ii + 1].y;
      results.push(new Point((x0 + x1) / 2.0, (y1 - y0) / (x1 - x0)));
    }
    return results;
  }

  getField(field: string[]): Point[] {
    // Any requested input that ends with "_derivative" will get a derivative
    // calculated for the provided field.
    const derivative_suffix = "_derivative";
    const num_fields = field.length;
    const end_field = field[num_fields - 1];
    if (end_field.endsWith(derivative_suffix)) {
      const field_copy = [];
      for (let ii = 0; ii < num_fields - 1; ++ii) {
        field_copy.push(field[ii]);
      }
      field_copy.push(end_field.slice(0, end_field.length - derivative_suffix.length));
      return this.derivative(this.getField(field_copy));
    } else {
      return super.getField(field);
    }
  }
}

export function plotTurret(conn: Connection, element: Element) : void {
  const aosPlotter = new AosPlotter(conn);
  const goal = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Goal');
  const output = aosPlotter.addMessageSource('/superstructure', 'y2020.control_loops.superstructure.Output');
  const status = aosPlotter.addRawMessageSource(
      '/superstructure', 'y2020.control_loops.superstructure.Status',
    new DerivativeMessageHandler(conn.getSchema('y2020.control_loops.superstructure.Status'))
  );
  const pdpValues =
      aosPlotter.addMessageSource('/roborio/aos', 'frc971.PDPValues');
  const localizerDebug =
      aosPlotter.addMessageSource('/drivetrain', 'y2020.control_loops.drivetrain.LocalizerDebug');

  var currentTop = 0;

  const turretPosPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretPosPlot.plot.getAxisLabels().setTitle('Turret Position');
  turretPosPlot.plot.getAxisLabels().setXLabel(TIME);
  turretPosPlot.plot.getAxisLabels().setYLabel('rad');

  turretPosPlot.addMessageLine(status, ['aimer', 'turret_position'])
      .setColor(RED)
      .setPointSize(0.0);
  turretPosPlot.addMessageLine(status, ['turret', 'position'])
      .setColor(GREEN)
      .setPointSize(0.0);
  turretPosPlot.addMessageLine(localizerDebug, ['matches[]', 'implied_turret_goal'])
      .setColor(GREEN)
      .setDrawLine(false);
  turretPosPlot.addMessageLine(status, ['turret', 'unprofiled_goal_position'])
      .setColor(BLUE)
      .setDrawLine(false);

  const turretVelPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretVelPlot.plot.getAxisLabels().setTitle('Turret Velocity');
  turretVelPlot.plot.getAxisLabels().setXLabel(TIME);
  turretVelPlot.plot.getAxisLabels().setYLabel('rad / sec');

  turretVelPlot.addMessageLine(status, ['aimer', 'turret_velocity'])
      .setColor(RED)
      .setPointSize(0.0);
  turretVelPlot.addMessageLine(status, ['turret', 'velocity'])
      .setColor(GREEN)
      .setPointSize(0.0);
  turretVelPlot.addMessageLine(status, ['turret', 'unprofiled_goal_velocity'])
      .setColor(BLUE)
      .setDrawLine(false);

  const turretAccelPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretAccelPlot.plot.getAxisLabels().setTitle('Turret Acceleration');
  turretAccelPlot.plot.getAxisLabels().setXLabel(TIME);
  turretAccelPlot.plot.getAxisLabels().setYLabel('rad / sec / sec');

  turretAccelPlot.addMessageLine(status, ['aimer', 'turret_velocity_derivative'])
      .setColor(RED)
      .setPointSize(0.0);

  const turretVoltagePlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  turretVoltagePlot.plot.getAxisLabels().setTitle('Turret Voltage');
  turretVoltagePlot.plot.getAxisLabels().setXLabel(TIME);
  turretVoltagePlot.plot.getAxisLabels().setYLabel('V');

  turretVoltagePlot.addMessageLine(status, ['turret', 'voltage_error'])
      .setColor(GREEN)
      .setPointSize(0.0);
  turretVoltagePlot.addMessageLine(status, ['turret', 'position_power'])
      .setColor(BLUE)
      .setPointSize(0.0);
  turretVoltagePlot.addMessageLine(status, ['turret', 'velocity_power'])
      .setColor(CYAN)
      .setPointSize(0.0);
  turretVoltagePlot.addMessageLine(output, ['turret_voltage'])
      .setColor(RED)
      .setPointSize(0.0);

  const currentPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  currentPlot.plot.getAxisLabels().setTitle('Current');
  currentPlot.plot.getAxisLabels().setXLabel(TIME);
  currentPlot.plot.getAxisLabels().setYLabel('Amps');
  currentPlot.plot.setDefaultYRange([0.0, 40.0]);

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

  const imageAcceptedPlot = aosPlotter.addPlot(
      element, [0, currentTop], [DEFAULT_WIDTH, DEFAULT_HEIGHT]);
  currentTop += DEFAULT_HEIGHT;
  imageAcceptedPlot.plot.getAxisLabels().setTitle('Image Acceptance');
  imageAcceptedPlot.plot.getAxisLabels().setXLabel(TIME);
  imageAcceptedPlot.plot.getAxisLabels().setYLabel('[bool]');
  imageAcceptedPlot.plot.setDefaultYRange([-0.05, 1.05]);

  imageAcceptedPlot.addMessageLine(localizerDebug, ['matches[]', 'accepted'])
      .setColor(RED)
      .setDrawLine(false);
}
