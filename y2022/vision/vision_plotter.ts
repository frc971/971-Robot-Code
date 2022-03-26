import {ByteBuffer} from 'flatbuffers';
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {MessageHandler, TimestampedMessage} from 'org_frc971/aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from 'org_frc971/aos/network/www/colors';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import {Table} from 'org_frc971/aos/network/www/reflection';
import {Schema} from 'org_frc971/external/com_github_google_flatbuffers/reflection/reflection_generated';
import {TargetEstimate} from 'org_frc971/y2022/vision/target_estimate_generated';


const TIME = AosPlotter.TIME;
// magenta, yellow, cyan, orange
const PI_COLORS = [[255, 0, 255], [255, 255, 0], [0, 255, 255], [255, 165, 0]];

class VisionMessageHandler extends MessageHandler {
  constructor(private readonly schema: Schema) {
    super(schema);
  }

  private readScalar(table: Table, fieldName: string): number|BigInt|null {
    return this.parser.readScalar(table, fieldName);
  }

  addMessage(data: Uint8Array, time: number): void {
    const target = TargetEstimate.getRootAsTargetEstimate(new ByteBuffer(data));
    // Copied from localizer.cc
    const MIN_TARGET_ESTIMATE_CONFIDENCE = 0.75;
    // Only add estimates with decent confidences - these are what the localizer
    // uses
    if (target.confidence() >= MIN_TARGET_ESTIMATE_CONFIDENCE) {
      const table = Table.getNamedTable(
          target.bb, this.schema, 'y2022.vision.TargetEstimate', target.bb_pos);
      this.messages.push(new TimestampedMessage(table, time));
    }
  }
}

export function plotVision(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const targets = [];
  for (const pi of ['pi1', 'pi2', 'pi3', 'pi4']) {
    targets.push(aosPlotter.addRawMessageSource(
        '/' + pi + '/camera', 'y2022.vision.TargetEstimate',
        new VisionMessageHandler(
            conn.getSchema('y2022.vision.TargetEstimate'))));
  }
  const localizer = aosPlotter.addMessageSource(
      '/localizer', 'frc971.controls.LocalizerVisualization');
  const localizerOutput = aosPlotter.addMessageSource(
      '/localizer', 'frc971.controls.LocalizerOutput');
  const superstructureStatus = aosPlotter.addMessageSource(
      '/superstructure', 'y2022.control_loops.superstructure.Status');

  const rejectionPlot = aosPlotter.addPlot(element);
  rejectionPlot.plot.getAxisLabels().setTitle('Rejection Reasons');
  rejectionPlot.plot.getAxisLabels().setXLabel(TIME);
  rejectionPlot.plot.getAxisLabels().setYLabel('[bool, enum]');

  rejectionPlot.addMessageLine(localizer, ['targets[]', 'accepted'])
      .setDrawLine(false)
      .setColor(BLUE);
  rejectionPlot.addMessageLine(localizer, ['targets[]', 'rejection_reason'])
      .setDrawLine(false)
      .setColor(RED);

  const xPlot = aosPlotter.addPlot(element);
  xPlot.plot.getAxisLabels().setTitle('X Position');
  xPlot.plot.getAxisLabels().setXLabel(TIME);
  xPlot.plot.getAxisLabels().setYLabel('[m]');

  xPlot.addMessageLine(localizer, ['targets[]', 'implied_robot_x'])
      .setDrawLine(false)
      .setColor(RED);
  xPlot.addMessageLine(localizerOutput, ['x'])
      .setDrawLine(false)
      .setColor(BLUE);

  const yPlot = aosPlotter.addPlot(element);
  yPlot.plot.getAxisLabels().setTitle('Y Position');
  yPlot.plot.getAxisLabels().setXLabel(TIME);
  yPlot.plot.getAxisLabels().setYLabel('[m]');

  yPlot.addMessageLine(localizer, ['targets[]', 'implied_robot_y'])
      .setDrawLine(false)
      .setColor(RED);
  yPlot.addMessageLine(localizerOutput, ['y'])
      .setDrawLine(false)
      .setColor(BLUE);

  const turretPlot = aosPlotter.addPlot(element);
  turretPlot.plot.getAxisLabels().setTitle('Turret Position');
  turretPlot.plot.getAxisLabels().setXLabel(TIME);
  turretPlot.plot.getAxisLabels().setYLabel('[rad]');

  turretPlot.addMessageLine(localizer, ['targets[]', 'implied_turret_goal'])
      .setDrawLine(false)
      .setColor(RED);
  turretPlot.addMessageLine(superstructureStatus, ['turret', 'position'])
      .setPointSize(0.0)
      .setColor(BLUE);
  turretPlot.addMessageLine(superstructureStatus, ['aimer', 'turret_position'])
      .setPointSize(0.0)
      .setColor(GREEN);

  const anglePlot = aosPlotter.addPlot(element);
  anglePlot.plot.getAxisLabels().setTitle('TargetEstimate Angle');
  anglePlot.plot.getAxisLabels().setXLabel(TIME);
  anglePlot.plot.getAxisLabels().setYLabel('[rad]');

  for (let ii = 0; ii < targets.length; ++ii) {
    anglePlot.addMessageLine(targets[ii], ['angle_to_target'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel('pi' + (ii + 1));
  }

  const distancePlot = aosPlotter.addPlot(element);
  distancePlot.plot.getAxisLabels().setTitle('TargetEstimate Distance');
  distancePlot.plot.getAxisLabels().setXLabel(TIME);
  distancePlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    distancePlot.addMessageLine(targets[ii], ['distance'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel('pi' + (ii + 1));
  }

  const confidencePlot = aosPlotter.addPlot(element);
  confidencePlot.plot.getAxisLabels().setTitle('TargetEstimate Confidence');
  confidencePlot.plot.getAxisLabels().setXLabel(TIME);
  confidencePlot.plot.getAxisLabels().setYLabel('[0-1]');

  for (let ii = 0; ii < targets.length; ++ii) {
    confidencePlot.addMessageLine(targets[ii], ['confidence'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel('pi' + (ii + 1));
  }
}
