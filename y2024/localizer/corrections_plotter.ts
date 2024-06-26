import {ByteBuffer} from 'flatbuffers';
import {AosPlotter} from '../../aos/network/www/aos_plotter';
import {MessageHandler, TimestampedMessage} from '../../aos/network/www/aos_plotter';
import {BLUE, BROWN, CYAN, GREEN, PINK, RED, WHITE} from '../../aos/network/www/colors';
import {Connection} from '../../aos/network/www/proxy';
import {Table} from '../../aos/network/www/reflection';
import {Schema} from 'flatbuffers_reflection/reflection_generated';
import {Visualization, TargetEstimateDebug} from './visualization_generated';


const TIME = AosPlotter.TIME;
// magenta, yellow, cyan, black
const PI_COLORS = [[255, 0, 255], [255, 255, 0], [0, 255, 255], [0, 0, 0]];

class VisionMessageHandler extends MessageHandler {
  constructor(private readonly schema: Schema) {
    super(schema);
  }

  private readScalar(table: Table, fieldName: string): number|BigInt|null {
    return this.parser.readScalar(table, fieldName);
  }

  addMessage(data: Uint8Array, time: number): void {
    const message = Visualization.getRootAsVisualization(new ByteBuffer(data));
    for (let ii = 0; ii < message.targetsLength(); ++ii) {
      const target = message.targets(ii);
      const time = Number(target.imageMonotonicTimestampNs()) * 1e-9;
      if (time == 0) {
        console.log('Dropping message without populated time?');
        continue;
      }
      const table = Table.getNamedTable(
          target.bb, this.schema, 'y2024.localizer.TargetEstimateDebug', target.bb_pos);
      this.messages.push(new TimestampedMessage(table, time));
    }
  }
}

export function plotVision(conn: Connection, element: Element): void {
  const aosPlotter = new AosPlotter(conn);

  const targets = [];
  const targetLabels = [];
  for (const orin of ['orin1', 'imu']) {
    for (const camera of ['camera0', 'camera1']) {
      targetLabels.push(orin + ' ' + camera);
      targets.push(aosPlotter.addRawMessageSource(
          '/' + orin + '/' + camera, 'y2024.localizer.Visualization',
          new VisionMessageHandler(
              conn.getSchema('y2024.localizer.Visualization'))));
    }
  }
  const localizerStatus = aosPlotter.addMessageSource(
      '/localizer', 'y2024.localizer.Status');
  const localizerOutput = aosPlotter.addMessageSource(
      '/localizer', 'frc971.controls.LocalizerOutput');

  const statsPlot = aosPlotter.addPlot(element);
  statsPlot.plot.getAxisLabels().setTitle('Statistics');
  statsPlot.plot.getAxisLabels().setXLabel(TIME);
  statsPlot.plot.getAxisLabels().setYLabel('[bool, enum]');

  statsPlot
      .addMessageLine(localizerStatus, ['statistics[]', 'total_accepted'])
      .setDrawLine(false)
      .setColor(BLUE);
  statsPlot
      .addMessageLine(localizerStatus, ['statistics[]', 'total_candidates'])
      .setDrawLine(false)
      .setColor(RED);

  const rejectionPlot = aosPlotter.addPlot(element);
  rejectionPlot.plot.getAxisLabels().setTitle('Rejection Reasons');
  rejectionPlot.plot.getAxisLabels().setXLabel(TIME);
  rejectionPlot.plot.getAxisLabels().setYLabel('[bool, enum]');

  for (let ii = 0; ii < targets.length; ++ii) {
    rejectionPlot.addMessageLine(targets[ii], ['rejection_reason'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }

  const xPlot = aosPlotter.addPlot(element);
  xPlot.plot.getAxisLabels().setTitle('X Position');
  xPlot.plot.getAxisLabels().setXLabel(TIME);
  xPlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    xPlot.addMessageLine(targets[ii], ['implied_robot_x'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }
  xPlot.addMessageLine(localizerOutput, ['x'])
      .setDrawLine(false)
      .setColor(BLUE);

  const correctionXPlot = aosPlotter.addPlot(element);
  correctionXPlot.plot.getAxisLabels().setTitle('X Corrections');
  correctionXPlot.plot.getAxisLabels().setXLabel(TIME);
  correctionXPlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    correctionXPlot.addMessageLine(targets[ii], ['correction_x'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }

  const yPlot = aosPlotter.addPlot(element);
  yPlot.plot.getAxisLabels().setTitle('Y Position');
  yPlot.plot.getAxisLabels().setXLabel(TIME);
  yPlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    yPlot.addMessageLine(targets[ii], ['implied_robot_y'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }
  yPlot.addMessageLine(localizerOutput, ['y'])
      .setDrawLine(false)
      .setColor(BLUE);

  const correctionYPlot = aosPlotter.addPlot(element);
  correctionYPlot.plot.getAxisLabels().setTitle('Y Corrections');
  correctionYPlot.plot.getAxisLabels().setXLabel(TIME);
  correctionYPlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    correctionYPlot.addMessageLine(targets[ii], ['correction_y'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }

  const thetaPlot = aosPlotter.addPlot(element);
  thetaPlot.plot.getAxisLabels().setTitle('Yaw');
  thetaPlot.plot.getAxisLabels().setXLabel(TIME);
  thetaPlot.plot.getAxisLabels().setYLabel('[m]');

  for (let ii = 0; ii < targets.length; ++ii) {
    thetaPlot.addMessageLine(targets[ii], ['implied_robot_theta'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }
  thetaPlot.addMessageLine(localizerOutput, ['theta'])
      .setDrawLine(false)
      .setColor(BLUE);

  const correctionThetaPlot = aosPlotter.addPlot(element);
  correctionThetaPlot.plot.getAxisLabels().setTitle('Theta Corrections');
  correctionThetaPlot.plot.getAxisLabels().setXLabel(TIME);
  correctionThetaPlot.plot.getAxisLabels().setYLabel('[rad]');

  for (let ii = 0; ii < targets.length; ++ii) {
    correctionThetaPlot.addMessageLine(targets[ii], ['correction_theta'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }

  const aprilTagPlot = aosPlotter.addPlot(element);
  aprilTagPlot.plot.getAxisLabels().setTitle('April Tag IDs');
  aprilTagPlot.plot.getAxisLabels().setXLabel(TIME);
  aprilTagPlot.plot.getAxisLabels().setYLabel('[id]');

  for (let ii = 0; ii < targets.length; ++ii) {
    aprilTagPlot.addMessageLine(targets[ii], ['april_tag'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }

  const imageAgePlot = aosPlotter.addPlot(element);
  imageAgePlot.plot.getAxisLabels().setTitle('Image Age');
  imageAgePlot.plot.getAxisLabels().setXLabel(TIME);
  imageAgePlot.plot.getAxisLabels().setYLabel('[sec]');

  for (let ii = 0; ii < targets.length; ++ii) {
    imageAgePlot.addMessageLine(targets[ii], ['image_age_sec'])
        .setDrawLine(false)
        .setColor(PI_COLORS[ii])
        .setLabel(targetLabels[ii]);
  }
}
