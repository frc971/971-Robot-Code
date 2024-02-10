import {ByteBuffer} from 'flatbuffers'
import {ClientStatistics} from '../../aos/network/message_bridge_client_generated'
import {ServerStatistics, State as ConnectionState} from '../../aos/network/message_bridge_server_generated'
import {Connection} from '../../aos/network/www/proxy'
import {ZeroingError} from '../../frc971/control_loops/control_loops_generated'
import {Position as DrivetrainPosition} from '../../frc971/control_loops/drivetrain/drivetrain_position_generated'
import {CANPosition as DrivetrainCANPosition} from '../../frc971/control_loops/drivetrain/drivetrain_can_position_generated'
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated'
import {LocalizerOutput} from '../../frc971/control_loops/drivetrain/localization/localizer_output_generated'
import {TargetMap} from '../../frc971/vision/target_map_generated'


import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 29 * IN_TO_M;
const ROBOT_LENGTH = 32 * IN_TO_M;

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private fieldImage: HTMLImageElement = new Image();
  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.fieldImage.src = '2024.png';
  }

  drawField(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.scale(1.0, -1.0);
    ctx.drawImage(
        this.fieldImage, 0, 0, this.fieldImage.width, this.fieldImage.height,
        -FIELD_EDGE_X, -FIELD_SIDE_Y, FIELD_LENGTH, FIELD_WIDTH);
    ctx.restore();
  }

  draw(): void {
    this.reset();
    this.drawField();

    window.requestAnimationFrame(() => this.draw());
  }

  reset(): void {
    const ctx = this.canvas.getContext('2d');
    // Empty space from the canvas boundary to the image
    const IMAGE_PADDING = 10;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const size = window.innerHeight * 0.9;
    ctx.canvas.height = size;
    const width = size / 2 + 20;
    ctx.canvas.width = width;
    ctx.clearRect(0, 0, size, width);

    // Translate to center of display.
    ctx.translate(width / 2, size / 2);
    // Coordinate system is:
    // x -> forward.
    // y -> to the left.
    ctx.rotate(-Math.PI / 2);
    ctx.scale(1, -1);

    const M_TO_PX = (size - IMAGE_PADDING) / FIELD_LENGTH;
    ctx.scale(M_TO_PX, M_TO_PX);
    ctx.lineWidth = 1 / M_TO_PX;
  }
}
