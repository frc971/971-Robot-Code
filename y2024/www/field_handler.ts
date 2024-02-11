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
  private localizerOutput: LocalizerOutput|null = null;
  private drivetrainStatus: DrivetrainStatus|null = null;
  private drivetrainPosition: DrivetrainPosition|null = null;
  private drivetrainCANPosition: DrivetrainCANPosition|null = null;

  private x: HTMLElement = (document.getElementById('x') as HTMLElement);
  private y: HTMLElement = (document.getElementById('y') as HTMLElement);
  private theta: HTMLElement =
      (document.getElementById('theta') as HTMLElement);

  private fieldImage: HTMLImageElement = new Image();

  private leftDrivetrainEncoder: HTMLElement =
      (document.getElementById('left_drivetrain_encoder') as HTMLElement);
  private rightDrivetrainEncoder: HTMLElement =
      (document.getElementById('right_drivetrain_encoder') as HTMLElement);
  private falconRightFrontPosition: HTMLElement =
      (document.getElementById('falcon_right_front') as HTMLElement);
  private falconRightBackPosition: HTMLElement =
      (document.getElementById('falcon_right_back') as HTMLElement);
  private falconLeftFrontPosition: HTMLElement =
      (document.getElementById('falcon_left_front') as HTMLElement);
  private falconLeftBackPosition: HTMLElement =
      (document.getElementById('falcon_left_back') as HTMLElement);

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.fieldImage.src = '2024.png';

    this.connection.addConfigHandler(() => {

      this.connection.addHandler(
        '/drivetrain', 'frc971.control_loops.drivetrain.Status', (data) => {
          this.handleDrivetrainStatus(data);
        });
      this.connection.addHandler(
        '/drivetrain', 'frc971.control_loops.drivetrain.Position', (data) => {
          this.handleDrivetrainPosition(data);
        });
      this.connection.addHandler(
        '/drivetrain', 'frc971.control_loops.drivetrain.CANPosition', (data) => {
          this.handleDrivetrainCANPosition(data);
        });
      this.connection.addHandler(
        '/localizer', 'frc971.controls.LocalizerOutput', (data) => {
          this.handleLocalizerOutput(data);
        });
      });
  }

  private handleDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainStatus = DrivetrainStatus.getRootAsStatus(fbBuffer);
  }

  private handleDrivetrainPosition(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainPosition = DrivetrainPosition.getRootAsPosition(fbBuffer);
  }

  private handleDrivetrainCANPosition(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainCANPosition = DrivetrainCANPosition.getRootAsCANPosition(fbBuffer);
  }

  private handleLocalizerOutput(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.localizerOutput = LocalizerOutput.getRootAsLocalizerOutput(fbBuffer);
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

  drawRobot(
    x: number, y: number, theta: number, color: string = 'blue',
    dashed: boolean = false): void {
  const ctx = this.canvas.getContext('2d');
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(theta);
  ctx.strokeStyle = color;
  ctx.lineWidth = ROBOT_WIDTH / 10.0;
  if (dashed) {
    ctx.setLineDash([0.05, 0.05]);
  } else {
    // Empty array = solid line.
    ctx.setLineDash([]);
  }
  ctx.rect(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, ROBOT_LENGTH, ROBOT_WIDTH);
  ctx.stroke();

  // Draw line indicating which direction is forwards on the robot.
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(ROBOT_LENGTH / 2.0, 0);
  ctx.stroke();

  ctx.restore();
}

  setZeroing(div: HTMLElement): void {
    div.innerHTML = 'zeroing';
    div.classList.remove('faulted');
    div.classList.add('zeroing');
    div.classList.remove('near');
}

  setValue(div: HTMLElement, val: number): void {
    div.innerHTML = val.toFixed(4);
    div.classList.remove('faulted');
    div.classList.remove('zeroing');
    div.classList.remove('near');
}
  draw(): void {
    this.reset();
    this.drawField();

    if (this.drivetrainPosition) {
        this.leftDrivetrainEncoder.innerHTML =
        this.drivetrainPosition.leftEncoder().toString();

        this.rightDrivetrainEncoder.innerHTML =
        this.drivetrainPosition.rightEncoder().toString();
    }

    if (this.drivetrainCANPosition) {
      this.falconRightFrontPosition.innerHTML =
      this.drivetrainCANPosition.talonfxs(0).position().toString();

      this.falconRightBackPosition.innerHTML =
      this.drivetrainCANPosition.talonfxs(1).position().toString();

      this.falconLeftFrontPosition.innerHTML =
      this.drivetrainCANPosition.talonfxs(2).position().toString();

      this.falconLeftBackPosition.innerHTML =
      this.drivetrainCANPosition.talonfxs(3).position().toString();
    }

    if (this.drivetrainStatus && this.drivetrainStatus.trajectoryLogging()) {
      this.drawRobot(
          this.drivetrainStatus.trajectoryLogging().x(),
          this.drivetrainStatus.trajectoryLogging().y(),
          this.drivetrainStatus.trajectoryLogging().theta(), '#000000FF',
          false);
    }

    if (this.localizerOutput) {
      if (!this.localizerOutput.zeroed()) {
        this.setZeroing(this.x);
        this.setZeroing(this.y);
        this.setZeroing(this.theta);
      } else {
        this.setValue(this.x, this.localizerOutput.x());
        this.setValue(this.y, this.localizerOutput.y());
        this.setValue(this.theta, this.localizerOutput.theta());
      }

      this.drawRobot(
          this.localizerOutput.x(), this.localizerOutput.y(),
          this.localizerOutput.theta());
    }
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
