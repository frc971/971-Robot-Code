import {ByteBuffer} from 'flatbuffers';
import {Connection} from '../../aos/network/www/proxy';
import {Status as SuperstructureStatus} from '../control_loops/superstructure/superstructure_status_generated'
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated';

import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 34 * IN_TO_M;
const ROBOT_LENGTH = 36 * IN_TO_M;

const PI_COLORS = ['#ff00ff', '#ffff00', '#00ffff', '#ffa500'];

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private drivetrainStatus: DrivetrainStatus|null = null;
  private superstructureStatus: SuperstructureStatus|null = null;

  // Image information indexed by timestamp (seconds since the epoch), so that
  // we can stop displaying images after a certain amount of time.
    private x: HTMLElement = (document.getElementById('x') as HTMLElement);
  private y: HTMLElement = (document.getElementById('y') as HTMLElement);
  private theta: HTMLElement =
      (document.getElementById('theta') as HTMLElement);
  private superstructureState: HTMLElement =
      (document.getElementById('superstructure_state') as HTMLElement);
  private imagesAcceptedCounter: HTMLElement =
      (document.getElementById('images_accepted') as HTMLElement);
  private imagesRejectedCounter: HTMLElement =
      (document.getElementById('images_rejected') as HTMLElement);
  private fieldImage: HTMLImageElement = new Image();

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.fieldImage.src = "2022.png";

    for (let ii = 0; ii < PI_COLORS.length; ++ii) {
      const legendEntry = document.createElement('div');
      legendEntry.style.color = PI_COLORS[ii];
      legendEntry.innerHTML = 'PI' + (ii + 1).toString()
      document.getElementById('legend').appendChild(legendEntry);
    }

    this.connection.addConfigHandler(() => {
      // Visualization message is reliable so that we can see *all* the vision
      // matches.
      this.connection.addHandler(
          '/drivetrain', "frc971.control_loops.drivetrain.Status", (data) => {
            this.handleDrivetrainStatus(data);
          });
      this.connection.addHandler(
          '/superstructure', "y2023.control_loops.superstructure.Status",
          (data) => {
            this.handleSuperstructureStatus(data);
          });
    });
  }

  private handleDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainStatus = DrivetrainStatus.getRootAsStatus(fbBuffer);
  }

  private handleSuperstructureStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.superstructureStatus = SuperstructureStatus.getRootAsStatus(fbBuffer);
  }

  drawField(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.scale(-1.0, 1.0);
    ctx.drawImage(
        this.fieldImage, 0, 0, this.fieldImage.width, this.fieldImage.height,
        -FIELD_EDGE_X, -FIELD_SIDE_Y, FIELD_LENGTH, FIELD_WIDTH);
    ctx.restore();
  }

  drawCamera(
      x: number, y: number, theta: number, color: string = 'blue',
      extendLines: boolean = true): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(0.5, 0.5);
    ctx.lineTo(0, 0);
    if (extendLines) {
      ctx.lineTo(100.0, 0);
      ctx.lineTo(0, 0);
    }
    ctx.lineTo(0.5, -0.5);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, 0, 0.25, -Math.PI / 4, Math.PI / 4);
    ctx.stroke();
    ctx.restore();
  }

  drawRobot(
      x: number, y: number, theta: number, turret: number|null,
      color: string = 'blue', dashed: boolean = false,
      extendLines: boolean = true): void {
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
    if (extendLines) {
      ctx.lineTo(1000.0, 0);
    } else {
      ctx.lineTo(ROBOT_LENGTH / 2.0, 0);
    }
    ctx.stroke();

    ctx.restore();
  }

  setZeroing(div: HTMLElement): void {
    div.innerHTML = 'zeroing';
    div.classList.remove('faulted');
    div.classList.add('zeroing');
    div.classList.remove('near');
  }

  setEstopped(div: HTMLElement): void {
    div.innerHTML = 'estopped';
    div.classList.add('faulted');
    div.classList.remove('zeroing');
    div.classList.remove('near');
  }

  setTargetValue(
      div: HTMLElement, target: number, val: number, tolerance: number): void {
    div.innerHTML = val.toFixed(4);
    div.classList.remove('faulted');
    div.classList.remove('zeroing');
    if (Math.abs(target - val) < tolerance) {
      div.classList.add('near');
    } else {
      div.classList.remove('near');
    }
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

    // Draw the matches with debugging information from the localizer.
    const now = Date.now() / 1000.0;
    
    if (this.drivetrainStatus && this.drivetrainStatus.trajectoryLogging()) {
      this.drawRobot(
          this.drivetrainStatus.trajectoryLogging().x(),
          this.drivetrainStatus.trajectoryLogging().y(),
          this.drivetrainStatus.trajectoryLogging().theta(), null, "#000000FF",
          false);
    }

    window.requestAnimationFrame(() => this.draw());
  }

  reset(): void {
    const ctx = this.canvas.getContext('2d');
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

    const M_TO_PX = (size - 10) / FIELD_LENGTH;
    ctx.scale(M_TO_PX, M_TO_PX);
    ctx.lineWidth = 1 / M_TO_PX;
  }
}
