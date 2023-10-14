import {ByteBuffer} from 'flatbuffers'
import {ClientStatistics} from '../../aos/network/message_bridge_client_generated'
import {ServerStatistics, State as ConnectionState} from '../../aos/network/message_bridge_server_generated'
import {Connection} from '../../aos/network/www/proxy'
import {ZeroingError} from '../../frc971/control_loops/control_loops_generated'
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated'
import {LocalizerOutput} from '../../frc971/control_loops/drivetrain/localization/localizer_output_generated'

import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 25 * IN_TO_M;
const ROBOT_LENGTH = 32 * IN_TO_M;

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private localizerOutput: LocalizerOutput|null = null;
  private drivetrainStatus: DrivetrainStatus|null = null;

  private handleDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainStatus = DrivetrainStatus.getRootAsStatus(fbBuffer);
  }

  private setCurrentNodeState(element: HTMLElement, state: ConnectionState):
      void {
    if (state === ConnectionState.CONNECTED) {
      element.innerHTML = ConnectionState[state];
      element.classList.remove('faulted');
      element.classList.add('connected');
    } else {
      element.innerHTML = ConnectionState[state];
      element.classList.remove('connected');
      element.classList.add('faulted');
    }
  }

  private handleServerStatistics(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    const serverStatistics =
        ServerStatistics.getRootAsServerStatistics(fbBuffer);

    for (let ii = 0; ii < serverStatistics.connectionsLength(); ++ii) {
      const connection = serverStatistics.connections(ii);
      const nodeName = connection.node().name();
      if (!this.serverStatuses.has(nodeName)) {
        this.populateNodeConnections(nodeName);
      }
      this.setCurrentNodeState(
          this.serverStatuses.get(nodeName), connection.state());
    }
  }

  private handleClientStatistics(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    const clientStatistics =
        ClientStatistics.getRootAsClientStatistics(fbBuffer);

    for (let ii = 0; ii < clientStatistics.connectionsLength(); ++ii) {
      const connection = clientStatistics.connections(ii);
      const nodeName = connection.node().name();
      if (!this.clientStatuses.has(nodeName)) {
        this.populateNodeConnections(nodeName);
      }
      this.setCurrentNodeState(
          this.clientStatuses.get(nodeName), connection.state());
    }
  }

  drawField(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.scale(1.0, -1.0);
    ctx.restore();
  }

  drawCamera(x: number, y: number, theta: number, color: string = 'blue'):
      void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(0.5, 0.5);
    ctx.lineTo(0, 0);
    ctx.lineTo(0.5, -0.5);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, 0, 0.25, -Math.PI / 4, Math.PI / 4);
    ctx.stroke();
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

  setEstopped(div: HTMLElement): void {
    div.innerHTML = 'estopped';
    div.classList.add('faulted');
    div.classList.remove('zeroing');
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

    // Draw the matches with debugging information from the localizer.
    const now = Date.now() / 1000.0;

    if (this.drivetrainStatus && this.drivetrainStatus.trajectoryLogging()) {
      this.drawRobot(
          this.drivetrainStatus.trajectoryLogging().x(),
          this.drivetrainStatus.trajectoryLogging().y(),
          this.drivetrainStatus.trajectoryLogging().theta(), '#000000FF',
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
