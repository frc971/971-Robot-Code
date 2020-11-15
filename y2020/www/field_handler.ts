import {aos} from 'aos/configuration_generated';
import {aos} from 'aos/network/connect_generated';
import {Connection} from 'aos/network/www/proxy';
import {frc971} from 'frc971/control_loops/drivetrain/drivetrain_status_generated';
import {frc971} from 'y2020/vision/sift/sift_generated';

import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const DS_WIDTH = 69 * IN_TO_M;
const DS_ANGLE = 20 * Math.PI / 180;
const DS_END_X = FIELD_EDGE_X - DS_WIDTH * Math.sin(DS_ANGLE);
const DS_INSIDE_Y = FIELD_SIDE_Y - DS_WIDTH * Math.cos(DS_ANGLE);

const TRENCH_X = 108 * IN_TO_M;
const TRENCH_WIDTH = 55.5 * IN_TO_M;
const TRENCH_INSIDE = FIELD_SIDE_Y - TRENCH_WIDTH;

const SPINNER_LENGTH = 30 * IN_TO_M;
const SPINNER_TOP_X = 374.59 * IN_TO_M - FIELD_EDGE_X;
const SPINNER_BOTTOM_X = SPINNER_TOP_X - SPINNER_LENGTH;

const SHIELD_BOTTOM_X = -116 * IN_TO_M;
const SHIELD_BOTTOM_Y = 43.75 * IN_TO_M;

const SHIELD_TOP_X = 116 * IN_TO_M;
const SHIELD_TOP_Y = -43.75 * IN_TO_M;

const SHIELD_RIGHT_X = -51.06 * IN_TO_M;
const SHIELD_RIGHT_Y = -112.88 * IN_TO_M;

const SHIELD_LEFT_X = 51.06 * IN_TO_M;
const SHIELD_LEFT_Y = 112.88 * IN_TO_M;

const SHIELD_CENTER_TOP_X = (SHIELD_TOP_X + SHIELD_LEFT_X) / 2
const SHIELD_CENTER_TOP_Y = (SHIELD_TOP_Y + SHIELD_LEFT_Y) / 2

const SHIELD_CENTER_BOTTOM_X = (SHIELD_BOTTOM_X + SHIELD_RIGHT_X) / 2
const SHIELD_CENTER_BOTTOM_Y = (SHIELD_BOTTOM_Y + SHIELD_RIGHT_Y) / 2

const INITIATION_X = FIELD_EDGE_X - 120 * IN_TO_M;

const TARGET_ZONE_TIP_X = FIELD_EDGE_X - 30 * IN_TO_M;
const TARGET_ZONE_WIDTH = 48 * IN_TO_M;
const LOADING_ZONE_WIDTH = 60 * IN_TO_M;

const ROBOT_WIDTH = 28 * IN_TO_M;
const ROBOT_LENGTH = 30 * IN_TO_M;

/**
 * All the messages that are required to display camera information on the field.
 * Messages not readable on the server node are ignored.
 */
const REQUIRED_CHANNELS = [
  {
    name: '/pi1/camera',
    type: 'frc971.vision.sift.ImageMatchResult',
  },
  {
    name: '/pi2/camera',
    type: 'frc971.vision.sift.ImageMatchResult',
  },
  {
    name: '/pi3/camera',
    type: 'frc971.vision.sift.ImageMatchResult',
  },
  {
    name: '/pi4/camera',
    type: 'frc971.vision.sift.ImageMatchResult',
  },
  {
    name: '/pi5/camera',
    type: 'frc971.vision.sift.ImageMatchResult',
  },
  {
    name: '/drivetrain',
    type: 'frc971.control_loops.drivetrain.Status',
  },
];

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private imageMatchResult: frc971.vision.sift.ImageMatchResult|null = null;
  private drivetrainStatus: DrivetrianStatus|null = null;

  constructor(private readonly connection: Connection) {
    document.body.appendChild(this.canvas);

    this.connection.addConfigHandler(() => {
      this.sendConnect();
    });
    this.connection.addHandler(
        frc971.vision.sift.ImageMatchResult.getFullyQualifiedName(), (res) => {
          this.handleImageMatchResult(res);
        });
    this.connection.addHandler(frc971.control_loops.drivetrain.Status.getFullyQualifiedName(), (data) => {
      this.handleDrivetrainStatus(data);
    });
  }

  private handleImageMatchResult(data: Uint8Array): void {
    const fbBuffer = new flatbuffers.ByteBuffer(data);
    this.imageMatchResult =
        frc971.vision.sift.ImageMatchResult.getRootAsImageMatchResult(fbBuffer);
  }

  private handleDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new flatbuffers.ByteBuffer(data);
    this.drivetrainStatus = frc971.control_loops.drivetrain.Status.getRootAsStatus(fbBuffer);
  }

  private sendConnect(): void {
    const builder = new flatbuffers.Builder(512);
    const channels: flatbuffers.Offset[] = [];
    for (const channel of REQUIRED_CHANNELS) {
      const nameFb = builder.createString(channel.name);
      const typeFb = builder.createString(channel.type);
      aos.Channel.startChannel(builder);
      aos.Channel.addName(builder, nameFb);
      aos.Channel.addType(builder, typeFb);
      const channelFb = aos.Channel.endChannel(builder);
      channels.push(channelFb);
    }

    const channelsFb = aos.message_bridge.Connect.createChannelsToTransferVector(builder, channels);
    aos.message_bridge.Connect.startConnect(builder);
    aos.message_bridge.Connect.addChannelsToTransfer(builder, channelsFb);
    const connect = aos.message_bridge.Connect.endConnect(builder);
    builder.finish(connect);
    this.connection.sendConnectMessage(builder);
  }

  drawField(): void {
    const MY_COLOR = 'red';
    const OTHER_COLOR = 'blue';
    const ctx = this.canvas.getContext('2d');
    // draw perimiter
    ctx.beginPath();
    ctx.moveTo(FIELD_EDGE_X, DS_INSIDE_Y);
    ctx.lineTo(DS_END_X, FIELD_SIDE_Y);
    ctx.lineTo(-DS_END_X, FIELD_SIDE_Y);
    ctx.lineTo(-FIELD_EDGE_X, DS_INSIDE_Y);
    ctx.lineTo(-FIELD_EDGE_X, -DS_INSIDE_Y);
    ctx.lineTo(-DS_END_X, -FIELD_SIDE_Y);
    ctx.lineTo(DS_END_X, -FIELD_SIDE_Y);
    ctx.lineTo(FIELD_EDGE_X, -DS_INSIDE_Y);
    ctx.lineTo(FIELD_EDGE_X, DS_INSIDE_Y);
    ctx.stroke();

    // draw shield generator
    ctx.beginPath();
    ctx.moveTo(SHIELD_BOTTOM_X, SHIELD_BOTTOM_Y);
    ctx.lineTo(SHIELD_RIGHT_X, SHIELD_RIGHT_Y);
    ctx.lineTo(SHIELD_TOP_X, SHIELD_TOP_Y);
    ctx.lineTo(SHIELD_LEFT_X, SHIELD_LEFT_Y);
    ctx.lineTo(SHIELD_BOTTOM_X, SHIELD_BOTTOM_Y);
    ctx.moveTo(SHIELD_CENTER_TOP_X, SHIELD_CENTER_TOP_Y);
    ctx.lineTo(SHIELD_CENTER_BOTTOM_X, SHIELD_CENTER_BOTTOM_Y);
    ctx.stroke();

    this.drawHalfField(ctx, 'red');
    ctx.rotate(Math.PI);
    this.drawHalfField(ctx, 'blue');
    ctx.rotate(Math.PI);
  }

  drawHalfField(ctx, color: string): void {
    // trenches
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(TRENCH_X, FIELD_SIDE_Y);
    ctx.lineTo(TRENCH_X, TRENCH_INSIDE);
    ctx.lineTo(-TRENCH_X, TRENCH_INSIDE);
    ctx.lineTo(-TRENCH_X, FIELD_SIDE_Y);
    ctx.stroke();

    ctx.strokeStyle = 'black';
    ctx.beginPath();
    ctx.moveTo(SPINNER_TOP_X, FIELD_SIDE_Y);
    ctx.lineTo(SPINNER_TOP_X, TRENCH_INSIDE);
    ctx.lineTo(SPINNER_BOTTOM_X, TRENCH_INSIDE);
    ctx.lineTo(SPINNER_BOTTOM_X, FIELD_SIDE_Y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(INITIATION_X, FIELD_SIDE_Y);
    ctx.lineTo(INITIATION_X, -FIELD_SIDE_Y);
    ctx.stroke();

    // target/loading
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(FIELD_EDGE_X, DS_INSIDE_Y);
    ctx.lineTo(TARGET_ZONE_TIP_X, DS_INSIDE_Y - 0.5 * TARGET_ZONE_WIDTH);
    ctx.lineTo(FIELD_EDGE_X, DS_INSIDE_Y - TARGET_ZONE_WIDTH);

    ctx.moveTo(-FIELD_EDGE_X, DS_INSIDE_Y);
    ctx.lineTo(-TARGET_ZONE_TIP_X, DS_INSIDE_Y - 0.5 * LOADING_ZONE_WIDTH);
    ctx.lineTo(-FIELD_EDGE_X, DS_INSIDE_Y - LOADING_ZONE_WIDTH);
    ctx.stroke();
  }

  drawCamera(x: number, y: number, theta: number): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.beginPath();
    ctx.moveTo(0.5, 0.5);
    ctx.lineTo(0, 0);
    ctx.lineTo(0.5, -0.5);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, 0, 0.25, -Math.PI/4, Math.PI/4);
    ctx.stroke();
    ctx.restore();
  }

  drawRobot(x: number, y: number, theta: number): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.rect(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, ROBOT_LENGTH, ROBOT_WIDTH);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(ROBOT_LENGTH / 2, 0);
    ctx.stroke();
    ctx.restore();
  }

  draw(): void  {
    this.reset();
    this.drawField();
    //draw cameras
    if (this.imageMatchResult) {
      for (const i = 0; i < this.imageMatchResult.cameraPosesLength(); i++) {
        const pose = this.imageMatchResult.cameraPoses(i);
        const mat = pose.fieldToCamera();
        const x = mat.data(3);
        const y = mat.data(7);
        const theta = Math.atan2(
            -mat.data(8),
            Math.sqrt(Math.pow(mat.data(9), 2) + Math.pow(mat.data(10), 2)));
        this.drawCamera(x, y, theta);
      }
    }

    if (this.drivetrainStatus) {
      this.drawRobot(
          this.drivetrainStatus.x(), this.drivetrainStatus.y(),
          this.drivetrainStatus.theta());
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
