import * as configuration from 'org_frc971/aos/configuration_generated';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import * as flatbuffers_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import * as drivetrain from 'org_frc971/frc971/control_loops/drivetrain/drivetrain_status_generated';
import * as sift from 'org_frc971/y2020/vision/sift/sift_generated';
import * as web_proxy from 'org_frc971/aos/network/web_proxy_generated';
import * as ss from 'org_frc971/y2020/control_loops/superstructure/superstructure_status_generated'

import DrivetrainStatus = drivetrain.frc971.control_loops.drivetrain.Status;
import SuperstructureStatus = ss.y2020.control_loops.superstructure.Status;
import ImageMatchResult = sift.frc971.vision.sift.ImageMatchResult;
import Channel = configuration.aos.Channel;
import SubscriberRequest = web_proxy.aos.web_proxy.SubscriberRequest;
import ChannelRequest = web_proxy.aos.web_proxy.ChannelRequest;
import TransferMethod = web_proxy.aos.web_proxy.TransferMethod;

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


export class FieldHandler {
  private canvas = document.createElement('canvas');
  private imageMatchResult =  new Map<string, ImageMatchResult>();
  private drivetrainStatus: DrivetrainStatus|null = null;
  private superstructureStatus: SuperstructureStatus|null = null;
  private x: HTMLDivElement = (document.getElementById('x') as HTMLDivElement);
  private y: HTMLDivElement = (document.getElementById('y') as HTMLDivElement);
  private theta: HTMLDivElement = (document.getElementById('theta') as HTMLDivElement);
  private shotDistance: HTMLDivElement = (document.getElementById('shot_distance') as HTMLDivElement);
  private finisher: HTMLDivElement = (document.getElementById('finisher') as HTMLDivElement);
  private leftAccelerator: HTMLDivElement = (document.getElementById('left_accelerator') as HTMLDivElement);
  private rightAccelerator: HTMLDivElement = (document.getElementById('right_accelerator') as HTMLDivElement);
  private innerPort: HTMLDivElement = (document.getElementById('inner_port') as HTMLDivElement);
  private hood: HTMLDivElement = (document.getElementById('hood') as HTMLDivElement);
  private turret: HTMLDivElement = (document.getElementById('turret') as HTMLDivElement);
  private intake: HTMLDivElement = (document.getElementById('intake') as HTMLDivElement);

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.connection.addConfigHandler(() => {
      // Go through and register handlers for both all the individual pis as
      // well as the local pi. Depending on the node that we are running on,
      // different subsets of these will be available.
      for (const prefix of ['', '/pi1', '/pi2', '/pi3', '/pi4']) {
        this.connection.addHandler(
            prefix + '/camera', ImageMatchResult.getFullyQualifiedName(), (res) => {
              this.handleImageMatchResult(prefix, res);
            });
      }
      this.connection.addHandler(
          '/drivetrain', DrivetrainStatus.getFullyQualifiedName(), (data) => {
            this.handleDrivetrainStatus(data);
          });
      this.connection.addHandler(
          '/superstructure', SuperstructureStatus.getFullyQualifiedName(),
          (data) => {
            this.handleSuperstructureStatus(data);
          });
    });
  }

  private handleImageMatchResult(prefix: string, data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.imageMatchResult.set(
        prefix,
        ImageMatchResult.getRootAsImageMatchResult(
            fbBuffer as unknown as flatbuffers.ByteBuffer));
  }

  private handleDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.drivetrainStatus = DrivetrainStatus.getRootAsStatus(
        fbBuffer as unknown as flatbuffers.ByteBuffer);
  }

  private handleSuperstructureStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.superstructureStatus = SuperstructureStatus.getRootAsStatus(
        fbBuffer as unknown as flatbuffers.ByteBuffer);
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
    ctx.lineTo(100.0, 0);
    ctx.lineTo(0, 0);
    ctx.lineTo(0.5, -0.5);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, 0, 0.25, -Math.PI / 4, Math.PI / 4);
    ctx.stroke();
    ctx.restore();
  }

  drawRobot(x: number, y: number, theta: number, turret: number|null): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(theta);
    ctx.rect(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, ROBOT_LENGTH, ROBOT_WIDTH);
    ctx.stroke();
    if (turret) {
      ctx.save();
      ctx.rotate(turret + Math.PI);
      const turretRadius = ROBOT_WIDTH / 4.0;
      ctx.strokeStyle = "red";
      // Draw circle for turret.
      ctx.beginPath();
      ctx.arc(0, 0, turretRadius, 0, 2.0 * Math.PI);
      ctx.stroke();
      // Draw line in circle to show forwards.
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(1000.0 * turretRadius, 0);
      ctx.stroke();
      ctx.restore();
    }
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(100.0 * ROBOT_LENGTH / 2, 0);
    ctx.stroke();
    ctx.restore();
  }

  setZeroing(div: HTMLDivElement): void {
        div.innerHTML = "zeroing";
        div.classList.remove("faulted");
        div.classList.add("zeroing");
  }
  setEstopped(div: HTMLDivElement): void {
        div.innerHTML = "estopped";
        div.classList.add("faulted");
        div.classList.remove("zeroing");
  }
  setValue(div: HTMLDivElement, val: Number): void {
        div.innerHTML = val.toFixed(4);
        div.classList.remove("faulted");
        div.classList.remove("zeroing");
  }

  draw(): void {
    this.reset();
    this.drawField();
    // draw cameras
    for (const keyPair of this.imageMatchResult) {
      const value = keyPair[1];
      for (let i = 0; i < value.cameraPosesLength(); i++) {
        const pose = value.cameraPoses(i);
        const mat = pose.fieldToCamera();
        // Matrix layout:
        // [0,  1,  2,  3]
        // [4,  5,  6,  7]
        // [8,  9,  10, 11]
        // [12, 13, 14, 15]
        const x = mat.data(3);
        const y = mat.data(7);
        const theta = Math.atan2(mat.data(6), mat.data(2));
        this.drawCamera(x, y, theta);
      }
    }

    if (this.drivetrainStatus) {
      if (!this.drivetrainStatus.zeroing().zeroed()) {
        this.setZeroing(this.x);
        this.setZeroing(this.y);
        this.setZeroing(this.theta);
      } else if (this.drivetrainStatus.zeroing().faulted()) {
        this.setEstopped(this.x);
        this.setEstopped(this.y);
        this.setEstopped(this.theta);
      } else {
        this.setValue(this.x, this.drivetrainStatus.x());
        this.setValue(this.y, this.drivetrainStatus.y());
        this.setValue(this.theta, this.drivetrainStatus.theta());
      }

      this.shotDistance.innerHTML = this.superstructureStatus.aimer().shotDistance().toFixed(2);
      this.finisher.innerHTML = this.superstructureStatus.shooter().finisher().angularVelocity().toFixed(2);
      this.leftAccelerator.innerHTML = this.superstructureStatus.shooter().acceleratorLeft().angularVelocity().toFixed(2);
      this.rightAccelerator.innerHTML = this.superstructureStatus.shooter().acceleratorRight().angularVelocity().toFixed(2);
      if (this.superstructureStatus.aimer().aimingForInnerPort()) {
        this.innerPort.innerHTML = "true";
      } else {
        this.innerPort.innerHTML = "false";
      }
      if (!this.superstructureStatus.hood().zeroed()) {
        this.setZeroing(this.hood);
      } else if (this.superstructureStatus.hood().estopped()) {
        this.setEstopped(this.hood);
      } else {
        this.setValue(this.hood, this.superstructureStatus.hood().estimatorState().position());
      }
      if (!this.superstructureStatus.turret().zeroed()) {
        this.setZeroing(this.turret);
      } else if (this.superstructureStatus.turret().estopped()) {
        this.setEstopped(this.turret);
      } else {
        this.setValue(this.turret, this.superstructureStatus.turret().estimatorState().position());
      }
      if (!this.superstructureStatus.intake().zeroed()) {
        this.setZeroing(this.intake);
      } else if (this.superstructureStatus.intake().estopped()) {
        this.setEstopped(this.intake);
      } else {
        this.setValue(this.intake, this.superstructureStatus.intake().estimatorState().position());
      }
      this.drawRobot(
          this.drivetrainStatus.x(), this.drivetrainStatus.y(),
          this.drivetrainStatus.theta(),
          this.superstructureStatus ?
              this.superstructureStatus.turret().position() :
              null);
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
