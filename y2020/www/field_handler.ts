import {ByteBuffer} from 'flatbuffers';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import * as flatbuffers_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {Status as DrivetrainStatus} from 'org_frc971/frc971/control_loops/drivetrain/drivetrain_status_generated';
import {LocalizerDebug, RejectionReason, ImageMatchDebug} from 'org_frc971/y2020/control_loops/drivetrain/localizer_debug_generated';
import {Status as SuperstructureStatus, FlywheelControllerStatus} from 'org_frc971/y2020/control_loops/superstructure/superstructure_status_generated'
import {ImageMatchResult} from 'org_frc971/y2020/vision/sift/sift_generated';

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

const ROBOT_WIDTH = 34 * IN_TO_M;
const ROBOT_LENGTH = 36 * IN_TO_M;

const PI_COLORS = ['#ff00ff', '#ffff00', '#000000', '#00ffff', '#ffa500'];

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private imageMatchResult = new Map<string, ImageMatchResult>();
  private drivetrainStatus: DrivetrainStatus|null = null;
  private superstructureStatus: SuperstructureStatus|null = null;

  // Image information indexed by timestamp (seconds since the epoch), so that
  // we can stop displaying images after a certain amount of time.
  private localizerImageMatches = new Map<number, LocalizerDebug>();
  private outerTarget: HTMLElement =
      (document.getElementById('outer_target') as HTMLElement);
  private innerTarget: HTMLElement =
      (document.getElementById('inner_target') as HTMLElement);
  private x: HTMLElement = (document.getElementById('x') as HTMLElement);
  private y: HTMLElement = (document.getElementById('y') as HTMLElement);
  private theta: HTMLElement =
      (document.getElementById('theta') as HTMLElement);
  private shotDistance: HTMLElement =
      (document.getElementById('shot_distance') as HTMLElement);
  private finisher: HTMLElement =
      (document.getElementById('finisher') as HTMLElement);
  private leftAccelerator: HTMLElement =
      (document.getElementById('left_accelerator') as HTMLElement);
  private rightAccelerator: HTMLElement =
      (document.getElementById('right_accelerator') as HTMLElement);
  private innerPort: HTMLElement =
      (document.getElementById('inner_port') as HTMLElement);
  private hood: HTMLElement = (document.getElementById('hood') as HTMLElement);
  private turret: HTMLElement =
      (document.getElementById('turret') as HTMLElement);
  private ballsShot: HTMLElement =
      (document.getElementById('balls_shot') as HTMLElement);
  private intake: HTMLElement =
      (document.getElementById('intake') as HTMLElement);
  private imagesAcceptedCounter: HTMLElement =
      (document.getElementById('images_accepted') as HTMLElement);
  private imagesRejectedCounter: HTMLElement =
      (document.getElementById('images_rejected') as HTMLElement);
  private rejectionReasonCells: HTMLElement[] = [];

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    for (const value in RejectionReason) {
      // Typescript generates an iterator that produces both numbers and
      // strings... don't do anything on the string iterations.
      if (isNaN(Number(value))) {
        continue;
      }
      const row = document.createElement('div');
      const nameCell = document.createElement('div');
      nameCell.innerHTML = RejectionReason[value];
      row.appendChild(nameCell);
      const valueCell = document.createElement('div');
      valueCell.innerHTML = 'NA';
      this.rejectionReasonCells.push(valueCell);
      row.appendChild(valueCell);
      document.getElementById('vision_readouts').appendChild(row);
    }

    for (let ii = 0; ii < PI_COLORS.length; ++ii) {
      const legendEntry = document.createElement('div');
      legendEntry.style.color = PI_COLORS[ii];
      legendEntry.innerHTML = 'PI' + (ii + 1).toString()
      document.getElementById('legend').appendChild(legendEntry);
    }

    this.connection.addConfigHandler(() => {
      // Go through and register handlers for both all the individual pis as
      // well as the local pi. Depending on the node that we are running on,
      // different subsets of these will be available.
      for (const prefix of ['', '/pi1', '/pi2', '/pi3', '/pi4', '/pi5']) {
        this.connection.addHandler(
            prefix + '/camera', ImageMatchResult.getFullyQualifiedName(),
            (res) => {
              this.handleImageMatchResult(prefix, res);
            });
      }
      this.connection.addHandler(
          '/drivetrain', LocalizerDebug.getFullyQualifiedName(), (data) => {
            this.handleLocalizerDebug(data);
          });
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
        prefix, ImageMatchResult.getRootAsImageMatchResult(fbBuffer));
  }

  private handleLocalizerDebug(data: Uint8Array): void {
    const now = Date.now() / 1000.0;

    const fbBuffer = new ByteBuffer(data);
    this.localizerImageMatches.set(
        now, LocalizerDebug.getRootAsLocalizerDebug(fbBuffer));

    const debug = this.localizerImageMatches.get(now);

    if (debug.statistics()) {
      this.imagesAcceptedCounter.innerHTML =
          debug.statistics().totalAccepted().toString();
      this.imagesRejectedCounter.innerHTML =
          (debug.statistics().totalCandidates() -
           debug.statistics().totalAccepted())
              .toString();
      if (debug.statistics().rejectionReasonCountLength() ==
          this.rejectionReasonCells.length) {
        for (let ii = 0; ii < debug.statistics().rejectionReasonCountLength();
             ++ii) {
          this.rejectionReasonCells[ii].innerHTML =
              debug.statistics().rejectionReasonCount(ii).toString();
        }
      } else {
        console.error('Unexpected number of rejection reasons in counter.');
      }
      this.imagesRejectedCounter.innerHTML =
          (debug.statistics().totalCandidates() -
           debug.statistics().totalAccepted())
              .toString();
    }
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

    if (turret) {
      ctx.save();
      ctx.rotate(turret + Math.PI);
      const turretRadius = ROBOT_WIDTH / 4.0;
      ctx.strokeStyle = 'red';
      // Draw circle for turret.
      ctx.beginPath();
      ctx.arc(0, 0, turretRadius, 0, 2.0 * Math.PI);
      ctx.stroke();
      // Draw line in circle to show forwards.
      ctx.beginPath();
      ctx.moveTo(0, 0);
      if (extendLines) {
        ctx.lineTo(1000.0, 0);
      } else {
        ctx.lineTo(turretRadius, 0);
      }
      ctx.stroke();
      ctx.restore();
    }
    ctx.restore();
  }

  setShooter(div: HTMLElement, flywheelStatus: FlywheelControllerStatus): void {
    const currentVelocity = flywheelStatus.angularVelocity();
    const goal = flywheelStatus.angularVelocityGoal();
    // Show it as 'near' if difference between
    if (Math.abs(currentVelocity - goal) < 5) {
      this.setNear(div, currentVelocity.toFixed(2));
      return;
    }
    div.classList.remove('faulted');
    div.classList.remove('zeroing');
    div.classList.remove('near');
    div.innerHTML = currentVelocity.toFixed(2);
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

  setNear(div: HTMLElement, val: string): void {
    div.innerHTML = val;
    div.classList.remove('faulted');
    div.classList.remove('zeroing');
    div.classList.add('near');
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
    for (const [time, value] of this.localizerImageMatches) {
      const age = now - time;
      const kRemovalAge = 2.0;
      if (age > kRemovalAge) {
        this.localizerImageMatches.delete(time);
        continue;
      }
      const ageAlpha = (kRemovalAge - age) / kRemovalAge
      for (let i = 0; i < value.matchesLength(); i++) {
        const imageDebug = value.matches(i);
        const x = imageDebug.impliedRobotX();
        const y = imageDebug.impliedRobotY();
        const theta = imageDebug.impliedRobotTheta();
        const cameraX = imageDebug.cameraX();
        const cameraY = imageDebug.cameraY();
        const cameraTheta = imageDebug.cameraTheta();
        const accepted = imageDebug.accepted();
        // Make camera readings fade over time.
        const alpha = Math.round(255 * ageAlpha).toString(16).padStart(2, '0');
        const dashed = false;
        const acceptedRgb = accepted ? '#00FF00' : '#FF0000';
        const acceptedRgba = acceptedRgb + alpha;
        const cameraRgb = PI_COLORS[imageDebug.camera()];
        const cameraRgba = cameraRgb + alpha;
        this.drawRobot(x, y, theta, null, acceptedRgba, dashed, false);
        this.drawCamera(cameraX, cameraY, cameraTheta, cameraRgba, false);
      }
    }

    // draw cameras from ImageMatchResults directly (helpful when viewing page
    // on the pis individually).
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
        const cameraColor = (keyPair[0].length > 0) ?
            PI_COLORS[Number(keyPair[0][3]) - 1] :
            'blue';
        this.drawCamera(x, y, theta, cameraColor);
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

      if (this.superstructureStatus) {
        this.shotDistance.innerHTML =
            this.superstructureStatus.aimer().shotDistance().toFixed(2);
        this.setShooter(
            this.finisher, this.superstructureStatus.shooter().finisher());
        this.setShooter(
            this.leftAccelerator,
            this.superstructureStatus.shooter().acceleratorLeft());
        this.setShooter(
            this.rightAccelerator,
            this.superstructureStatus.shooter().acceleratorRight());

        if (this.superstructureStatus.aimer().aimingForInnerPort()) {
          this.innerPort.innerHTML = 'true';
          this.outerTarget.classList.remove('targetted');
          this.innerTarget.classList.add('targetted');
        } else {
          this.innerPort.innerHTML = 'false';
          this.outerTarget.classList.add('targetted');
          this.innerTarget.classList.remove('targetted');
        }

        if (!this.superstructureStatus.hood().zeroed()) {
          this.setZeroing(this.hood);
        } else if (this.superstructureStatus.hood().estopped()) {
          this.setEstopped(this.hood);
        } else {
          this.setTargetValue(
              this.hood,
              this.superstructureStatus.hood().unprofiledGoalPosition(),
              this.superstructureStatus.hood().estimatorState().position(),
              1e-3);
        }

        this.ballsShot.innerHTML =
            this.superstructureStatus.shooter().ballsShot().toString();

        if (!this.superstructureStatus.turret().zeroed()) {
          this.setZeroing(this.turret);
        } else if (this.superstructureStatus.turret().estopped()) {
          this.setEstopped(this.turret);
        } else {
          this.setTargetValue(
              this.turret,
              this.superstructureStatus.turret().unprofiledGoalPosition(),
              this.superstructureStatus.turret().estimatorState().position(),
              1e-3);
        }

        if (!this.superstructureStatus.intake().zeroed()) {
          this.setZeroing(this.intake);
        } else if (this.superstructureStatus.intake().estopped()) {
          this.setEstopped(this.intake);
        } else {
          this.setValue(
              this.intake,
              this.superstructureStatus.intake().estimatorState().position());
        }
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
