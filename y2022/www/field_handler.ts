import {ByteBuffer} from 'flatbuffers';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import {IntakeState, Status as SuperstructureStatus, SuperstructureState} from 'org_frc971/y2022/control_loops/superstructure/superstructure_status_generated'
import {LocalizerOutput} from 'org_frc971/y2022/localizer/localizer_output_generated';
import {RejectionReason} from 'org_frc971/y2022/localizer/localizer_status_generated';
import {LocalizerVisualization, TargetEstimateDebug} from 'org_frc971/y2022/localizer/localizer_visualization_generated';

import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 34 * IN_TO_M;
const ROBOT_LENGTH = 36 * IN_TO_M;

const PI_COLORS = ['#ff00ff', '#ffff00', '#00ffff', '#ffa500'];

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private localizerOutput: LocalizerOutput|null = null;
  private superstructureStatus: SuperstructureStatus|null = null;

  // Image information indexed by timestamp (seconds since the epoch), so that
  // we can stop displaying images after a certain amount of time.
  private localizerImageMatches = new Map<number, LocalizerVisualization>();
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
  private turret: HTMLElement =
      (document.getElementById('turret') as HTMLElement);
  private fire: HTMLElement =
      (document.getElementById('fire') as HTMLElement);
  private mpcSolveTime: HTMLElement =
      (document.getElementById('mpc_solve_time') as HTMLElement);
  private mpcHorizon: HTMLElement =
      (document.getElementById('mpc_horizon') as HTMLElement);
  private shotCount: HTMLElement =
      (document.getElementById('shot_count') as HTMLElement);
  private catapult: HTMLElement =
      (document.getElementById('catapult') as HTMLElement);
  private superstructureState: HTMLElement =
      (document.getElementById('superstructure_state') as HTMLElement);
  private intakeState: HTMLElement =
      (document.getElementById('intake_state') as HTMLElement);
  private reseatingInCatapult: HTMLElement =
      (document.getElementById('reseating_in_catapult') as HTMLElement);
  private flippersOpen: HTMLElement =
      (document.getElementById('flippers_open') as HTMLElement);
  private climber: HTMLElement =
      (document.getElementById('climber') as HTMLElement);
  private frontIntake: HTMLElement =
      (document.getElementById('front_intake') as HTMLElement);
  private backIntake: HTMLElement =
      (document.getElementById('back_intake') as HTMLElement);
  private imagesAcceptedCounter: HTMLElement =
      (document.getElementById('images_accepted') as HTMLElement);
  private imagesRejectedCounter: HTMLElement =
      (document.getElementById('images_rejected') as HTMLElement);
  private rejectionReasonCells: HTMLElement[] = [];
  private fieldImage: HTMLImageElement = new Image();

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.fieldImage.src = "2022.png";

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
      // Visualization message is reliable so that we can see *all* the vision
      // matches.
      this.connection.addReliableHandler(
          '/localizer', LocalizerVisualization.getFullyQualifiedName(),
          (data) => {
            this.handleLocalizerDebug(data);
          });
      this.connection.addHandler(
          '/localizer', LocalizerOutput.getFullyQualifiedName(), (data) => {
            this.handleLocalizerOutput(data);
          });
      this.connection.addHandler(
          '/superstructure', SuperstructureStatus.getFullyQualifiedName(),
          (data) => {
            this.handleSuperstructureStatus(data);
          });
    });
  }

  private handleLocalizerDebug(data: Uint8Array): void {
    const now = Date.now() / 1000.0;

    const fbBuffer = new ByteBuffer(data);
    this.localizerImageMatches.set(
        now, LocalizerVisualization.getRootAsLocalizerVisualization(fbBuffer));

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

  private handleLocalizerOutput(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.localizerOutput = LocalizerOutput.getRootAsLocalizerOutput(fbBuffer);
  }

  private handleSuperstructureStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.superstructureStatus = SuperstructureStatus.getRootAsStatus(fbBuffer);
  }

  drawField(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.drawImage(
        this.fieldImage, 0, 0, this.fieldImage.width, this.fieldImage.height,
        -FIELD_EDGE_X, -FIELD_SIDE_Y, FIELD_LENGTH, FIELD_WIDTH);
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

    if (turret !== null) {
      ctx.save();
      ctx.rotate(turret);
      const turretRadius = ROBOT_WIDTH / 3.0;
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
    if (this.superstructureStatus) {
      this.shotDistance.innerHTML = this.superstructureStatus.aimer() ?
          (this.superstructureStatus.aimer().shotDistance() /
           0.0254).toFixed(2) +
              'in, ' +
              this.superstructureStatus.aimer().shotDistance().toFixed(2) +
              'm' :
          'NA';

      this.fire.innerHTML = this.superstructureStatus.fire() ? 'true' : 'false';

      this.mpcHorizon.innerHTML =
          this.superstructureStatus.mpcHorizon().toFixed(2);

      this.setValue(this.mpcSolveTime, this.superstructureStatus.solveTime());

      this.shotCount.innerHTML =
          this.superstructureStatus.shotCount().toFixed(0);

      this.superstructureState.innerHTML =
          SuperstructureState[this.superstructureStatus.state()];

      this.intakeState.innerHTML =
          IntakeState[this.superstructureStatus.intakeState()];

      this.reseatingInCatapult.innerHTML =
          this.superstructureStatus.reseatingInCatapult() ? 'true' : 'false';

      this.flippersOpen.innerHTML =
          this.superstructureStatus.flippersOpen() ? 'true' : 'false';

      if (!this.superstructureStatus.catapult() ||
          !this.superstructureStatus.catapult().zeroed()) {
        this.setZeroing(this.catapult);
      } else if (this.superstructureStatus.catapult().estopped()) {
        this.setEstopped(this.catapult);
      } else {
        this.setTargetValue(
            this.catapult,
            this.superstructureStatus.catapult().unprofiledGoalPosition(),
            this.superstructureStatus.catapult().estimatorState().position(),
            1e-3);
      }

      if (!this.superstructureStatus.climber() ||
          !this.superstructureStatus.climber().zeroed()) {
        this.setZeroing(this.climber);
      } else if (this.superstructureStatus.climber().estopped()) {
        this.setEstopped(this.climber);
      } else {
        this.setTargetValue(
            this.climber,
            this.superstructureStatus.climber().unprofiledGoalPosition(),
            this.superstructureStatus.climber().estimatorState().position(),
            1e-3);
      }



      if (!this.superstructureStatus.turret() ||
          !this.superstructureStatus.turret().zeroed()) {
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

      if (!this.superstructureStatus.intakeBack() ||
          !this.superstructureStatus.intakeBack().zeroed()) {
        this.setZeroing(this.backIntake);
      } else if (this.superstructureStatus.intakeBack().estopped()) {
        this.setEstopped(this.backIntake);
      } else {
        this.setValue(
            this.backIntake,
            this.superstructureStatus.intakeBack().estimatorState().position());
      }

      if (!this.superstructureStatus.intakeFront() ||
          !this.superstructureStatus.intakeFront().zeroed()) {
        this.setZeroing(this.frontIntake);
      } else if (this.superstructureStatus.intakeFront().estopped()) {
        this.setEstopped(this.frontIntake);
      } else {
        this.setValue(
            this.frontIntake,
            this.superstructureStatus.intakeFront()
                .estimatorState()
                .position());
      }
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
          this.localizerOutput.theta(),
          this.superstructureStatus ?
              this.superstructureStatus.turret().position() :
              null);
    }

    for (const [time, value] of this.localizerImageMatches) {
      const age = now - time;
      const kRemovalAge = 1.0;
      if (age > kRemovalAge) {
        this.localizerImageMatches.delete(time);
        continue;
      }
      const kMaxImageAlpha = 0.5;
      const ageAlpha = kMaxImageAlpha * (kRemovalAge - age) / kRemovalAge
      for (let i = 0; i < value.targetsLength(); i++) {
        const imageDebug = value.targets(i);
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
