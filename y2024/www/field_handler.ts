import {ByteBuffer} from 'flatbuffers'
import {ClientStatistics} from '../../aos/network/message_bridge_client_generated'
import {ServerStatistics, State as ConnectionState} from '../../aos/network/message_bridge_server_generated'
import {Connection} from '../../aos/network/www/proxy'
import {ZeroingError} from '../../frc971/control_loops/control_loops_generated'
import {Position as DrivetrainPosition} from '../../frc971/control_loops/drivetrain/drivetrain_position_generated'
import {CANPosition as DrivetrainCANPosition} from '../../frc971/control_loops/drivetrain/drivetrain_can_position_generated'
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated'
import {SuperstructureState, IntakeRollerStatus, CatapultState, TransferRollerStatus, ExtendRollerStatus, ExtendStatus, NoteStatus, Status as SuperstructureStatus} from '../control_loops/superstructure/superstructure_status_generated'
import {LocalizerOutput} from '../../frc971/control_loops/drivetrain/localization/localizer_output_generated'
import {TargetMap} from '../../frc971/vision/target_map_generated'
import {RejectionReason} from '../localizer/status_generated'
import {TargetEstimateDebug, Visualization} from '../localizer/visualization_generated'


import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 29 * IN_TO_M;
const ROBOT_LENGTH = 32 * IN_TO_M;

const CAMERA_COLORS = ['#ff00ff', '#ffff00', '#00ffff', '#ffa500'];
const CAMERAS = ['/orin1/camera0', '/orin1/camera1', '/imu/camera0', '/imu/camera1'];

export class FieldHandler {
  private canvas = document.createElement('canvas');
  private localizerOutput: LocalizerOutput|null = null;
  private drivetrainStatus: DrivetrainStatus|null = null;
  private drivetrainPosition: DrivetrainPosition|null = null;
  private drivetrainCANPosition: DrivetrainCANPosition|null = null;
  private superstructureStatus: SuperstructureStatus|null = null;

  // Image information indexed by timestamp (seconds since the epoch), so that
  // we can stop displaying images after a certain amount of time.
  private localizerImageMatches = new Map<number, Visualization>();
  private x: HTMLElement = (document.getElementById('x') as HTMLElement);
  private y: HTMLElement = (document.getElementById('y') as HTMLElement);
  private theta: HTMLElement =
      (document.getElementById('theta') as HTMLElement);

  private imagesAcceptedCounter: HTMLElement =
      (document.getElementById('images_accepted') as HTMLElement);
  // HTML elements for rejection reasons for individual cameras. Indices
  // corresponding to RejectionReason enum values will be for those reasons. The
  // final row will account for images rejected by the aprilrobotics detector
  // instead of the localizer.
  private rejectionReasonCells: HTMLElement[][] = [];
  private messageBridgeDiv: HTMLElement =
      (document.getElementById('message_bridge_status') as HTMLElement);
  private clientStatuses = new Map<string, HTMLElement>();
  private serverStatuses = new Map<string, HTMLElement>();

  private fieldImage: HTMLImageElement = new Image();

  private zeroingFaults: HTMLElement =
      (document.getElementById('zeroing_faults') as HTMLElement);

  private superstructureState: HTMLElement =
    (document.getElementById('superstructure_state') as HTMLElement);

  private intakeRollerState: HTMLElement =
    (document.getElementById('intake_roller_state') as HTMLElement);
  private transferRollerState: HTMLElement =
    (document.getElementById('transfer_roller_state') as HTMLElement);
  private extendState: HTMLElement =
    (document.getElementById('extend_state') as HTMLElement);
  private extendRollerState: HTMLElement =
    (document.getElementById('extend_roller_state') as HTMLElement);
  private catapultState: HTMLElement =
    (document.getElementById('catapult_state') as HTMLElement);
  private uncompletedNoteGoal: HTMLElement =
  (document.getElementById('uncompleted_note_goal') as HTMLElement);

  private extend_beambreak: HTMLElement =
  (document.getElementById('extend_beambreak') as HTMLElement);
  private catapult_beambreak: HTMLElement =
  (document.getElementById('catapult_beambreak') as HTMLElement);

  private extend_at_retracted: HTMLElement =
  (document.getElementById('extend_at_retracted') as HTMLElement);
  private extend_ready_for_transfer: HTMLElement =
  (document.getElementById('extend_ready_for_transfer') as HTMLElement);
  private extend_ready_for_catapult_transfer: HTMLElement =
  (document.getElementById('extend_ready_for_catapult_transfer') as HTMLElement);
  private turret_ready_for_load: HTMLElement =
  (document.getElementById('turret_ready_for_load') as HTMLElement);
  private altitude_ready_for_load: HTMLElement =
  (document.getElementById('altitude_ready_for_load') as HTMLElement);

  private turret_in_range: HTMLElement =
  (document.getElementById('turret_in_range') as HTMLElement);
  private altitude_in_range: HTMLElement =
  (document.getElementById('altitude_in_range') as HTMLElement);
  private altitude_above_min_angle: HTMLElement =
  (document.getElementById('altitude_above_min_angle') as HTMLElement);


  private intakePivot: HTMLElement =
    (document.getElementById('intake_pivot') as HTMLElement);
  private intakePivotAbs: HTMLElement =
    (document.getElementById('intake_pivot_abs') as HTMLElement);

  private climber: HTMLElement =
    (document.getElementById('climber') as HTMLElement);
  private climberAbs: HTMLElement =
    (document.getElementById('climber_abs') as HTMLElement);
  private climberPot: HTMLElement =
    (document.getElementById('climber_pot') as HTMLElement);

  private extend: HTMLElement =
    (document.getElementById('extend') as HTMLElement);
  private extendAbs: HTMLElement =
    (document.getElementById('extend_abs') as HTMLElement);
  private extendPot: HTMLElement =
    (document.getElementById('extend_pot') as HTMLElement);

  private turret: HTMLElement =
    (document.getElementById('turret') as HTMLElement);
  private turretAbs: HTMLElement =
    (document.getElementById('turret_abs') as HTMLElement);
  private turretPot: HTMLElement =
    (document.getElementById('turret_pot') as HTMLElement);

  private catapult: HTMLElement =
    (document.getElementById('catapult') as HTMLElement);
  private catapultAbs: HTMLElement =
    (document.getElementById('catapult_abs') as HTMLElement);
  private catapultPot: HTMLElement =
    (document.getElementById('catapult_pot') as HTMLElement);

  private altitude: HTMLElement =
    (document.getElementById('altitude') as HTMLElement);
  private altitudeAbs: HTMLElement =
    (document.getElementById('altitude_abs') as HTMLElement);
  private altitudePot: HTMLElement =
    (document.getElementById('altitude_pot') as HTMLElement);

  private turret_position: HTMLElement =
    (document.getElementById('turret_position') as HTMLElement);
  private turret_velocity: HTMLElement =
    (document.getElementById('turret_velocity') as HTMLElement);
  private target_distance: HTMLElement =
    (document.getElementById('target_distance') as HTMLElement);
  private shot_distance: HTMLElement =
    (document.getElementById('shot_distance') as HTMLElement);

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

    // Construct a table header.
    {
      const row = document.createElement('div');
      const nameCell = document.createElement('div');
      nameCell.innerHTML = 'Rejection Reason';
      row.appendChild(nameCell);
      for (const camera of CAMERAS) {
        const nodeCell = document.createElement('div');
        nodeCell.innerHTML = camera;
        row.appendChild(nodeCell);
      }
      document.getElementById('vision_readouts').appendChild(row);
    }

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
      this.rejectionReasonCells.push([]);
      for (const camera of CAMERAS) {
        const valueCell = document.createElement('div');
        valueCell.innerHTML = 'NA';
        this.rejectionReasonCells[this.rejectionReasonCells.length - 1].push(
            valueCell);
        row.appendChild(valueCell);
      }
      document.getElementById('vision_readouts').appendChild(row);
    }

    // Add rejection reason row for aprilrobotics rejections.
    {
      const row = document.createElement('div');
      const nameCell = document.createElement('div');
      nameCell.innerHTML = 'Rejected by aprilrobotics';
      row.appendChild(nameCell);
      this.rejectionReasonCells.push([]);
      for (const camera of CAMERAS) {
        const valueCell = document.createElement('div');
        valueCell.innerHTML = 'NA';
        this.rejectionReasonCells[this.rejectionReasonCells.length - 1].push(
            valueCell);
        row.appendChild(valueCell);
      }
      document.getElementById('vision_readouts').appendChild(row);
    }

    for (let ii = 0; ii < CAMERA_COLORS.length; ++ii) {
      const legendEntry = document.createElement('div');
      legendEntry.style.color = CAMERA_COLORS[ii];
      legendEntry.innerHTML = CAMERAS[ii];
      document.getElementById('legend').appendChild(legendEntry);
    }

    this.connection.addConfigHandler(() => {
      // Visualization message is reliable so that we can see *all* the vision
      // matches.
      for (const camera in CAMERAS) {
        this.connection.addHandler(
            CAMERAS[camera], 'y2024.localizer.Visualization',
            (data) => {
              this.handleLocalizerDebug(Number(camera), data);
            });
      }
      for (const camera in CAMERAS) {
        // Make unreliable to reduce network spam.
        this.connection.addHandler(
          CAMERAS[camera], 'frc971.vision.TargetMap', (data) => {
              this.handleCameraTargetMap(camera, data);
            });
      }

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
      this.connection.addHandler(
        '/superstructure', "y2024.control_loops.superstructure.Status",
        (data) => {
          this.handleSuperstructureStatus(data)
          });
      this.connection.addHandler(
        '/aos', 'aos.message_bridge.ServerStatistics',
        (data) => {this.handleServerStatistics(data)});
      this.connection.addHandler(
        '/aos', 'aos.message_bridge.ClientStatistics',
        (data) => {this.handleClientStatistics(data)});
      });
  }
  private handleLocalizerDebug(camera: number, data: Uint8Array): void {
    const now = Date.now() / 1000.0;

    const fbBuffer = new ByteBuffer(data);
    this.localizerImageMatches.set(
        now, Visualization.getRootAsVisualization(fbBuffer));

    const debug = this.localizerImageMatches.get(now);

    if (debug.statistics()) {
      if ((debug.statistics().rejectionReasonsLength() + 1) ==
          this.rejectionReasonCells.length) {
        for (let ii = 0; ii < debug.statistics().rejectionReasonsLength();
             ++ii) {
          this.rejectionReasonCells[ii][camera].innerHTML =
              debug.statistics().rejectionReasons(ii).count().toString();
        }
      } else {
        console.error('Unexpected number of rejection reasons in counter.');
      }
    }
  }

  private handleCameraTargetMap(pi: string, data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    const targetMap = TargetMap.getRootAsTargetMap(fbBuffer);
    this.rejectionReasonCells[this.rejectionReasonCells.length - 1][pi]
        .innerHTML = targetMap.rejections().toString();
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

  private handleSuperstructureStatus(data: Uint8Array): void {
	  const fbBuffer = new ByteBuffer(data);
	  this.superstructureStatus = SuperstructureStatus.getRootAsStatus(fbBuffer);
  }

  private populateNodeConnections(nodeName: string): void {
    const row = document.createElement('div');
    this.messageBridgeDiv.appendChild(row);
    const nodeDiv = document.createElement('div');
    nodeDiv.innerHTML = nodeName;
    row.appendChild(nodeDiv);
    const clientDiv = document.createElement('div');
    clientDiv.innerHTML = 'N/A';
    row.appendChild(clientDiv);
    const serverDiv = document.createElement('div');
    serverDiv.innerHTML = 'N/A';
    row.appendChild(serverDiv);
    this.serverStatuses.set(nodeName, serverDiv);
    this.clientStatuses.set(nodeName, clientDiv);
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
    ctx.drawImage(
        this.fieldImage, 0, 0, this.fieldImage.width, this.fieldImage.height,
        -FIELD_EDGE_X, -FIELD_SIDE_Y, FIELD_LENGTH, FIELD_WIDTH);
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

  setBoolean(div: HTMLElement, triggered: boolean): void {
    div.innerHTML = ((triggered) ? "TRUE" : "FALSE")
    if (triggered) {
      div.classList.remove('false');
      div.classList.add('true');
    } else {
      div.classList.remove('true');
      div.classList.add('false');
    }
  }

  draw(): void {
    this.reset();
    this.drawField();

    // Draw the matches with debugging information from the localizer.
    const now = Date.now() / 1000.0;

    if (this.superstructureStatus) {
      this.superstructureState.innerHTML =
        SuperstructureState[this.superstructureStatus.state()];

      this.intakeRollerState.innerHTML =
        IntakeRollerStatus[this.superstructureStatus.intakeRoller()];
      this.transferRollerState.innerHTML =
        TransferRollerStatus[this.superstructureStatus.transferRoller()];
      this.extendState.innerHTML =
        ExtendStatus[this.superstructureStatus.extendStatus()];
      this.extendRollerState.innerHTML =
        ExtendRollerStatus[this.superstructureStatus.extendRoller()];
      this.catapultState.innerHTML =
        CatapultState[this.superstructureStatus.shooter().catapultState()];
      this.uncompletedNoteGoal.innerHTML =
        NoteStatus[this.superstructureStatus.uncompletedNoteGoal()];

      this.setBoolean(this.extend_beambreak, this.superstructureStatus.extendBeambreak());

      this.setBoolean(this.catapult_beambreak, this.superstructureStatus.catapultBeambreak());

      this.setBoolean(this.extend_ready_for_transfer, this.superstructureStatus.extendReadyForTransfer());

      this.setBoolean(this.extend_at_retracted, this.superstructureStatus.extendAtRetracted());

      this.setBoolean(this.turret_ready_for_load, this.superstructureStatus.turretReadyForLoad());

      this.setBoolean(this.altitude_ready_for_load, this.superstructureStatus.altitudeReadyForLoad());

      this.setBoolean(this.extend_ready_for_catapult_transfer, this.superstructureStatus.extendReadyForCatapultTransfer());

      this.setBoolean(this.turret_in_range, this.superstructureStatus.shooter().turretInRange())

      this.setBoolean(this.altitude_in_range, this.superstructureStatus.shooter().altitudeInRange())

      this.setBoolean(this.altitude_above_min_angle, this.superstructureStatus.shooter().altitudeAboveMinAngle())

      if (this.superstructureStatus.shooter() &&
          this.superstructureStatus.shooter().aimer()) {
        this.turret_position.innerHTML = this.superstructureStatus.shooter()
                                             .aimer()
                                             .turretPosition()
                                             .toString();
        this.turret_velocity.innerHTML = this.superstructureStatus.shooter()
                                             .aimer()
                                             .turretVelocity()
                                             .toString();
        this.target_distance.innerHTML = this.superstructureStatus.shooter()
                                             .aimer()
                                             .targetDistance()
                                             .toString();
        this.shot_distance.innerHTML = this.superstructureStatus.shooter()
                                           .aimer()
                                           .shotDistance()
                                           .toString();
      }

      if (!this.superstructureStatus.intakePivot() ||
          !this.superstructureStatus.intakePivot().zeroed()) {
        this.setZeroing(this.intakePivot);
      } else if (this.superstructureStatus.intakePivot().estopped()) {
        this.setEstopped(this.intakePivot);
      } else {
        this.setTargetValue(
            this.intakePivot,
            this.superstructureStatus.intakePivot().unprofiledGoalPosition(),
            this.superstructureStatus.intakePivot().estimatorState().position(),
            1e-3);
      }

      this.intakePivotAbs.innerHTML = this.superstructureStatus.intakePivot().estimatorState().absolutePosition().toString();

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

      this.climberAbs.innerHTML = this.superstructureStatus.climber().estimatorState().absolutePosition().toString();
      this.climberPot.innerHTML = this.superstructureStatus.climber().estimatorState().potPosition().toString();

      if (!this.superstructureStatus.extend() ||
          !this.superstructureStatus.extend().zeroed()) {
        this.setZeroing(this.extend);
      } else if (this.superstructureStatus.extend().estopped()) {
        this.setEstopped(this.extend);
      } else {
        this.setTargetValue(
            this.extend,
            this.superstructureStatus.extend().unprofiledGoalPosition(),
            this.superstructureStatus.extend().estimatorState().position(),
            1e-3);
      }

      this.extendAbs.innerHTML = this.superstructureStatus.extend().estimatorState().absolutePosition().toString();
      this.extendPot.innerHTML = this.superstructureStatus.extend().estimatorState().potPosition().toString();

      if (!this.superstructureStatus.shooter().turret() ||
          !this.superstructureStatus.shooter().turret().zeroed()) {
        this.setZeroing(this.turret);
      } else if (this.superstructureStatus.shooter().turret().estopped()) {
        this.setEstopped(this.turret);
      } else {
        this.setTargetValue(
            this.turret,
            this.superstructureStatus.shooter().turret().unprofiledGoalPosition(),
            this.superstructureStatus.shooter().turret().estimatorState().position(),
            1e-3);
      }

      this.turretAbs.innerHTML = this.superstructureStatus.shooter().turret().estimatorState().absolutePosition().toString();
      this.turretPot.innerHTML = this.superstructureStatus.shooter().turret().estimatorState().potPosition().toString();

      if (!this.superstructureStatus.shooter().catapult() ||
          !this.superstructureStatus.shooter().catapult().zeroed()) {
        this.setZeroing(this.catapult);
      } else if (this.superstructureStatus.shooter().catapult().estopped()) {
        this.setEstopped(this.catapult);
      } else {
        this.setTargetValue(
            this.catapult,
            this.superstructureStatus.shooter().catapult().unprofiledGoalPosition(),
            this.superstructureStatus.shooter().catapult().estimatorState().position(),
            1e-3);
      }

      this.catapultAbs.innerHTML = this.superstructureStatus.shooter().catapult().estimatorState().absolutePosition().toString();
      this.catapultPot.innerHTML = this.superstructureStatus.shooter().catapult().estimatorState().potPosition().toString();

      if (!this.superstructureStatus.shooter().altitude() ||
          !this.superstructureStatus.shooter().altitude().zeroed()) {
        this.setZeroing(this.altitude);
      } else if (this.superstructureStatus.shooter().altitude().estopped()) {
        this.setEstopped(this.altitude);
      } else {
        this.setTargetValue(
            this.altitude,
            this.superstructureStatus.shooter().altitude().unprofiledGoalPosition(),
            this.superstructureStatus.shooter().altitude().estimatorState().position(),
            1e-3);
      }

      this.altitudeAbs.innerHTML = this.superstructureStatus.shooter().altitude().estimatorState().absolutePosition().toString();
      this.altitudePot.innerHTML = this.superstructureStatus.shooter().altitude().estimatorState().potPosition().toString();

      let zeroingErrors: string = 'Intake Pivot Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.intakePivot()
                              .estimatorState()
                              .errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.intakePivot()
                                          .estimatorState()
                                          .errors(i)] +
            '<br/>';
      }
      zeroingErrors += '<br/>' +
          'Climber Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.climber().estimatorState().errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.climber().estimatorState().errors(i)] +
            '<br/>';
      }
      zeroingErrors += '<br/>' +
          'Extend Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.extend().estimatorState().errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.extend().estimatorState().errors(i)] +
            '<br/>';
      }
      zeroingErrors += '<br/>' +
          'Turret Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.shooter().turret().estimatorState().errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.shooter().turret().estimatorState().errors(i)] +
            '<br/>';
      }
      zeroingErrors += '<br/>' +
          'Catapult Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.shooter().catapult().estimatorState().errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.shooter().catapult().estimatorState().errors(i)] +
            '<br/>';
      }
      zeroingErrors += '<br/>' +
          'Altitude Errors:' +
          '<br/>';
      for (let i = 0; i < this.superstructureStatus.shooter().altitude().estimatorState().errorsLength();
           i++) {
        zeroingErrors += ZeroingError[this.superstructureStatus.shooter().altitude().estimatorState().errors(i)] +
            '<br/>';
      }
      this.zeroingFaults.innerHTML = zeroingErrors;
    }

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

      this.imagesAcceptedCounter.innerHTML =
          this.localizerOutput.imageAcceptedCount().toString();
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
        const cameraRgb = CAMERA_COLORS[imageDebug.camera()];
        const cameraRgba = cameraRgb + alpha;
        this.drawRobot(x, y, theta, cameraRgba, dashed);
        this.drawCamera(cameraX, cameraY, cameraTheta, cameraRgba);
      }
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
