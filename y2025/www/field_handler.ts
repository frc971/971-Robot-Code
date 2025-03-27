import {ByteBuffer} from 'flatbuffers';
import {ClientStatistics} from '../../aos/network/message_bridge_client_generated';
import {
  ServerStatistics,
  State as ConnectionState,
} from '../../aos/network/message_bridge_server_generated';
import {Connection} from '../../aos/network/www/proxy';
import {ZeroingError} from '../../frc971/control_loops/control_loops_generated';
import {Position as DrivetrainPosition} from '../../frc971/control_loops/drivetrain/drivetrain_position_generated';
import {CANPosition as DrivetrainCANPosition} from '../../frc971/control_loops/drivetrain/drivetrain_can_position_generated';
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated';
import {Status as SwerveDrivetrainStatus} from '../../frc971/control_loops/swerve/swerve_drivetrain_status_generated';
import {Position as SuperstructurePosition} from '../control_loops/superstructure/superstructure_position_generated';
import {
  EndEffectorStatus,
  ClimberStatus,
  WristStatus,
  Status as SuperstructureStatus,
} from '../control_loops/superstructure/superstructure_status_generated';
import {TargetMap} from '../../frc971/vision/target_map_generated';
import {LocalizerState} from '../../frc971/control_loops/swerve/swerve_localizer_state_generated';
import {AutonomousInit} from '../../frc971/control_loops/swerve/autonomous_init_generated';
import {Status} from '../localizer/status_generated';
import {
  ClimberGoal,
  ElevatorGoal,
  PivotGoal,
  RobotSide,
  AutoAlignDirection,
  EndEffectorGoal,
  WristGoal,
  Goal as SuperstructureGoal,
} from '../control_loops/superstructure/superstructure_goal_generated';

import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 32 * IN_TO_M;
const ROBOT_LENGTH = 32 * IN_TO_M;

const CAMERA_COLORS = ['#ff00ff', '#ffff00'];
const CAMERAS = ['/orin1/camera0', '/orin1/camera1'];

const use_one_orin = true;

if (!use_one_orin) {
  CAMERAS.push('/imu/camera0', '/imu/camera1');
  CAMERA_COLORS.push('#00ffff', '#ffa500');
}

export class FieldHandler {
  private canvas = document.createElement('canvas');

  private autonomousInit: AutonomousInit | null = null;
  private localizerState: LocalizerState | null = null;
  private localizerStatus: Status | null = null;
  private superstructureGoal: SuperstructureGoal | null = null;
  private drivetrainStatus: DrivetrainStatus | null = null;
  private swerveDrivetrainStatus: SwerveDrivetrainStatus | null = null;
  private drivetrainPosition: DrivetrainPosition | null = null;
  private drivetrainCANPosition: DrivetrainCANPosition | null = null;
  private superstructureStatus: SuperstructureStatus | null = null;
  private superstructurePosition: SuperstructurePosition | null = null;

  // Image information indexed by timestamp (seconds since the epoch), so that
  // we can stop displaying images after a certain amount of time.
  private x: HTMLElement = document.getElementById('x') as HTMLElement;
  private y: HTMLElement = document.getElementById('y') as HTMLElement;
  private theta: HTMLElement = document.getElementById('theta') as HTMLElement;
  private naiveEstimatorTheta: HTMLElement =
    document.getElementById('naive-theta');

  private imagesAcceptedCounter: HTMLElement = document.getElementById(
    'images_accepted',
  ) as HTMLElement;
  // HTML elements for rejection reasons for individual cameras. Indices
  // corresponding to RejectionReason enum values will be for those reasons. The
  // final row will account for images rejected by the aprilrobotics detector
  // instead of the localizer.
  private rejectionReasonCells: HTMLElement[][] = [];
  private messageBridgeDiv: HTMLElement = document.getElementById(
    'message_bridge_status',
  ) as HTMLElement;
  private clientStatuses = new Map<string, HTMLElement>();
  private serverStatuses = new Map<string, HTMLElement>();

  private fieldImage: HTMLImageElement = new Image();

  private zeroingFaults: HTMLElement = document.getElementById(
    'zeroing_faults',
  ) as HTMLElement;

  private elevatorGoal: HTMLElement = document.getElementById(
    'elevator_goal',
  ) as HTMLElement;
  private pivotGoal: HTMLElement = document.getElementById(
    'pivot_goal',
  ) as HTMLElement;
  private wristGoal: HTMLElement = document.getElementById(
    'wrist_goal',
  ) as HTMLElement;
  private endEffectorGoal: HTMLElement = document.getElementById(
    'end_effector_goal',
  ) as HTMLElement;
  private climberGoal: HTMLElement = document.getElementById(
    'climber_goal',
  ) as HTMLElement;
  private robotSide: HTMLElement = document.getElementById(
    'robot_side',
  ) as HTMLElement;
  private autoAlignDirection: HTMLElement = document.getElementById(
    'auto_align_direction',
  ) as HTMLElement;
  private thetaLock: HTMLElement = document.getElementById(
    'theta_lock',
  ) as HTMLElement;

  private elevatorPos: HTMLElement = document.getElementById(
    'elevator_pos',
  ) as HTMLElement;
  private elevatorPot: HTMLElement = document.getElementById(
    'elevator_pot',
  ) as HTMLElement;
  private elevatorAbs: HTMLElement = document.getElementById(
    'elevator_abs',
  ) as HTMLElement;

  private pivotPos: HTMLElement = document.getElementById(
    'pivot_pos',
  ) as HTMLElement;
  private pivotPot: HTMLElement = document.getElementById(
    'pivot_pot',
  ) as HTMLElement;
  private pivotAbs: HTMLElement = document.getElementById(
    'pivot_abs',
  ) as HTMLElement;

  private wristPos: HTMLElement = document.getElementById(
    'wrist_pos',
  ) as HTMLElement;
  private wristAbs: HTMLElement = document.getElementById(
    'wrist_abs',
  ) as HTMLElement;

  constructor(private readonly connection: Connection) {
    (document.getElementById('field') as HTMLElement).appendChild(this.canvas);

    this.fieldImage.src = '2025.png';

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
          valueCell,
        );
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
        // Make unreliable to reduce network spam.
        this.connection.addHandler(
          CAMERAS[camera],
          'frc971.vision.TargetMap',
          (data) => {
            this.handleCameraTargetMap(camera, data);
          },
        );
      }

      let imu_or_orin1 = use_one_orin ? 'orin1' : 'imu';

      // is this the right node?
      this.connection.addHandler(
        '/' + imu_or_orin1 + '/drivetrain',
        'frc971.control_loops.swerve.Status',
        (data) => {
          this.handleSwerveDrivetrainStatus(data);
        },
      );
      this.connection.addHandler(
        '/orin1/localizer',
        'frc971.control_loops.swerve.LocalizerState',
        (data) => {
          this.handleLocalizerState(data);
        },
      );
      this.connection.addHandler(
        '/' + imu_or_orin1 + '/localizer',
        'frc971.control_loops.swerve.LocalizerState',
        (data) => {
          this.handleLocalizerState(data);
        },
      );
      this.connection.addHandler(
        '/drivetrain',
        'frc971.control_loops.swerve.AutonomousInit',
        (data) => {
          this.handleAutonomousInit(data);
        },
      );
      this.connection.addHandler(
        '/localizer',
        'y2025.localizer.Status',
        (data) => {
          this.handleLocalizerStatus(data);
        },
      );
      this.connection.addHandler(
        '/superstructure',
        'y2025.control_loops.superstructure.Status',
        (data) => {
          this.handleSuperstructureStatus(data);
        },
      );
      this.connection.addHandler(
        '/superstructure',
        'y2025.control_loops.superstructure.Goal',
        (data) => {
          this.handleSuperstructureGoal(data);
        },
      );
      this.connection.addHandler(
        '/aos',
        'aos.message_bridge.ServerStatistics',
        (data) => {
          this.handleServerStatistics(data);
        },
      );
      this.connection.addHandler(
        '/aos',
        'aos.message_bridge.ClientStatistics',
        (data) => {
          this.handleClientStatistics(data);
        },
      );
    });
  }
  private handleCameraTargetMap(pi: string, data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    const targetMap = TargetMap.getRootAsTargetMap(fbBuffer);
    this.rejectionReasonCells[this.rejectionReasonCells.length - 1][
      pi
    ].innerHTML = targetMap.rejections().toString();
  }

  private handleLocalizerState(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.localizerState = LocalizerState.getRootAsLocalizerState(fbBuffer);
  }

  private handleSwerveDrivetrainStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.swerveDrivetrainStatus =
      SwerveDrivetrainStatus.getRootAsStatus(fbBuffer);
  }

  private handleSuperstructureStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.superstructureStatus = SuperstructureStatus.getRootAsStatus(fbBuffer);
  }

  private handleSuperstructureGoal(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.superstructureGoal = SuperstructureGoal.getRootAsGoal(fbBuffer);
  }

  private handleAutonomousInit(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.autonomousInit = AutonomousInit.getRootAsAutonomousInit(fbBuffer);
  }

  private handleLocalizerStatus(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.localizerStatus = Status.getRootAsStatus(fbBuffer);
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

  private setCurrentNodeState(
    element: HTMLElement,
    state: ConnectionState,
  ): void {
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
        this.serverStatuses.get(nodeName),
        connection.state(),
      );
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
        this.clientStatuses.get(nodeName),
        connection.state(),
      );
    }
  }

  drawField(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.scale(1.0, -1.0);
    ctx.drawImage(
      this.fieldImage,
      0,
      0,
      this.fieldImage.width,
      this.fieldImage.height,
      -FIELD_EDGE_X,
      -FIELD_SIDE_Y,
      FIELD_LENGTH,
      FIELD_WIDTH,
    );
    ctx.restore();
  }

  drawCamera(
    x: number,
    y: number,
    theta: number,
    color: string = 'blue',
  ): void {
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
    x: number,
    y: number,
    theta: number,
    color: string = 'blue',
    dashed: boolean = false,
  ): void {
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
    div: HTMLElement,
    target: number,
    val: number,
    tolerance: number,
  ): void {
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
    div.innerHTML = triggered ? 'TRUE' : 'FALSE';
    div.className = '';
    if (triggered) {
      div.classList.add('lightgreen');
    } else {
      div.classList.add('lightcoral');
    }
  }

  draw(): void {
    this.reset();
    this.drawField();
    if (this.superstructureGoal) {
      this.elevatorGoal.innerHTML =
        ElevatorGoal[this.superstructureGoal.elevatorGoal()];
      this.pivotGoal.innerHTML = PivotGoal[this.superstructureGoal.pivotGoal()];
      this.wristGoal.innerHTML = WristGoal[this.superstructureGoal.wristGoal()];
      this.endEffectorGoal.innerHTML =
        EndEffectorGoal[this.superstructureGoal.endEffectorGoal()];
      this.climberGoal.innerHTML =
        ClimberGoal[this.superstructureGoal.climberGoal()];
      this.robotSide.innerHTML = RobotSide[this.superstructureGoal.robotSide()];
      this.autoAlignDirection.innerHTML =
        AutoAlignDirection[this.superstructureGoal.autoAlignDirection()];
      this.setBoolean(this.thetaLock, this.superstructureGoal.thetaLock());
    }
    if (this.superstructureStatus) {
      if (
        !this.superstructureStatus.elevator() ||
        !this.superstructureStatus.elevator().zeroed()
      ) {
        this.setZeroing(this.elevatorPos);
      } else if (this.superstructureStatus.elevator().estopped()) {
        this.setEstopped(this.elevatorPos);
      } else {
        this.setTargetValue(
          this.elevatorPos,
          this.superstructureStatus.elevator().unprofiledGoalPosition(),
          this.superstructureStatus.elevator().estimatorState().position(),
          1e-3,
        );
      }

      this.elevatorPot.innerHTML = this.superstructureStatus
        .elevator()
        .estimatorState()
        .potPosition()
        .toString();
      this.elevatorAbs.innerHTML = this.superstructureStatus
        .elevator()
        .estimatorState()
        .absolutePosition()
        .toString();

      if (
        !this.superstructureStatus.pivot() ||
        !this.superstructureStatus.pivot().zeroed()
      ) {
        this.setZeroing(this.pivotPos);
      } else if (this.superstructureStatus.pivot().estopped()) {
        this.setEstopped(this.pivotPos);
      } else {
        this.setTargetValue(
          this.pivotPos,
          this.superstructureStatus.pivot().unprofiledGoalPosition(),
          this.superstructureStatus.pivot().estimatorState().position(),
          1e-3,
        );
      }

      this.pivotPot.innerHTML = this.superstructureStatus
        .pivot()
        .estimatorState()
        .potPosition()
        .toString();
      this.pivotAbs.innerHTML = this.superstructureStatus
        .pivot()
        .estimatorState()
        .absolutePosition()
        .toString();

      if (
        !this.superstructureStatus.wrist() ||
        !this.superstructureStatus.wrist().zeroed()
      ) {
        this.setZeroing(this.wristPos);
      } else if (this.superstructureStatus.wrist().estopped()) {
        this.setEstopped(this.wristPos);
      } else {
        this.setTargetValue(
          this.wristPos,
          this.superstructureStatus.wrist().unprofiledGoalPosition(),
          this.superstructureStatus.wrist().estimatorState().position(),
          1e-3,
        );
      }

      this.wristAbs.innerHTML = this.superstructureStatus
        .wrist()
        .estimatorState()
        .absolutePosition()
        .toString();
    }
    if (this.drivetrainStatus && this.drivetrainStatus.trajectoryLogging()) {
      this.drawRobot(
        this.drivetrainStatus.trajectoryLogging().x(),
        this.drivetrainStatus.trajectoryLogging().y(),
        this.drivetrainStatus.trajectoryLogging().theta(),
        '#000000FF',
        false,
      );
    }

    if (this.localizerState) {
      this.drawRobot(
        this.localizerState.x(),
        this.localizerState.y(),
        this.localizerState.theta(),
      );
      this.setValue(this.x, this.localizerState.x());
      this.setValue(this.y, this.localizerState.y());
      this.setValue(this.theta, this.localizerState.theta());
      this.setValue(
        this.naiveEstimatorTheta,
        this.swerveDrivetrainStatus.naiveEstimator().yaw(),
      );
    } else {
      this.x.textContent = 'NA';
      this.y.textContent = 'NA';
      this.theta.textContent = 'NA';
      this.naiveEstimatorTheta.textContent = 'NA';
    }

    if (this.localizerStatus) {
      for (let i = 0; i < this.localizerStatus.debugEstimates.length; i++) {
        const pose_estimate = this.localizerStatus.debugEstimates(i);
        if (pose_estimate.robotX() == 0) {
          continue;
        }
        this.drawRobot(
          pose_estimate.robotX(),
          pose_estimate.robotY(),
          pose_estimate.robotTheta(),
          CAMERA_COLORS[Number(pose_estimate.camera())],
        );
      }
    }

    if (this.autonomousInit) {
      this.drawRobot(
        this.autonomousInit.x(),
        this.autonomousInit.y(),
        this.autonomousInit.theta(),
        '#c8ff69',
      );
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
