import {FIELD_WIDTH, FT_TO_M} from './constants';
import {drawField, drawTarget} from './field';
import {drawRobot, Frame} from './robot';

function main(): void {
  const vis = new Visualiser();
}

class Visualiser {
  private x = 3;
  private y = 0;
  private theta = 0;

  private drawLocked = false;
  private targetLocked = false;
  private targetX = 0;
  private targetY = 0;
  private targetTheta = 0;
  private cameraFrames : Frame[] = [];

  private wrist: number = -1;
  private elevator: number = -1;
  private intake: number = -1;
  private stilts: number = -1;
  private has_piece: number = 0;

  private wrist_div: HTMLDivElement;
  private elevator_div: HTMLDivElement;
  private intake_div: HTMLDivElement;
  private stilts_div: HTMLDivElement;
  private has_piece_div: HTMLDivElement;

  constructor() {
    const canvas = <HTMLCanvasElement>document.getElementById('field');
    this.wrist_div = <HTMLDivElement>document.getElementById('wrist');
    this.elevator_div = <HTMLDivElement>document.getElementById('elevator');
    this.intake_div = <HTMLDivElement>document.getElementById('intake');
    this.stilts_div = <HTMLDivElement>document.getElementById('stilts');
    this.has_piece_div = <HTMLDivElement>document.getElementById('has_piece');

    const ctx = canvas.getContext('2d');

    const server = location.host;
    this.initWebSocket(server);
    if (!!ctx) {
      window.requestAnimationFrame(() => this.draw(ctx));
    }
  }

  initWebSocket(server: string): void {
    const socket = new WebSocket(`ws://${server}/ws`);

    socket.addEventListener('message', (event) => {
      const j = JSON.parse(event.data);
      this.x = j.robotPose.x;
      this.y = j.robotPose.y;
      this.theta = j.robotPose.theta;

      if (j.lineFollowDebug) {
        this.targetLocked =
            j.lineFollowDebug.frozen && j.lineFollowDebug.haveTarget;
        this.targetX = j.lineFollowDebug.goalTarget.x;
        this.targetY = j.lineFollowDebug.goalTarget.y;
        this.targetTheta = j.lineFollowDebug.goalTarget.theta;
      }
      this.cameraFrames = j.cameraDebug;

      this.wrist = j.sensors.wrist;
      this.elevator = j.sensors.elevator;
      this.intake = j.sensors.intake;
      this.stilts = j.sensors.stilts;
      this.has_piece = j.sensors.hasPiece;
    });
    socket.addEventListener('close', (event) => {
      setTimeout(() => {
        this.initWebSocket(server);
      }, 1000);
    });
  }

  reset(ctx: CanvasRenderingContext2D): void {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const size = Math.min(window.innerHeight, window.innerWidth) * 0.98;
    ctx.canvas.height = size;
    ctx.canvas.width = size;
    ctx.clearRect(0, 0, size, size);

    ctx.translate(size / 2, size);
    ctx.rotate(-Math.PI / 2);
    ctx.scale(1, -1);
    const M_TO_PX = size / FIELD_WIDTH
    ctx.scale(M_TO_PX, M_TO_PX);
    ctx.lineWidth = 1 / M_TO_PX;

    this.wrist_div.textContent = "";
    this.elevator_div.textContent = "";
    this.intake_div.textContent = "";
    this.stilts_div.textContent = "";
    this.has_piece_div.textContent = "";
  }

  draw(ctx: CanvasRenderingContext2D): void {
    this.reset(ctx);

    drawField(ctx);
    drawRobot(ctx, this.x, this.y, this.theta, this.cameraFrames);
    ctx.save();
    ctx.lineWidth = 2.0 * ctx.lineWidth;
    if (this.targetLocked) {
      ctx.strokeStyle = 'blue';
    } else {
      ctx.strokeStyle = 'red';
    }
    drawTarget(ctx, this.targetX, this.targetY, this.targetTheta);
    ctx.restore();

    // Now update the superstructure positions.
    this.wrist_div.textContent = this.wrist.toFixed(3);
    this.elevator_div.textContent = this.elevator.toFixed(3);
    this.intake_div.textContent = this.intake.toFixed(3);
    this.stilts_div.textContent = this.stilts.toFixed(3);
    if (this.has_piece) {
      this.has_piece_div.textContent = "t";
    } else {
      this.has_piece_div.textContent = "f";
    }

    window.requestAnimationFrame(() => this.draw(ctx));
  }
}

main();
