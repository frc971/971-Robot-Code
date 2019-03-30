import {FT_TO_M, FIELD_WIDTH} from './constants';
import {drawField, drawTarget} from './field';
import {drawRobot} from './robot';

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
  private cameraColors = ['red', 'red', 'red', 'red', 'red'];

  constructor() {
    const canvas = <HTMLCanvasElement>document.getElementById('field');
    const ctx = canvas.getContext('2d');

    const server = location.host;
    const socket = new WebSocket(`ws://${server}/ws`);
    const reader = new FileReader();
    reader.addEventListener('loadend', (e) => {
      const text = e.srcElement.result;
      const j = JSON.parse(text);
      this.x = j.robotPose.x;
      this.y = j.robotPose.y;
      this.theta = j.robotPose.theta;

      if(j.lineFollowDebug) {
        this.targetLocked = j.lineFollowDebug.frozen && j.lineFollowDebug.haveTarget;
        this.targetX = j.lineFollowDebug.goalTarget.x;
        this.targetY = j.lineFollowDebug.goalTarget.y;
        this.targetTheta = j.lineFollowDebug.goalTarget.theta;
      }
      for (let ii of [0, 1, 2, 3, 4]) {
        if (j.cameraDebug[ii].timeSinceLastTarget > 0.25) {
          this.cameraColors[ii] = 'red';
        } else {
          this.cameraColors[ii] = 'green';
        }
      }
    });
    socket.addEventListener('message', (event) => {
      reader.readAsText(event.data);
    });
    window.requestAnimationFrame(() => this.draw(ctx));
  }

  reset(ctx : CanvasRenderingContext2D) : void {
    ctx.setTransform(1,0,0,1,0,0);
    const size = Math.min(window.innerHeight, window.innerWidth) * 0.98;
    ctx.canvas.height = size;
    ctx.canvas.width = size;
    ctx.clearRect(0,0,size,size);

    ctx.translate(size/2, size);
    ctx.rotate(-Math.PI / 2);
    ctx.scale(1, -1);
    const M_TO_PX = size / FIELD_WIDTH
    ctx.scale(M_TO_PX, M_TO_PX);
    ctx.lineWidth = 1 / M_TO_PX;
  }

  draw(ctx : CanvasRenderingContext2D) : void {
    this.reset(ctx);

    drawField(ctx);
    drawRobot(ctx, this.x, this.y, this.theta, this.cameraColors);
    ctx.save();
    ctx.lineWidth = 2.0 * ctx.lineWidth;
    if (this.targetLocked) {
      ctx.strokeStyle = 'blue';
    } else {
      ctx.strokeStyle = 'red';
    }
    drawTarget(ctx, this.targetX, this.targetY, this.targetTheta);
    ctx.restore();
    window.requestAnimationFrame(() => this.draw(ctx));
  }
}

main();
