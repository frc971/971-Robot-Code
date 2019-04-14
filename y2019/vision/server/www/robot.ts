import {CAMERA_POSES} from './camera_constants';
import {FT_TO_M, IN_TO_M} from './constants';
import {drawTarget} from './field';

const ROBOT_WIDTH = 25 * IN_TO_M;
const ROBOT_LENGTH = 31 * IN_TO_M;
const CAMERA_SCALE = 0.2;

interface Pose {
  x : number;
  y : number;
  theta: number;
}

export interface Frame {
  timeSinceLastTarget : number;
  currentFrameAge : number;
  targets : Pose[];
}

function drawCamera(
    ctx: CanvasRenderingContext2D, pose: Pose, frame: Frame): void {
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.theta);
  if (frame.timeSinceLastTarget > 0.25) {
    ctx.strokeStyle = 'red';
  } else {
    ctx.strokeStyle = 'green';
  }
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(CAMERA_SCALE, CAMERA_SCALE);
  ctx.lineTo(CAMERA_SCALE, -CAMERA_SCALE);
  ctx.closePath();
  ctx.stroke();
  ctx.restore();
}

export function drawRobot(
    ctx: CanvasRenderingContext2D, x: number, y: number, theta: number,
    cameraFrames: Frame[]): void {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(theta);

  ctx.fillStyle = 'blue';
  ctx.fillRect(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, ROBOT_LENGTH, ROBOT_WIDTH);

  ctx.beginPath();
  ctx.strokeStyle = 'black';
  ctx.moveTo(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  ctx.lineTo(ROBOT_LENGTH / 2 + 0.1, 0);
  ctx.lineTo(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  ctx.closePath();
  ctx.stroke();
  ctx.lineWidth = 3.0 * ctx.lineWidth;
  for (let ii of [0, 1, 2, 3, 4]) {
    if (ii < cameraFrames.length) {
      drawCamera(ctx, CAMERA_POSES[ii], cameraFrames[ii]);
    }
  }

  ctx.restore();

  ctx.save();
  ctx.lineWidth = 3.0 * ctx.lineWidth;
  ctx.strokeStyle = 'yellow';
  for (let frame of cameraFrames) {
    if (frame.targets) {
      for (let target of frame.targets) {
        drawTarget(ctx, target.x, target.y, target.theta);
      }
    }
  }
  ctx.restore();
}
