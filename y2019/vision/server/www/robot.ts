import {CAMERA_POSES} from './camera_constants';
import {FT_TO_M, IN_TO_M} from './constants';

const ROBOT_WIDTH = 25 * IN_TO_M;
const ROBOT_LENGTH = 31 * IN_TO_M;
const CAMERA_SCALE = 0.3;

function drawCamera(
    ctx: CanvasRenderingContext2D,
    pose: {x: number, y: number, theta: number}): void {
  ctx.beginPath();
  ctx.moveTo(pose.x, pose.y);
  ctx.lineTo(
      pose.x + CAMERA_SCALE * Math.cos(pose.theta + Math.PI / 4.0),
      pose.y + CAMERA_SCALE * Math.sin(pose.theta + Math.PI / 4.0));
  ctx.lineTo(
      pose.x + CAMERA_SCALE * Math.cos(pose.theta - Math.PI / 4.0),
      pose.y + CAMERA_SCALE * Math.sin(pose.theta - Math.PI / 4.0));
  ctx.closePath();
  ctx.stroke();
}

export function drawRobot(
    ctx: CanvasRenderingContext2D, x: number, y: number, theta: number,
    camera_colors: string[]): void {
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
    ctx.strokeStyle = camera_colors[ii];
    drawCamera(ctx, CAMERA_POSES[ii]);
  }

  ctx.restore();
}
