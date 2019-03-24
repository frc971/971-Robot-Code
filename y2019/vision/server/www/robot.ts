import {IN_TO_M, FT_TO_M} from './constants';

const ROBOT_WIDTH = 25 * IN_TO_M;
const ROBOT_LENGTH = 31 * IN_TO_M;

export function drawRobot(ctx : CanvasRenderingContext2D, x : number, y : number, theta : number) : void {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(theta);

  ctx.fillStyle = 'blue';
  ctx.fillRect(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, ROBOT_LENGTH, ROBOT_WIDTH);

  ctx.moveTo(ROBOT_LENGTH / 2, -ROBOT_WIDTH/2);
  ctx.lineTo(ROBOT_LENGTH / 2 + 0.1, 0);
  ctx.lineTo(ROBOT_LENGTH / 2, ROBOT_WIDTH/2);
  ctx.closePath();
  ctx.stroke();

  ctx.restore();
}
