import {FT_TO_M, IN_TO_M} from './constants';

const CENTER_FIELD_X = 27 * FT_TO_M + 1.125 * IN_TO_M;

const FAR_CARGO_X = CENTER_FIELD_X - 20.875 * IN_TO_M;
const MID_CARGO_X = FAR_CARGO_X - 21.75 * IN_TO_M;
const NEAR_CARGO_X = MID_CARGO_X - 21.75 * IN_TO_M;
const SIDE_CARGO_Y = (24 + 3 + 0.875) * IN_TO_M;
const SIDE_CARGO_THETA = -Math.PI / 2;

const FACE_CARGO_X = CENTER_FIELD_X - (7 * 12 + 11.75 + 9) * IN_TO_M;
const FACE_CARGO_Y = (10.875) * IN_TO_M;
const FACE_CARGO_THETA = 0;

const ROCKET_X = CENTER_FIELD_X - 8 * FT_TO_M;
const ROCKET_Y = (26 * 12 + 10.5) / 2.0 * IN_TO_M;

const ROCKET_PORT_X = ROCKET_X;
const ROCKET_PORT_Y = ROCKET_Y - 0.7;
const ROCKET_PORT_THETA = Math.PI / 2;

const ROCKET_HATCH_X_OFFSET = 14.634 * IN_TO_M;
const ROCKET_HATCH_Y = ROCKET_PORT_Y + 9.326 * IN_TO_M;
const ROCKET_NEAR_X = ROCKET_X - ROCKET_HATCH_X_OFFSET;
const ROCKET_FAR_X = ROCKET_X + ROCKET_HATCH_X_OFFSET;
const ROCKET_NEAR_THETA = -28.5 * 180 / Math.PI;
const ROCKET_FAR_THETA = Math.PI - ROCKET_NEAR_THETA;

const HP_Y = ((26 * 12 + 10.5) / 2 - 25.9) * IN_TO_M;
const HP_THETA = Math.PI;

const HAB_LENGTH = 4 * FT_TO_M;
const HALF_HAB_3_WIDTH = 2 * FT_TO_M;
const HAB_2_WIDTH = (3 * 12 + 4) * IN_TO_M;
const HALF_HAB_1_WIDTH = (6 * 12 + 3.25) * IN_TO_M;
const HAB_1_LENGTH = (3 * 12 + 11.25) * IN_TO_M;

const DEPOT_WIDTH = (12 + 10.625) * IN_TO_M;

export function drawField(ctx: CanvasRenderingContext2D): void {
  drawTargets(ctx);
  drawHab(ctx);
}

function drawHab(ctx: CanvasRenderingContext2D): void {
  drawHalfHab(ctx);
  ctx.save();

  ctx.scale(1, -1);
  drawHalfHab(ctx);

  ctx.restore();
}

function drawHalfHab(ctx: CanvasRenderingContext2D): void {
  ctx.fillStyle = 'rgb(50, 50, 50)';
  ctx.fillRect(0, 0, HAB_LENGTH, HALF_HAB_3_WIDTH);
  ctx.fillStyle = 'rgb(100, 100, 100)';
  ctx.fillRect(0, HALF_HAB_3_WIDTH, HAB_LENGTH, HAB_2_WIDTH);
  ctx.fillStyle = 'rgb(200, 200, 200)';
  ctx.fillRect(HAB_LENGTH, 0, HAB_1_LENGTH, HALF_HAB_1_WIDTH);
  ctx.strokeRect(0, HALF_HAB_3_WIDTH + HAB_2_WIDTH, HAB_LENGTH, DEPOT_WIDTH);
}

function drawTargets(ctx: CanvasRenderingContext2D): void {
  drawHalfCargo(ctx);
  drawRocket(ctx);
  drawHP(ctx);
  ctx.save();

  ctx.scale(1, -1);
  drawHalfCargo(ctx);
  drawRocket(ctx);
  drawHP(ctx);

  ctx.restore();
}

function drawHP(ctx: CanvasRenderingContext2D): void {
  drawTarget(ctx, 0, HP_Y, HP_THETA);
}

function drawRocket(ctx: CanvasRenderingContext2D): void {
  drawTarget(ctx, ROCKET_PORT_X, ROCKET_PORT_Y, ROCKET_PORT_THETA);

  drawTarget(ctx, ROCKET_NEAR_X, ROCKET_HATCH_Y, ROCKET_NEAR_THETA);

  drawTarget(ctx, ROCKET_FAR_X, ROCKET_HATCH_Y, ROCKET_FAR_THETA);
}

function drawHalfCargo(ctx: CanvasRenderingContext2D): void {
  drawTarget(ctx, FAR_CARGO_X, SIDE_CARGO_Y, SIDE_CARGO_THETA);

  drawTarget(ctx, MID_CARGO_X, SIDE_CARGO_Y, SIDE_CARGO_THETA);

  drawTarget(ctx, NEAR_CARGO_X, SIDE_CARGO_Y, SIDE_CARGO_THETA);

  drawTarget(ctx, FACE_CARGO_X, FACE_CARGO_Y, FACE_CARGO_THETA);
}

export function drawTarget(
    ctx: CanvasRenderingContext2D, x: number, y: number, theta: number): void {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(theta);

  ctx.beginPath();
  ctx.moveTo(0, -0.15);
  ctx.lineTo(0, 0.15);
  ctx.moveTo(0, 0);
  ctx.lineTo(-0.15, 0);
  ctx.stroke();

  ctx.restore();
}
