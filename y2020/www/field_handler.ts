import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_CENTER_X = (198.75 + 116) * IN_TO_M;

const DS_WIDTH = 69 * IN_TO_M;
const DS_ANGLE = 20 * Math.PI / 180;
const DS_END_X = DS_WIDTH * Math.sin(DS_ANGLE);
const OTHER_DS_X = FIELD_LENGTH - DS_END_X;
const DS_INSIDE_Y = FIELD_SIDE_Y - DS_WIDTH * Math.cos(DS_ANGLE);

const TRENCH_START_X = 206.57 * IN_TO_M;
const TRENCH_END_X = FIELD_LENGTH - TRENCH_START_X;
const TRENCH_WIDTH = 55.5 * IN_TO_M;
const TRENCH_INSIDE = FIELD_SIDE_Y - TRENCH_WIDTH;

const SPINNER_LENGTH = 30 * IN_TO_M;
const SPINNER_TOP_X = 374.59 * IN_TO_M;
const SPINNER_BOTTOM_X = SPINNER_TOP_X - SPINNER_LENGTH;

const SHIELD_BOTTOM_X = FIELD_CENTER_X - 116 * IN_TO_M;
const SHIELD_BOTTOM_Y = 43.75 * IN_TO_M;

const SHIELD_TOP_X = FIELD_CENTER_X + 116 * IN_TO_M;
const SHIELD_TOP_Y = -43.75 * IN_TO_M;

const SHIELD_RIGHT_X = FIELD_CENTER_X - 51.06 * IN_TO_M;
const SHIELD_RIGHT_Y = -112.88 * IN_TO_M;

const SHIELD_LEFT_X = FIELD_CENTER_X + 51.06 * IN_TO_M;
const SHIELD_LEFT_Y = 112.88 * IN_TO_M;

const SHIELD_CENTER_TOP_X = (SHIELD_TOP_X + SHIELD_LEFT_X) / 2
const SHIELD_CENTER_TOP_Y = (SHIELD_TOP_Y + SHIELD_LEFT_Y) / 2

const SHIELD_CENTER_BOTTOM_X = (SHIELD_BOTTOM_X + SHIELD_RIGHT_X) / 2
const SHIELD_CENTER_BOTTOM_Y = (SHIELD_BOTTOM_Y + SHIELD_RIGHT_Y) / 2

const INITIATION_X = 120 * IN_TO_M;
const FAR_INITIATION_X = FIELD_LENGTH - 120 * IN_TO_M;

const TARGET_ZONE_TIP_X = 30 * IN_TO_M;
const TARGET_ZONE_WIDTH = 48 * IN_TO_M;
const LOADING_ZONE_WIDTH = 60 * IN_TO_M;

export class FieldHandler {
  private canvas = document.createElement('canvas');

  constructor() {
    document.body.appendChild(this.canvas);
  }

  drawField(): void {
    const MY_COLOR = 'red';
    const OTHER_COLOR = 'blue';
    const ctx = this.canvas.getContext('2d');
    // draw perimiter
    ctx.beginPath();
    ctx.moveTo(0, DS_INSIDE_Y);
    ctx.lineTo(DS_END_X, FIELD_SIDE_Y);
    ctx.lineTo(OTHER_DS_X, FIELD_SIDE_Y);
    ctx.lineTo(FIELD_LENGTH, DS_INSIDE_Y);
    ctx.lineTo(FIELD_LENGTH, -DS_INSIDE_Y);
    ctx.lineTo(OTHER_DS_X, -FIELD_SIDE_Y);
    ctx.lineTo(DS_END_X, -FIELD_SIDE_Y);
    ctx.lineTo(0, -DS_INSIDE_Y);
    ctx.lineTo(0, DS_INSIDE_Y);
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

    // draw trenches
    ctx.strokeStyle = MY_COLOR;
    ctx.beginPath();
    ctx.moveTo(TRENCH_START_X, FIELD_SIDE_Y);
    ctx.lineTo(TRENCH_START_X, TRENCH_INSIDE);
    ctx.lineTo(TRENCH_END_X, TRENCH_INSIDE);
    ctx.lineTo(TRENCH_END_X, FIELD_SIDE_Y);
    ctx.stroke();

    ctx.strokeStyle = OTHER_COLOR;
    ctx.beginPath();
    ctx.moveTo(TRENCH_START_X, -FIELD_SIDE_Y);
    ctx.lineTo(TRENCH_START_X, -TRENCH_INSIDE);
    ctx.lineTo(TRENCH_END_X, -TRENCH_INSIDE);
    ctx.lineTo(TRENCH_END_X, -FIELD_SIDE_Y);
    ctx.stroke();

    ctx.strokeStyle = 'black';
    ctx.beginPath();
    ctx.moveTo(SPINNER_TOP_X, FIELD_SIDE_Y);
    ctx.lineTo(SPINNER_TOP_X, TRENCH_INSIDE);
    ctx.lineTo(SPINNER_BOTTOM_X, TRENCH_INSIDE);
    ctx.lineTo(SPINNER_BOTTOM_X, FIELD_SIDE_Y);
    ctx.moveTo(FIELD_LENGTH - SPINNER_TOP_X, -FIELD_SIDE_Y);
    ctx.lineTo(FIELD_LENGTH - SPINNER_TOP_X, -TRENCH_INSIDE);
    ctx.lineTo(FIELD_LENGTH - SPINNER_BOTTOM_X, -TRENCH_INSIDE);
    ctx.lineTo(FIELD_LENGTH - SPINNER_BOTTOM_X, -FIELD_SIDE_Y);
    ctx.stroke();

    // draw initiation lines
    ctx.beginPath();
    ctx.moveTo(INITIATION_X, FIELD_SIDE_Y);
    ctx.lineTo(INITIATION_X, -FIELD_SIDE_Y);
    ctx.moveTo(FAR_INITIATION_X, FIELD_SIDE_Y);
    ctx.lineTo(FAR_INITIATION_X, -FIELD_SIDE_Y);
    ctx.stroke();

    // draw target/loading zones
    ctx.strokeStyle = MY_COLOR;
    ctx.beginPath();
    ctx.moveTo(0, DS_INSIDE_Y);
    ctx.lineTo(TARGET_ZONE_TIP_X, DS_INSIDE_Y - 0.5 * TARGET_ZONE_WIDTH);
    ctx.lineTo(0, DS_INSIDE_Y - TARGET_ZONE_WIDTH);

    ctx.moveTo(FIELD_LENGTH, DS_INSIDE_Y);
    ctx.lineTo(
        FIELD_LENGTH - TARGET_ZONE_TIP_X,
        DS_INSIDE_Y - 0.5 * LOADING_ZONE_WIDTH);
    ctx.lineTo(FIELD_LENGTH, DS_INSIDE_Y - LOADING_ZONE_WIDTH);
    ctx.stroke();

    ctx.strokeStyle = OTHER_COLOR;
    ctx.beginPath();
    ctx.moveTo(0, -DS_INSIDE_Y);
    ctx.lineTo(TARGET_ZONE_TIP_X, -(DS_INSIDE_Y - 0.5 * LOADING_ZONE_WIDTH));
    ctx.lineTo(0, -(DS_INSIDE_Y - LOADING_ZONE_WIDTH));

    ctx.moveTo(FIELD_LENGTH, -DS_INSIDE_Y);
    ctx.lineTo(
        FIELD_LENGTH - TARGET_ZONE_TIP_X,
        -(DS_INSIDE_Y - 0.5 * TARGET_ZONE_WIDTH));
    ctx.lineTo(FIELD_LENGTH, -(DS_INSIDE_Y - TARGET_ZONE_WIDTH));
    ctx.stroke();
  }

  reset(): void {
    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const size = window.innerHeight * 0.9;
    ctx.canvas.height = size;
    ctx.canvas.width = size / 2 + 10;
    ctx.clearRect(0, 0, size, size / 2 + 10);

    // Translate to center of bottom of display.
    ctx.translate(size / 4, size);
    // Coordinate system is:
    // x -> forward.
    // y -> to the left.
    ctx.rotate(-Math.PI / 2);
    ctx.scale(1, -1);
    ctx.translate(5, 0);

    const M_TO_PX = (size - 10) / FIELD_LENGTH;
    ctx.scale(M_TO_PX, M_TO_PX);
    ctx.lineWidth = 1 / M_TO_PX;
  }
}
