import * as Colors from './colors';

// Multiplies all the values in the provided array by scale.
function scaleVec(vec: number[], scale: number): number[] {
  const scaled: number[] = [];
  for (let num of vec) {
    scaled.push(num * scale);
  }
  return scaled;
}

// Runs the operation op() over every pair of numbers in a, b and returns
// the result.
function cwiseOp(
    a: number[], b: number[], op: (a: number, b: number) => number): number[] {
  if (a.length !== b.length) {
    throw new Error("a and b must be of equal length.");
  }
  const min: number[] = [];
  for (let ii = 0; ii < a.length; ++ii) {
    min.push(op(a[ii], b[ii]));
  }
  return min;
}

// Adds vectors a and b.
function addVec(a: number[], b: number[]): number[] {
  return cwiseOp(a, b, (p, q) => {
    return p + q;
  });
}

function subtractVec(a: number[], b: number[]): number[] {
  return cwiseOp(a, b, (p, q) => {
    return p - q;
  });
}

function multVec(a: number[], b: number[]): number[] {
  return cwiseOp(a, b, (p, q) => {
    return p * q;
  });
}

function divideVec(a: number[], b: number[]): number[] {
  return cwiseOp(a, b, (p, q) => {
    return p / q;
  });
}

// Parameters used when scaling the lines to the canvas.
// If a point in a line is at pos then its position in the canvas space will be
// scale * pos + offset.
class ZoomParameters {
  public scale: number[] = [1.0, 1.0];
  public offset: number[] = [0.0, 0.0];
  copy():ZoomParameters {
    const copy = new ZoomParameters();
    copy.scale = [this.scale[0], this.scale[1]];
    copy.offset = [this.offset[0], this.offset[1]];
    return copy;
  }
}

export class Point {
  constructor(
  public x: number = 0.0,
  public y: number = 0.0) {}
}

// Represents a single line within a plot. Handles rendering the line with
// all of its points and the appropriate color/markers/lines.
export class Line {
  // Notes on zoom/precision management:
  // The adjustedPoints field is the buffert of points (formatted [x0, y0, x1,
  // y1, ..., xn, yn]) that will be read directly by WebGL and operated on in
  // the vertex shader. However, WebGL provides relatively minimal guarantess
  // about the floating point precision available in the shaders (to the point
  // where even Float32 precision is not guaranteed). As such, we
  // separately maintain the points vector using javascript number's
  // (arbitrary-precision ints or double-precision floats). We then periodically
  // set the baseZoom to be equal to the current desired zoom, calculate the
  // scaled values directly in typescript, store them in adjustedPoints, and
  // then just pass an identity transformation to WebGL for the zoom parameters.
  // When actively zooming, we then just use WebGL to compensate for the offset
  // between the baseZoom and the desired zoom, taking advantage of WebGL's
  // performance to handle the high-rate updates but then falling back to
  // typescript periodically to reset the offsets to avoid precision issues.
  //
  // As a practical matter, I've found that even if we were to recalculate
  // the zoom in typescript on every iteration, the penalty is relatively
  // minor--we still perform far better than using a non-WebGL canvas. This
  // suggests that the bulk of the performance advantage from using WebGL for
  // this use-case lies not in doing the zoom updates in the shaders, but rather
  // in relying on WebGL to figure out how to drawin the lines/points that we
  // specify.
  private adjustedPoints: Float32Array = new Float32Array([]);
  private points: Point[] = [];
  private _drawLine: boolean = true;
  private _pointSize: number = 3.0;
  private _hasUpdate: boolean = false;
  private _minValues: number[] = [Infinity, Infinity];
  private _maxValues: number[] = [-Infinity, -Infinity];
  private _color: number[] = [1.0, 0.0, 0.0];
  private pointAttribLocation: number;
  private colorLocation: WebGLUniformLocation | null;
  private pointSizeLocation: WebGLUniformLocation | null;
  private _label: string|null = null;
  private _hidden: boolean = false;
  constructor(
      private readonly ctx: WebGLRenderingContext,
      private readonly program: WebGLProgram,
      private readonly buffer: WebGLBuffer, private baseZoom: ZoomParameters) {
    this.pointAttribLocation = this.ctx.getAttribLocation(this.program, 'apos');
    this.colorLocation = this.ctx.getUniformLocation(this.program, 'color');
    this.pointSizeLocation =
        this.ctx.getUniformLocation(this.program, 'point_size');
  }

  // Return the largest x and y values present in the list of points.
  maxValues(): number[] {
    return this._maxValues;
  }

  // Return the smallest x and y values present in the list of points.
  minValues(): number[] {
    return this._minValues;
  }

  // Whether any parameters have changed that would require re-rending the line.
  hasUpdate(): boolean {
    return this._hasUpdate;
  }

  // Get/set the color of the line, returned as an RGB tuple.
  color(): number[] {
    return this._color;
  }

  setColor(newColor: number[]): Line {
    this._color = newColor;
    this._hasUpdate = true;
    return this;
  }

  // Get/set the size of the markers to draw, in pixels (zero means no markers).
  pointSize(): number {
    return this._pointSize;
  }

  setPointSize(size: number): Line {
    this._pointSize = size;
    this._hasUpdate = true;
    return this;
  }

  // Get/set whether we draw a line between the points (i.e., setting this to
  // false would effectively create a scatter-plot). If drawLine is false and
  // pointSize is zero, then no data is rendered.
  drawLine(): boolean {
    return this._drawLine;
  }

  setDrawLine(newDrawLine: boolean): Line {
    this._drawLine = newDrawLine;
    this._hasUpdate = true;
    return this;
  }

  // Set the points to render. The points in the line are ordered and should
  // be of the format:
  // [x1, y1, x2, y2, x3, y3, ...., xN, yN]
  setPoints(points: Point[]) {
    this.points = points;
    this.adjustedPoints = new Float32Array(points.length * 2);
    this.updateBaseZoom(this.baseZoom);
    this._hasUpdate = true;
    this._minValues[0] = Infinity;
    this._minValues[1] = Infinity;
    this._maxValues[0] = -Infinity;
    this._maxValues[1] = -Infinity;
    for (let ii = 0; ii < this.points.length; ++ii) {
      const x = this.points[ii].x;
      const y = this.points[ii].y;

      if (isNaN(x) || isNaN(y)) {
        continue;
      }

      this._minValues = cwiseOp(this._minValues, [x, y], Math.min);
      this._maxValues = cwiseOp(this._maxValues, [x, y], Math.max);
    }
  }

  hidden(): boolean {
    return this._hidden;
  }

  setHidden(hidden: boolean) {
    this._hasUpdate = true;
    this._hidden = hidden;
  }

  getPoints(): Point[] {
    return this.points;
  }

  // Get/set the label to use for the line when drawing the legend.
  setLabel(label: string): Line {
    this._label = label;
    return this;
  }

  label(): string|null {
    return this._label;
  }

  updateBaseZoom(zoom: ZoomParameters) {
    this.baseZoom = zoom;
    for (let ii = 0; ii < this.points.length; ++ii) {
      const point = this.points[ii];
      this.adjustedPoints[ii * 2] = point.x * zoom.scale[0] + zoom.offset[0];
      this.adjustedPoints[ii * 2 + 1] = point.y * zoom.scale[1] + zoom.offset[1];
    }
  }

  // Render the line on the canvas.
  draw() {
    this._hasUpdate = false;
    if (this.points.length === 0) {
      return;
    }

    if (this._hidden) {
      return;
    }

    this.ctx.bindBuffer(this.ctx.ARRAY_BUFFER, this.buffer);
    // Note: if this is generating errors associated with the buffer size,
    // confirm that this.points really is a Float32Array.
    this.ctx.bufferData(
        this.ctx.ARRAY_BUFFER,
        this.adjustedPoints,
        this.ctx.STATIC_DRAW);
    {
      const numComponents = 2;  // pull out 2 values per iteration
      const numType = this.ctx.FLOAT;    // the data in the buffer is 32bit floats
      const normalize = false;  // don't normalize
      const stride = 0;  // how many bytes to get from one set of values to the
                         // next 0 = use type and numComponents above
      const offset = 0;  // how many bytes inside the buffer to start from
      this.ctx.vertexAttribPointer(
          this.pointAttribLocation, numComponents, numType,
          normalize, stride, offset);
      this.ctx.enableVertexAttribArray(this.pointAttribLocation);
    }

    this.ctx.uniform1f(this.pointSizeLocation, this._pointSize);
    this.ctx.uniform4f(
        this.colorLocation, this._color[0], this._color[1], this._color[2],
        1.0);

    if (this._drawLine) {
      this.ctx.drawArrays(this.ctx.LINE_STRIP, 0, this.points.length);
    }
    if (this._pointSize > 0.0) {
      this.ctx.drawArrays(this.ctx.POINTS, 0, this.points.length);
    }
  }
}

enum MouseButton {
  Right,
  Middle,
  Left
}

// The button to use for panning the plot.
const PAN_BUTTON = MouseButton.Left;
const RECTANGLE_BUTTON = MouseButton.Right;

// Returns the mouse button that generated a given event.
function transitionButton(event: MouseEvent): MouseButton {
  switch (event.button) {
    case 0:
      return MouseButton.Left;
    case 1:
      return MouseButton.Middle;
    case 2:
      return MouseButton.Right;
  }
}

// Returns whether the given button is pressed on the mouse.
function buttonPressed(event: MouseEvent, button: MouseButton): boolean {
  switch (button) {
    // For some reason, the middle/right buttons are swapped relative to where
    // we would expect them to be given the .button field.
    case MouseButton.Left:
      return 0 !== (event.buttons & 0x1);
    case MouseButton.Right:
      return 0 !== (event.buttons & 0x2);
    case MouseButton.Middle:
      return 0 !== (event.buttons & 0x4);
  }
}

// Handles rendering a Legend for a list of lines.
// This takes a 2d canvas, which is what we use for rendering all the text of
// the plot and is separate, but overlayed on top of, the WebGL canvas that the
// lines are drawn on.
export class Legend {
  // Location, in pixels, of the legend in the text canvas.
  private location: number[] = [0, 0];
  constructor(
      private plot: Plot, private lines: Line[],
      private legend: HTMLDivElement) {
    this.setPosition([80, 30]);
  }

  setPosition(location: number[]): void {
    this.location = location;
    this.legend.style.left = location[0] + 'px';
    this.legend.style.top = location[1] + 'px';
  }

  draw(): void {
    // First, figure out if anything has changed.  The legend is created and
    // then titles are changed afterwords, so we have to do this lazily.
    let needsUpdate = false;
    {
      let child = 0;
      for (let line of this.lines) {
        if (line.label() === null) {
          continue;
        }

        if (child >= this.legend.children.length) {
          needsUpdate = true;
          break;
        }

        // Make sure both have text in the right spot.  Don't be too picky since
        // nothing should really be changing here, and it's handy to let the
        // user edit the HTML for testing.
        let textdiv = this.legend.children[child].lastChild;
        let canvas = this.legend.children[child].firstChild;
        if ((textdiv.textContent.length == 0 && line.label().length != 0) ||
            (textdiv as HTMLDivElement).offsetHeight !=
                (canvas as HTMLCanvasElement).height) {
          needsUpdate = true;
          break;
        }
        child += 1;
      }

      // If we got through everything, we should be pointed past the last child.
      // If not, more children exists than lines.
      if (child != this.legend.children.length) {
        needsUpdate = true;
      }
    }
    if (!needsUpdate) {
      return;
    }

    // Nuke the old legend.
    while (this.legend.firstChild) {
      this.legend.removeChild(this.legend.firstChild);
    }

    // Now, build up a new legend.
    for (let line of this.lines) {
      if (line.label() === null) {
        continue;
      }

      // The legend is a div containing both a canvas for the style/color, and a
      // div for the text.  Make those, color in the canvas, and add it to the
      // page.
      let l = document.createElement('div');
      l.classList.add('aos_legend_line');
      let text = document.createElement('div');
      text.textContent = line.label();

      l.appendChild(text);
      this.legend.appendChild(l);

      let c = document.createElement('canvas');
      c.width = text.offsetHeight;
      c.height = text.offsetHeight;

      const linestyleContext = c.getContext("2d");
      linestyleContext.clearRect(0, 0, c.width, c.height);

      const color = line.color();
      linestyleContext.strokeStyle = `rgb(${255.0 * color[0]}, ${
          255.0 * color[1]}, ${255.0 * color[2]})`;
      linestyleContext.fillStyle = linestyleContext.strokeStyle;

      const pointSize = line.pointSize();
      const kDistanceIn = pointSize / 2.0;

      if (line.drawLine()) {
        linestyleContext.beginPath();
        linestyleContext.moveTo(0, 0);
        linestyleContext.lineTo(c.height, c.width);
        linestyleContext.closePath();
        linestyleContext.stroke();
      }

      if (pointSize > 0) {
        linestyleContext.fillRect(0, 0, pointSize, pointSize);
        linestyleContext.fillRect(
            c.height - 1 - pointSize, c.width - 1 - pointSize, pointSize,
            pointSize);
      }

      c.addEventListener('click', (e) => {
        if (!line.hidden()) {
          l.classList.add('aos_legend_line_hidden');
        } else {
          l.classList.remove('aos_legend_line_hidden');
        }

        line.setHidden(!line.hidden());
        this.plot.draw();
      });

      l.prepend(c);
    }
  }
}

// This class manages all the WebGL rendering--namely, drawing the reference
// grid for the user and then rendering all the actual lines of the plot.
export class LineDrawer {
  private program: WebGLProgram|null = null;
  private scaleLocation: WebGLUniformLocation;
  private offsetLocation: WebGLUniformLocation;
  private vertexBuffer: WebGLBuffer;
  private lines: Line[] = [];
  private zoom: ZoomParameters = new ZoomParameters();
  private baseZoom: ZoomParameters = new ZoomParameters();
  private zoomUpdated: boolean = true;
  // Maximum grid lines to render at once--this is used provide an upper limit
  // on the number of Line objects we need to create in order to render the
  // grid.
  public readonly MAX_GRID_LINES: number = 5;
  // Arrays of the points at which we will draw grid lines for the x/y axes.
  private xTicks: number[] = [];
  private yTicks: number[] = [];
  private xGridLines: Line[] = [];
  private yGridLines: Line[] = [];

  public static readonly COLOR_CYCLE = [
    Colors.RED, Colors.GREEN, Colors.BLUE, Colors.BROWN, Colors.PINK,
    Colors.CYAN, Colors.WHITE, Colors.ORANGE, Colors.YELLOW
  ];
  private colorCycleIndex = 0;

  constructor(public readonly ctx: WebGLRenderingContext) {
    this.program = this.compileShaders();
    this.scaleLocation = this.ctx.getUniformLocation(this.program, 'scale');
    this.offsetLocation = this.ctx.getUniformLocation(this.program, 'offset');
    this.vertexBuffer = this.ctx.createBuffer();

    for (let ii = 0; ii < this.MAX_GRID_LINES; ++ii) {
      this.xGridLines.push(
          new Line(this.ctx, this.program, this.vertexBuffer, this.baseZoom));
      this.yGridLines.push(
          new Line(this.ctx, this.program, this.vertexBuffer, this.baseZoom));
    }
  }

  getZoom(): ZoomParameters {
    return this.zoom.copy();
  }

  plotToCanvasCoordinates(plotPos: number[]): number[] {
    return addVec(multVec(plotPos, this.zoom.scale), this.zoom.offset);
  }


  canvasToPlotCoordinates(canvasPos: number[]): number[] {
    return divideVec(subtractVec(canvasPos, this.zoom.offset), this.zoom.scale);
  }

  // These return the max/min rendered points, in plot-space (this is helpful
  // for drawing axis labels).
  maxVisiblePoint(): number[] {
    return this.canvasToPlotCoordinates([1.0, 1.0]);
  }

  minVisiblePoint(): number[] {
    return this.canvasToPlotCoordinates([-1.0, -1.0]);
  }

  getLines(): Line[] {
    return this.lines;
  }

  setZoom(zoom: ZoomParameters) {
    if (this.zoom.scale[0] == zoom.scale[0] &&
        this.zoom.scale[1] == zoom.scale[1] &&
        this.zoom.offset[0] == zoom.offset[0] &&
        this.zoom.offset[1] == zoom.offset[1]) {
      return;
    }
    this.zoomUpdated = true;
    this.zoom = zoom.copy();
  }

  setXTicks(ticks: number[]): void  {
    this.xTicks = ticks;
  }

  setYTicks(ticks: number[]): void  {
    this.yTicks = ticks;
  }

  // Update the grid lines.
  updateTicks() {
    for (let ii = 0; ii < this.MAX_GRID_LINES; ++ii) {
      this.xGridLines[ii].setPoints([]);
      this.yGridLines[ii].setPoints([]);
    }

    const minValues = this.minVisiblePoint();
    const maxValues = this.maxVisiblePoint();

    for (let ii = 0; ii < this.xTicks.length; ++ii) {
      this.xGridLines[ii].setColor([0.0, 0.0, 0.0]);
      const points = [
        new Point(this.xTicks[ii], minValues[1]),
        new Point(this.xTicks[ii], maxValues[1])
      ];
      this.xGridLines[ii].setPointSize(0);
      this.xGridLines[ii].setPoints(points);
      this.xGridLines[ii].draw();
    }

    for (let ii = 0; ii < this.yTicks.length; ++ii) {
      this.yGridLines[ii].setColor([0.0, 0.0, 0.0]);
      const points = [
        new Point(minValues[0], this.yTicks[ii]),
        new Point(maxValues[0], this.yTicks[ii])
      ];
      this.yGridLines[ii].setPointSize(0);
      this.yGridLines[ii].setPoints(points);
      this.yGridLines[ii].draw();
    }
  }

  // Handles redrawing any of the WebGL objects, if necessary.
  draw(): void  {
    let needsUpdate = this.zoomUpdated;
    this.zoomUpdated = false;
    for (let line of this.lines) {
      if (line.hasUpdate()) {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate) {
      return;
    }

    this.reset();

    this.updateTicks();

    for (let line of this.lines) {
      line.draw();
    }

    return;
  }

  loadShader(shaderType: number, source: string): WebGLShader {
    const shader = this.ctx.createShader(shaderType);
    this.ctx.shaderSource(shader, source);
    this.ctx.compileShader(shader);
    if (!this.ctx.getShaderParameter(shader, this.ctx.COMPILE_STATUS)) {
      alert(
          'Got an error compiling a shader: ' +
          this.ctx.getShaderInfoLog(shader));
      this.ctx.deleteShader(shader);
      return null;
    }

    return shader;
  }

  compileShaders(): WebGLProgram {
    const vertexShader = 'attribute vec2 apos;' +
        'uniform vec2 scale;' +
        'uniform vec2 offset;' +
        'uniform float point_size;' +
        'void main() {' +
        '  gl_Position.xy = apos.xy * scale.xy + offset.xy;' +
        '  gl_Position.z = 0.0;' +
        '  gl_Position.w = 1.0;' +
        '  gl_PointSize = point_size;' +
        '}';

    const fragmentShader = 'precision highp float;' +
        'uniform vec4 color;' +
        'void main() {' +
        '  gl_FragColor = color;' +
        '}';

    const compiledVertex =
        this.loadShader(this.ctx.VERTEX_SHADER, vertexShader);
    const compiledFragment =
        this.loadShader(this.ctx.FRAGMENT_SHADER, fragmentShader);
    const program = this.ctx.createProgram();
    this.ctx.attachShader(program, compiledVertex);
    this.ctx.attachShader(program, compiledFragment);
    this.ctx.linkProgram(program);
    if (!this.ctx.getProgramParameter(program, this.ctx.LINK_STATUS)) {
      alert(
          'Unable to link the shaders: ' + this.ctx.getProgramInfoLog(program));
      return null;
    }
    return program;
  }

  addLine(useColorCycle: boolean = true): Line {
    this.lines.push(
        new Line(this.ctx, this.program, this.vertexBuffer, this.baseZoom));
    const line = this.lines[this.lines.length - 1];
    if (useColorCycle) {
      line.setColor(LineDrawer.COLOR_CYCLE[this.colorCycleIndex++]);
    }
    return line;
  }

  minValues(): number[] {
    let minValues = [Infinity, Infinity];
    for (let line of this.lines) {
      minValues = cwiseOp(minValues, line.minValues(), Math.min);
    }
    return minValues;
  }

  maxValues(): number[] {
    let maxValues = [-Infinity, -Infinity];
    for (let line of this.lines) {
      maxValues = cwiseOp(maxValues, line.maxValues(), Math.max);
    }
    return maxValues;
  }

  reset(): void {
    // Set the background color
    this.ctx.clearColor(0.5, 0.5, 0.5, 1.0);
    this.ctx.clearDepth(1.0);
    this.ctx.enable(this.ctx.DEPTH_TEST);
    this.ctx.depthFunc(this.ctx.LEQUAL);
    this.ctx.clear(this.ctx.COLOR_BUFFER_BIT | this.ctx.DEPTH_BUFFER_BIT);

    this.ctx.useProgram(this.program);

    // Check for whether the zoom parameters have changed significantly; if so,
    // update the base zoom.
    // These thresholds are somewhat arbitrary.
    const scaleDiff = divideVec(this.zoom.scale, this.baseZoom.scale);
    const scaleChanged = scaleDiff[0] < 0.9 || scaleDiff[0] > 1.1 ||
        scaleDiff[1] < 0.9 || scaleDiff[1] > 1.1;
    const offsetDiff = subtractVec(this.zoom.offset, this.baseZoom.offset);
    // Note that offset is in the canvas coordinate frame and so just using
    // hard-coded constants is fine.
    const offsetChanged =
        Math.abs(offsetDiff[0]) > 0.1 || Math.abs(offsetDiff[1]) > 0.1;
    if (scaleChanged || offsetChanged) {
      this.baseZoom = this.zoom.copy();
      for (const line of this.lines) {
        line.updateBaseZoom(this.baseZoom);
      }
      for (const line of this.xGridLines) {
        line.updateBaseZoom(this.baseZoom);
      }
      for (const line of this.yGridLines) {
        line.updateBaseZoom(this.baseZoom);
      }
    }

    // all the points in the lines will be pre-scaled by this.baseZoom, so
    // we need to remove its effects before passing it in.
    // zoom.scale * pos + zoom.offset = scale * (baseZoom.scale * pos + baseZoom.offset) + offset
    // zoom.scale = scale * baseZoom.scale
    // scale = zoom.scale / baseZoom.scale
    // zoom.offset = scale * baseZoom.offset + offset
    // offset = zoom.offset - scale * baseZoom.offset
    const scale = divideVec(this.zoom.scale, this.baseZoom.scale);
    const offset =
        subtractVec(this.zoom.offset, multVec(scale, this.baseZoom.offset));
    this.ctx.uniform2f(
        this.scaleLocation, scale[0], scale[1]);
    this.ctx.uniform2f(
        this.offsetLocation, offset[0], offset[1]);
  }
}

// Class to store how much whitespace we put between the edges of the WebGL
// canvas (where we draw all the lines) and the edge of the plot. This gives
// us space to, e.g., draw axis labels, the plot title, etc.
class WhitespaceBuffers {
  constructor(
      public left: number, public right: number, public top: number,
      public bottom: number) {}
}

// Class to manage all the annotations associated with the plot--the axis/tick
// labels and the plot title.
class AxisLabels {
  private readonly INCREMENTS: number[] = [2, 4, 5, 10];
  // Space to leave to create some visual space around the text.
  private readonly TEXT_BUFFER: number = 5;
  private title: string = "";
  private xlabel: string = "";
  private ylabel: string = "";
  constructor(
      private ctx: CanvasRenderingContext2D, private drawer: LineDrawer,
      private graphBuffers: WhitespaceBuffers) {}

  numberToLabel(num: number): string {
    return num.toPrecision(5);
  }

  textWidth(str: string): number {
    return this.ctx.measureText(str).actualBoundingBoxRight;
  }

  textHeight(str: string): number {
    return this.ctx.measureText(str).actualBoundingBoxAscent;
  }

  textDepth(str: string): number {
    return this.ctx.measureText(str).actualBoundingBoxDescent;
  }

  setTitle(title: string) {
    this.title = title;
  }

  setXLabel(xlabel: string) {
    this.xlabel = xlabel;
  }

  setYLabel(ylabel: string) {
    this.ylabel = ylabel;
  }

  getIncrement(range: number[]): number {
    const diff = Math.abs(range[1] - range[0]);
    const minDiff = diff / this.drawer.MAX_GRID_LINES;
    const incrementsRatio = this.INCREMENTS[this.INCREMENTS.length - 1];
    const order = Math.pow(
        incrementsRatio,
        Math.floor(Math.log(minDiff) / Math.log(incrementsRatio)));
    const normalizedDiff = minDiff / order;
    for (let increment of this.INCREMENTS) {
      if (increment > normalizedDiff) {
        return increment * order;
      }
    }
    return 1.0;
  }

  getTicks(range: number[]): number[] {
    const increment = this.getIncrement(range);
    const start = Math.ceil(range[0] / increment) * increment;
    const values = [start];
    for (let ii = 0; ii < this.drawer.MAX_GRID_LINES - 1; ++ii) {
      const nextValue = values[ii] + increment;
      if (nextValue > range[1]) {
        break;
      }
      values.push(nextValue);
    }
    return values;
  }

  plotToCanvasCoordinates(plotPos: number[]): number[] {
    const webglCoord = this.drawer.plotToCanvasCoordinates(plotPos);
    const webglX = (webglCoord[0] + 1.0) / 2.0 * this.drawer.ctx.canvas.width;
    const webglY = (1.0 - webglCoord[1]) / 2.0 * this.drawer.ctx.canvas.height;
    return [webglX + this.graphBuffers.left, webglY + this.graphBuffers.top];
  }

  drawXTick(x: number) {
    const text = this.numberToLabel(x);
    const height = this.textHeight(text);
    const xpos = this.plotToCanvasCoordinates([x, 0])[0];
    this.ctx.textAlign = "center";
    this.ctx.fillText(
        text, xpos,
        this.ctx.canvas.height - this.graphBuffers.bottom + height +
            this.TEXT_BUFFER);
  }

  drawYTick(y: number) {
    const text = this.numberToLabel(y);
    const height = this.textHeight(text);
    const ypos = this.plotToCanvasCoordinates([0, y])[1];
    this.ctx.textAlign = "right";
    this.ctx.fillText(
        text, this.graphBuffers.left - this.TEXT_BUFFER,
        ypos + height / 2.0);
  }

  drawTitle() {
    if (this.title) {
      this.ctx.textAlign = 'center';
      this.ctx.fillText(
          this.title, this.ctx.canvas.width / 2.0,
          this.graphBuffers.top - this.TEXT_BUFFER);
    }
  }

  drawXLabel() {
    if (this.xlabel) {
      this.ctx.textAlign = 'center';
      this.ctx.fillText(
          this.xlabel, this.ctx.canvas.width / 2.0,
          this.ctx.canvas.height - this.TEXT_BUFFER);
    }
  }

  drawYLabel() {
    this.ctx.save();
    if (this.ylabel) {
      this.ctx.textAlign = 'center';
      const height = this.textHeight(this.ylabel);
      this.ctx.translate(
          height + this.TEXT_BUFFER, this.ctx.canvas.height / 2.0);
      this.ctx.rotate(-Math.PI / 2.0);
      this.ctx.fillText(this.ylabel, 0, 0);
    }
    this.ctx.restore();
  }

  draw() {
    this.ctx.fillStyle = 'black';
    const minValues = this.drawer.minVisiblePoint();
    const maxValues = this.drawer.maxVisiblePoint();
    let text = this.numberToLabel(maxValues[1]);
    this.drawYTick(maxValues[1]);
    this.drawYTick(minValues[1]);
    this.drawXTick(minValues[0]);
    this.drawXTick(maxValues[0]);
    this.ctx.strokeStyle = 'black';
    this.ctx.strokeRect(
        this.graphBuffers.left, this.graphBuffers.top,
        this.drawer.ctx.canvas.width, this.drawer.ctx.canvas.height);
    this.ctx.strokeRect(
        0, 0,
        this.ctx.canvas.width, this.ctx.canvas.height);
    const xTicks = this.getTicks([minValues[0], maxValues[0]]);
    this.drawer.setXTicks(xTicks);
    const yTicks = this.getTicks([minValues[1], maxValues[1]]);
    this.drawer.setYTicks(yTicks);

    for (let x of xTicks) {
      this.drawXTick(x);
    }

    for (let y of yTicks) {
      this.drawYTick(y);
    }

    this.drawTitle();
    this.drawXLabel();
    this.drawYLabel();
  }

  // Draws the current mouse position in the bottom-right of the plot.
  drawMousePosition(mousePos: number[]) {
    const plotPos = this.drawer.canvasToPlotCoordinates(mousePos);

    const text =
        `(${plotPos[0].toPrecision(10)}, ${plotPos[1].toPrecision(10)})`;
    const textDepth = this.textDepth(text);
    this.ctx.textAlign = 'right';
    this.ctx.fillText(
        text, this.ctx.canvas.width - this.graphBuffers.right,
        this.ctx.canvas.height - this.graphBuffers.bottom - textDepth);
  }
}

// This class manages the entirety of a single plot. Most of the logic in
// this class is around handling mouse/keyboard events for interacting with
// the plot.
export class Plot {
  private canvas = document.createElement('canvas');
  private textCanvas = document.createElement('canvas');
  private legendDiv = document.createElement('div');
  private lineDrawerContext: WebGLRenderingContext;
  private drawer: LineDrawer;
  private static keysPressed:
      object = {'x': false, 'y': false, 'Escape': false};
  // List of all plots to use for propagating key-press events to.
  private static allPlots: Plot[] = [];
  // In canvas coordinates (the +/-1 square).
  private lastMousePanPosition: number[]|null = null;
  private rectangleStartPosition: number[]|null = null;
  private axisLabelBuffer: WhitespaceBuffers =
      new WhitespaceBuffers(50, 20, 20, 30);
  private axisLabels: AxisLabels;
  private legend: Legend;
  private lastMousePosition: number[] = [0.0, 0.0];
  private autoFollow: boolean = true;
  private linkedXAxes: Plot[] = [];
  private lastTimeMs: number = 0;
  private defaultYRange: number[]|null = null;
  private zoomRectangle: Line;

  constructor(wrapperDiv: HTMLDivElement) {
    wrapperDiv.appendChild(this.canvas);
    wrapperDiv.appendChild(this.textCanvas);
    this.legendDiv.classList.add('aos_legend');
    wrapperDiv.appendChild(this.legendDiv);
    this.lastTimeMs = (new Date()).getTime();

    this.canvas.style.paddingLeft = this.axisLabelBuffer.left.toString() + "px";
    this.canvas.style.paddingRight = this.axisLabelBuffer.right.toString() + "px";
    this.canvas.style.paddingTop = this.axisLabelBuffer.top.toString() + "px";
    this.canvas.style.paddingBottom = this.axisLabelBuffer.bottom.toString() + "px";
    this.canvas.classList.add('aos_plot');

    this.lineDrawerContext = this.canvas.getContext('webgl');
    this.drawer = new LineDrawer(this.lineDrawerContext);

    this.textCanvas.classList.add('aos_plot_text');

    this.canvas.addEventListener('dblclick', (e) => {
      this.handleDoubleClick(e);
    });
    this.canvas.onwheel = (e) => {
      this.handleWheel(e);
      e.preventDefault();
    };
    this.canvas.onmousedown = (e) => {
      this.handleMouseDown(e);
    };
    this.canvas.onmouseup = (e) => {
      this.handleMouseUp(e);
    };
    this.canvas.onmousemove = (e) => {
      this.handleMouseMove(e);
    };
    this.canvas.addEventListener('contextmenu', event => event.preventDefault());
    // Note: To handle the fact that only one keypress handle can be registered
    // per browser tab, we share key-press handlers across all plot instances.
    Plot.allPlots.push(this);
    document.onkeydown = (e) => {
      Plot.handleKeyDown(e);
    };
    document.onkeyup = (e) => {
      Plot.handleKeyUp(e);
    };

    const textCtx = this.textCanvas.getContext("2d");
    this.axisLabels =
        new AxisLabels(textCtx, this.drawer, this.axisLabelBuffer);
    this.legend = new Legend(this, this.drawer.getLines(), this.legendDiv);

    this.zoomRectangle = this.getDrawer().addLine(false);
    this.zoomRectangle.setColor(Colors.WHITE);
    this.zoomRectangle.setPointSize(0);

    this.draw();
  }

  handleDoubleClick(event: MouseEvent) {
    this.resetZoom();
  }

  mouseCanvasLocation(event: MouseEvent): number[] {
    const computedStyle = window.getComputedStyle(this.canvas);
    const paddingLeftStr = computedStyle.getPropertyValue('padding-left');
    const paddingTopStr = computedStyle.getPropertyValue('padding-top');
    if (paddingLeftStr.substring(paddingLeftStr.length - 2) != "px") {
      throw new Error("Left padding should be specified in pixels.");
    }
    if (paddingTopStr.substring(paddingTopStr.length - 2) != "px") {
      throw new Error("Left padding should be specified in pixels.");
    }
    // Javascript will just ignore the extra "px".
    const paddingLeft = Number.parseInt(paddingLeftStr);
    const paddingTop = Number.parseInt(paddingTopStr);

    return [
      (event.offsetX - paddingLeft) * 2.0 / this.canvas.width - 1.0,
      -(event.offsetY - paddingTop) * 2.0 / this.canvas.height + 1.0
    ];
  }

  mousePlotLocation(event: MouseEvent): number[] {
    return this.drawer.canvasToPlotCoordinates(this.mouseCanvasLocation(event));
  }

  handleWheel(event: WheelEvent) {
    if (event.deltaMode !== event.DOM_DELTA_PIXEL) {
      return;
    }
    const mousePosition = this.mouseCanvasLocation(event);
    const kWheelTuningScalar = 1.5;
    const zoom = -kWheelTuningScalar * event.deltaY / this.canvas.height;
    let zoomScalar = 1.0 + Math.abs(zoom);
    if (zoom < 0.0) {
      zoomScalar = 1.0 / zoomScalar;
    }
    const scale = scaleVec(this.drawer.getZoom().scale, zoomScalar);
    const offset = addVec(
        scaleVec(mousePosition, 1.0 - zoomScalar),
        scaleVec(this.drawer.getZoom().offset, zoomScalar));
    this.setZoom(scale, offset);
  }

  handleMouseDown(event: MouseEvent) {
    for (let plot of this.linkedXAxes) {
      plot.autoFollow = false;
    }
    this.autoFollow = false;

    const button = transitionButton(event);
    switch (button) {
      case PAN_BUTTON:
        this.lastMousePanPosition = this.mouseCanvasLocation(event);
        break;
      case RECTANGLE_BUTTON:
        this.rectangleStartPosition = this.mousePlotLocation(event);
        break;
      default:
        break;
    }
  }

  handleMouseUp(event: MouseEvent) {
    const button = transitionButton(event);
    switch (button) {
      case PAN_BUTTON:
        this.lastMousePanPosition = null;
        break;
      case RECTANGLE_BUTTON:
        if (this.rectangleStartPosition === null) {
          // We got a right-button release without ever seeing the mouse-down;
          // just return.
          return;
        }
        this.finishRectangleZoom(event);
        break;
      default:
        break;
    }
  }

  private finishRectangleZoom(event: MouseEvent) {
    const currentPosition = this.mousePlotLocation(event);
    this.setZoomCorners(this.rectangleStartPosition, currentPosition);
    this.rectangleStartPosition = null;
    this.zoomRectangle.setPoints([]);
  }

  handleMouseMove(event: MouseEvent) {
    const mouseLocation = this.mouseCanvasLocation(event);
    if (buttonPressed(event, PAN_BUTTON) &&
        (this.lastMousePanPosition !== null)) {
      const mouseDiff =
          addVec(mouseLocation, scaleVec(this.lastMousePanPosition, -1));
      this.setZoom(
          this.drawer.getZoom().scale,
          addVec(this.drawer.getZoom().offset, mouseDiff));
      this.lastMousePanPosition = mouseLocation;
    }
    if (this.rectangleStartPosition !== null) {
      if (buttonPressed(event, RECTANGLE_BUTTON)) {
        // p0 and p1 are the two corners of the rectangle to draw.
        const p0 = [...this.rectangleStartPosition];
        const p1 = [...this.mousePlotLocation(event)];
        const minVisible = this.drawer.minVisiblePoint();
        const maxVisible = this.drawer.maxVisiblePoint();
        // Modify the rectangle corners to display correctly if we are limiting
        // the zoom to the x/y axis.
        const x_pressed = Plot.keysPressed['x'];
        const y_pressed = Plot.keysPressed["y"];
        if (x_pressed && !y_pressed) {
          p0[1] = minVisible[1];
          p1[1] = maxVisible[1];
        } else if (!x_pressed && y_pressed) {
          p0[0] = minVisible[0];
          p1[0] = maxVisible[0];
        }
        this.zoomRectangle.setPoints([
          new Point(p0[0], p0[1]), new Point(p0[0], p1[1]),
          new Point(p1[0], p1[1]), new Point(p1[0], p0[1]),
          new Point(p0[0], p0[1])
        ]);
      } else {
        this.finishRectangleZoom(event);
      }
    } else {
      this.zoomRectangle.setPoints([]);
    }
    this.lastMousePosition = mouseLocation;
  }

  setZoom(scale: number[], offset: number[]) {
    if (!isFinite(scale[0]) || !isFinite(scale[1])) {
      throw new Error("Doesn't support non-finite scales due to singularities.");
    }
    const x_pressed = Plot.keysPressed["x"];
    const y_pressed = Plot.keysPressed["y"];
    const zoom = this.drawer.getZoom();
    if (x_pressed && !y_pressed) {
      zoom.scale[0] = scale[0];
      zoom.offset[0] = offset[0];
    } else if (y_pressed && !x_pressed) {
      zoom.scale[1] = scale[1];
      zoom.offset[1] = offset[1];
    } else {
      zoom.scale = scale;
      zoom.offset = offset;
    }

    for (let plot of this.linkedXAxes) {
      const otherZoom = plot.drawer.getZoom();
      otherZoom.scale[0] = zoom.scale[0];
      otherZoom.offset[0] = zoom.offset[0];
      plot.drawer.setZoom(otherZoom);
      plot.autoFollow = false;
    }
    this.drawer.setZoom(zoom);
    this.autoFollow = false;
  }


  setZoomCorners(c1: number[], c2: number[]) {
    const scale = cwiseOp(c1, c2, (a, b) => {
      return 2.0 / Math.abs(a - b);
    });
    const offset = cwiseOp(scale, cwiseOp(c1, c2, Math.max), (a, b) => {
      return 1.0 - a * b;
    });
    this.setZoom(scale, offset);
  }

  setDefaultYRange(range: number[]|null) {
    if (range == null) {
      this.defaultYRange = null;
      return;
    }
    if (range.length != 2) {
      throw new Error('Range should contain exactly two values.');
    }
    this.defaultYRange = range;
  }

  resetZoom() {
    const minValues = this.drawer.minValues();
    const maxValues = this.drawer.maxValues();
    const kScalar = 0.05;
    for (const plot of this.linkedXAxes) {
      const otherMin = plot.drawer.minValues();
      const otherMax = plot.drawer.maxValues();
      // For linked x-axes, only adjust the x limits.
      minValues[0] = Math.min(minValues[0], otherMin[0]);
      maxValues[0] = Math.max(maxValues[0], otherMax[0]);
    }
    if (!isFinite(minValues[0]) || !isFinite(maxValues[0])) {
      minValues[0] = 0;
      maxValues[0] = 0;
    }
    if (!isFinite(minValues[1]) || !isFinite(maxValues[1])) {
      minValues[1] = 0;
      maxValues[1] = 0;
    }
    if (minValues[0] == maxValues[0]) {
      minValues[0] -= 1;
      maxValues[0] += 1;
    } else {
      const width = maxValues[0] - minValues[0];
      maxValues[0] += width * kScalar;
      minValues[0] -= width * kScalar;
    }
    if (minValues[1] == maxValues[1]) {
      minValues[1] -= 1;
      maxValues[1] += 1;
    } else {
      const height = maxValues[1] - minValues[1];
      maxValues[1] += height * kScalar;
      minValues[1] -= height * kScalar;
    }
    if (this.defaultYRange != null) {
      minValues[1] = this.defaultYRange[0];
      maxValues[1] = this.defaultYRange[1];
    }
    this.setZoomCorners(minValues, maxValues);
    this.autoFollow = true;
    for (let plot of this.linkedXAxes) {
      plot.autoFollow = true;
    }
  }

  static handleKeyUp(event: KeyboardEvent) {
    Plot.keysPressed[event.key] = false;
  }

  static handleKeyDown(event: KeyboardEvent) {
    Plot.keysPressed[event.key] = true;
    for (const plot of this.allPlots) {
      if (Plot.keysPressed['Escape']) {
        // Cancel zoom/pan operations on escape.
        plot.lastMousePanPosition = null;
        plot.rectangleStartPosition = null;
        plot.zoomRectangle.setPoints([]);
      }
    }
  }

  draw() {
    window.requestAnimationFrame(() => this.draw());
    const curTime = (new Date()).getTime();
    const frameRate = 1000.0 / (curTime - this.lastTimeMs);
    this.lastTimeMs = curTime;
    const parentWidth = this.textCanvas.parentElement.offsetWidth;
    const parentHeight = this.textCanvas.parentElement.offsetHeight;
    this.textCanvas.width = parentWidth;
    this.textCanvas.height = parentHeight;
    this.canvas.width =
        parentWidth - this.axisLabelBuffer.left - this.axisLabelBuffer.right;
    this.canvas.height =
        parentHeight - this.axisLabelBuffer.top - this.axisLabelBuffer.bottom;
    this.lineDrawerContext.viewport(
        0, 0, this.lineDrawerContext.drawingBufferWidth,
        this.lineDrawerContext.drawingBufferHeight);

    // Clear the overlay.
    const textCtx = this.textCanvas.getContext("2d");
    textCtx.clearRect(0, 0, this.textCanvas.width, this.textCanvas.height);

    this.axisLabels.draw();
    this.axisLabels.drawMousePosition(this.lastMousePosition);
    this.legend.draw();

    this.drawer.draw();

    if (this.autoFollow) {
      this.resetZoom();
    }
  }

  getDrawer(): LineDrawer {
    return this.drawer;
  }

  getLegend(): Legend {
    return this.legend;
  }

  getAxisLabels(): AxisLabels {
    return this.axisLabels;
  }

  // Links this plot's x-axis with that of another Plot (e.g., to share time
  // axes).
  linkXAxis(other: Plot) {
    this.linkedXAxes.push(other);
    other.linkedXAxes.push(this);
  }
}
