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

// Represents a single line within a plot. Handles rendering the line with
// all of its points and the appropriate color/markers/lines.
export class Line {
  private points: Float32Array = new Float32Array([]);
  private _drawLine: boolean = true;
  private _pointSize: number = 3.0;
  private _hasUpdate: boolean = false;
  private _minValues: number[] = [0.0, 0.0];
  private _maxValues: number[] = [0.0, 0.0];
  private _color: number[] = [1.0, 0.0, 0.0];
  private pointAttribLocation: number;
  private colorLocation: WebGLUniformLocation | null;
  private pointSizeLocation: WebGLUniformLocation | null;
  private _label: string = "";
  constructor(
      private readonly ctx: WebGLRenderingContext,
      private readonly program: WebGLProgram,
      private readonly buffer: WebGLBuffer) {
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

  setColor(newColor: number[]) {
    this._color = newColor;
    this._hasUpdate = true;
  }

  // Get/set the size of the markers to draw, in pixels (zero means no markers).
  pointSize(): number {
    return this._pointSize;
  }

  setPointSize(size: number) {
    this._pointSize = size;
    this._hasUpdate = true;
  }

  // Get/set whether we draw a line between the points (i.e., setting this to
  // false would effectively create a scatter-plot). If drawLine is false and
  // pointSize is zero, then no data is rendered.
  drawLine(): boolean {
    return this._drawLine;
  }

  setDrawLine(newDrawLine: boolean) {
    this._drawLine = newDrawLine;
    this._hasUpdate = true;
  }

  // Set the points to render. The points in the line are ordered and should
  // be of the format:
  // [x1, y1, x2, y2, x3, y3, ...., xN, yN]
  setPoints(points: Float32Array) {
    if (points.length % 2 !== 0) {
      throw new Error("Must have even number of elements in points array.");
    }
    if (points.BYTES_PER_ELEMENT != 4) {
      throw new Error(
          'Must pass in a Float32Array--actual size was ' +
          points.BYTES_PER_ELEMENT + '.');
    }
    this.points = points;
    this._hasUpdate = true;
    this._minValues[0] = Infinity;
    this._minValues[1] = Infinity;
    this._maxValues[0] = -Infinity;
    this._maxValues[1] = -Infinity;
    for (let ii = 0; ii < this.points.length; ii += 2) {
      const x = this.points[ii];
      const y = this.points[ii + 1];

      this._minValues = cwiseOp(this._minValues, [x, y], Math.min);
      this._maxValues = cwiseOp(this._maxValues, [x, y], Math.max);
    }
  }

  // Get/set the label to use for the line when drawing the legend.
  setLabel(label: string) {
    this._label = label;
  }

  label(): string {
    return this._label;
  }

  // Render the line on the canvas.
  draw() {
    this._hasUpdate = false;
    if (this.points.length === 0) {
      return;
    }

    this.ctx.bindBuffer(this.ctx.ARRAY_BUFFER, this.buffer);
    // Note: if this is generating errors associated with the buffer size,
    // confirm that this.points really is a Float32Array.
    this.ctx.bufferData(
        this.ctx.ARRAY_BUFFER,
        this.points,
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
      this.ctx.drawArrays(this.ctx.LINE_STRIP, 0, this.points.length / 2);
    }
    if (this._pointSize > 0.0) {
      this.ctx.drawArrays(this.ctx.POINTS, 0, this.points.length / 2);
    }
  }
}

// Parameters used when scaling the lines to the canvas.
// If a point in a line is at pos then its position in the canvas space will be
// scale * pos + offset.
class ZoomParameters {
  public scale: number[] = [1.0, 1.0];
  public offset: number[] = [0.0, 0.0];
}

enum MouseButton {
  Right,
  Middle,
  Left
}

// The button to use for panning the plot.
const PAN_BUTTON = MouseButton.Left;

// Returns the mouse button that generated a given event.
function transitionButton(event: MouseEvent): MouseButton {
  switch (event.button) {
    case 0:
      return MouseButton.Left;
    case 1:
      return MouseButton.Right;
    case 2:
      return MouseButton.Middle;
  }
}

// Returns whether the given button is pressed on the mouse.
function buttonPressed(event: MouseEvent, button: MouseButton): boolean {
  switch (button) {
    // For some reason, the middle/right buttons are swapped relative to where
    // we would expect them to be given the .button field.
    case MouseButton.Left:
      return 0 !== (event.buttons & 0x1);
    case MouseButton.Middle:
      return 0 !== (event.buttons & 0x2);
    case MouseButton.Right:
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
  constructor(private ctx: CanvasRenderingContext2D, private lines: Line[]) {
    this.location = [this.ctx.canvas.width - 100, 30];
  }

  setPosition(location: number[]): void {
    this.location = location;
  }

  draw(): void {
    this.ctx.save();

    this.ctx.translate(this.location[0], this.location[1]);

    // Space between rows of the legend.
    const step = 20;

    // Total height of the body of the legend.
    const height = step * this.lines.length;

    let maxWidth = 0;

    // In the legend, we render both a small line of the appropriate color as
    // well as the text label--start/endPoint are the relative locations of the
    // endpoints of the miniature line within the row, and textStart is where
    // we begin rendering the text within the row.
    const startPoint = [0, 0];
    const endPoint = [10, -10];
    const textStart = endPoint[0] + 5;

    // Calculate how wide the legend needs to be to fit all the text.
    this.ctx.textAlign = 'left';
    for (let line of this.lines) {
      const width =
          textStart + this.ctx.measureText(line.label()).actualBoundingBoxRight;
      maxWidth = Math.max(width, maxWidth);
    }

    // Set the legend background to be white and opaque.
    this.ctx.fillStyle = 'rgba(255, 255, 255, 1.0)';
    const backgroundBuffer = 5;
    this.ctx.fillRect(
        -backgroundBuffer, 0, maxWidth + 2.0 * backgroundBuffer,
        height + backgroundBuffer);

    // Go through each line and render the little lines and text for each Line.
    for (let line of this.lines) {
      this.ctx.translate(0, step);
      const color = line.color();
      this.ctx.strokeStyle = `rgb(${255.0 * color[0]}, ${255.0 * color[1]}, ${255.0 * color[2]})`;
      this.ctx.fillStyle = this.ctx.strokeStyle;
      if (line.drawLine()) {
        this.ctx.beginPath();
        this.ctx.moveTo(startPoint[0], startPoint[1]);
        this.ctx.lineTo(endPoint[0], endPoint[1]);
        this.ctx.closePath();
        this.ctx.stroke();
      }
      const pointSize = line.pointSize();
      if (pointSize > 0) {
        this.ctx.fillRect(
            startPoint[0] - pointSize / 2.0, startPoint[1] - pointSize / 2.0,
            pointSize, pointSize);
        this.ctx.fillRect(
            endPoint[0] - pointSize / 2.0, endPoint[1] - pointSize / 2.0,
            pointSize, pointSize);
      }

      this.ctx.fillStyle = 'black';
      this.ctx.textAlign = 'left';
      this.ctx.fillText(line.label(), textStart, 0);
    }

    this.ctx.restore();
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

  constructor(public readonly ctx: WebGLRenderingContext) {
    this.program = this.compileShaders();
    this.scaleLocation = this.ctx.getUniformLocation(this.program, 'scale');
    this.offsetLocation = this.ctx.getUniformLocation(this.program, 'offset');
    this.vertexBuffer = this.ctx.createBuffer();

    for (let ii = 0; ii < this.MAX_GRID_LINES; ++ii) {
      this.xGridLines.push(new Line(this.ctx, this.program, this.vertexBuffer));
      this.yGridLines.push(new Line(this.ctx, this.program, this.vertexBuffer));
    }
  }

  setXGrid(lines: Line[]) {
    this.xGridLines = lines;
  }

  getZoom(): ZoomParameters {
    return this.zoom;
  }

  plotToCanvasCoordinates(plotPos: number[]): number[] {
    return addVec(cwiseOp(plotPos, this.zoom.scale, (a, b) => {
                    return a * b;
                  }), this.zoom.offset);
  }


  canvasToPlotCoordinates(canvasPos: number[]): number[] {
    return cwiseOp(cwiseOp(canvasPos, this.zoom.offset, (a, b) => {
                     return a - b;
                   }), this.zoom.scale, (a, b) => {
      return a / b;
    });
  }

  // Tehse return the max/min rendered points, in plot-space (this is helpful
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
    this.zoomUpdated = true;
    this.zoom = zoom;
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
      this.xGridLines[ii].setPoints(new Float32Array([]));
      this.yGridLines[ii].setPoints(new Float32Array([]));
    }

    const minValues = this.minVisiblePoint();
    const maxValues = this.maxVisiblePoint();

    for (let ii = 0; ii < this.xTicks.length; ++ii) {
      this.xGridLines[ii].setColor([0.0, 0.0, 0.0]);
      const points = new Float32Array(
          [this.xTicks[ii], minValues[1], this.xTicks[ii], maxValues[1]]);
      this.xGridLines[ii].setPointSize(0);
      this.xGridLines[ii].setPoints(points);
      this.xGridLines[ii].draw();
    }

    for (let ii = 0; ii < this.yTicks.length; ++ii) {
      this.yGridLines[ii].setColor([0.0, 0.0, 0.0]);
      const points = new Float32Array(
          [minValues[0], this.yTicks[ii], maxValues[0], this.yTicks[ii]]);
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

  addLine(): Line {
    this.lines.push(new Line(this.ctx, this.program, this.vertexBuffer));
    return this.lines[this.lines.length - 1];
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

    this.ctx.uniform2f(
        this.scaleLocation, this.zoom.scale[0], this.zoom.scale[1]);
    this.ctx.uniform2f(
        this.offsetLocation, this.zoom.offset[0], this.zoom.offset[1]);
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
  private drawer: LineDrawer;
  private static keysPressed: object = {'x': false, 'y': false};
  // In canvas coordinates (the +/-1 square).
  private lastMousePanPosition: number[] = null;
  private axisLabelBuffer: WhitespaceBuffers =
      new WhitespaceBuffers(50, 20, 20, 30);
  private axisLabels: AxisLabels;
  private legend: Legend;
  private lastMousePosition: number[] = [0.0, 0.0];
  private autoFollow: boolean = true;
  private linkedXAxes: Plot[] = [];

  constructor(wrapperDiv: HTMLDivElement, width: number, height: number) {
    wrapperDiv.appendChild(this.canvas);
    wrapperDiv.appendChild(this.textCanvas);

    this.canvas.width =
        width - this.axisLabelBuffer.left - this.axisLabelBuffer.right;
    this.canvas.height =
        height - this.axisLabelBuffer.top - this.axisLabelBuffer.bottom;
    this.canvas.style.left = this.axisLabelBuffer.left.toString();
    this.canvas.style.top = this.axisLabelBuffer.top.toString();
    this.canvas.style.position = 'absolute';
    this.drawer = new LineDrawer(this.canvas.getContext('webgl'));

    this.textCanvas.width = width;
    this.textCanvas.height = height;
    this.textCanvas.style.left = '0';
    this.textCanvas.style.top = '0';
    this.textCanvas.style.position = 'absolute';
    this.textCanvas.style.pointerEvents = 'none';

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
    // TODO(james): Deconflict the global state....
    document.onkeydown = (e) => {
      this.handleKeyDown(e);
    };
    document.onkeyup = (e) => {
      this.handleKeyUp(e);
    };

    const textCtx = this.textCanvas.getContext("2d");
    this.axisLabels =
        new AxisLabels(textCtx, this.drawer, this.axisLabelBuffer);
    this.legend = new Legend(textCtx, this.drawer.getLines());

    this.draw();
  }

  handleDoubleClick(event: MouseEvent) {
    this.resetZoom();
  }

  mouseCanvasLocation(event: MouseEvent): number[] {
    return [
      event.offsetX * 2.0 / this.canvas.width - 1.0,
      -event.offsetY * 2.0 / this.canvas.height + 1.0
    ];
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
    if (transitionButton(event) === PAN_BUTTON) {
      this.lastMousePanPosition = this.mouseCanvasLocation(event);
    }
  }

  handleMouseUp(event: MouseEvent) {
    if (transitionButton(event) === PAN_BUTTON) {
      this.lastMousePanPosition = null;
    }
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
    this.lastMousePosition = mouseLocation;
  }

  setZoom(scale: number[], offset: number[]) {
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

  resetZoom() {
    this.setZoomCorners(this.drawer.minValues(), this.drawer.maxValues());
    this.autoFollow = true;
    for (let plot of this.linkedXAxes) {
      plot.autoFollow = true;
    }
  }

  handleKeyUp(event: KeyboardEvent) {
    Plot.keysPressed[event.key] = false;
  }

  handleKeyDown(event: KeyboardEvent) {
    Plot.keysPressed[event.key] = true;
  }

  draw() {
    window.requestAnimationFrame(() => this.draw());

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
