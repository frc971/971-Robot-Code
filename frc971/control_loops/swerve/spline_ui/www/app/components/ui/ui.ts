import type { Spline, Point, SplineControl, OptiData, ActionInfo, Constraint, GlobalConstraint, StopPoint } from './type'
import { Component, ViewChild, ElementRef, AfterViewInit } from '@angular/core'
import { Context, StopPointContext, ConstraintContext, GlobalContext, ActionContext, RotationContext } from './context'
import { ChangeEvent } from './changeEvent'
import { HttpClient } from '@angular/common/http';

type Mode = "view" | "add G1 spline" | "add G0 spline" | "edit rotation breakpoints" | "edit section constraints" | "edit actions"
type Field = {
  name: string,
  year: number,
  imagePath: string,
  imageElement: HTMLImageElement | null,
  field_size: [number, number],
}
const splineSubdivisions = 1000
const splineColors = ["#A00000", "#00A0A0", "#A0A000", "#0000A0", "#00A000", "#A000A0"]
//TODO:
//Load file resets all selections
//Display Voltage
//Have y-axis tick marks
//Make stop point spline allowable as starting spline
//More keybinds (and fix existing ones to only work when they should)
//Make rotation points selectable (already added to context menu)
//make mode selection a dropdown (clean up buttons)
//make clear() work with selections and actions (actually clear everything lol)
//make constraints not able to be dragged to negative length
//highlight selected action
//clean up gui to make selectables/draggables more clear (big - split into subtasks)
//show first point in new constraint (maybe even show whole thing!)

//Code-cleaning TODO:
//revamp 'grab' system to make simpler (probably goes with cleaned-up ui)
//separate canvas into its own component
//separate ContextMenu into its own component (?)
//move dropdown into own component (?)
//fix naming convention for 'location' vs 't' vs 'time' (too much overlap between t and others)

@Component({
  selector: 'spline-ui',
  templateUrl: './ui.ng.html',
  styleUrls: ['../../common.css'],
})

export class Ui implements AfterViewInit {
  @ViewChild("canvas")
  public canvasref: ElementRef<HTMLCanvasElement>
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D
  @ViewChild("playbarcircle")
  public playbarcircleref: ElementRef<SVGElement>
  private playbarcircle: SVGElement
  @ViewChild("playbar")
  public playbarref: ElementRef<SVGElement>
  private playbar: SVGElement
  @ViewChild("displayForces")
  public displayForcesRef: ElementRef<HTMLInputElement>
  private displayForces: HTMLInputElement
  @ViewChild("displayVelocities")
  public displayVelocitiesRef: ElementRef<HTMLInputElement>
  private displayVelocities: HTMLInputElement

  public splines: Spline[] = []
  private pointsToAdd: number = 0
  private addedPoints: Point[] = []
  private grabbedPoint: Point | null = null
  private grabbedSpline: Spline | null = null
  private movableGrabbedSpline: Spline | null = null
  private grabbedRotPointLen: number | null = null
  private grabbedRotPointRot: number | null = null
  private closeCurvePoint: Point | null = null
  private closeCurveT: number | null = null
  public mode: Mode = "view"
  public rotbreakpoints: [number, number][] = [[0, 0], [1, 0]]
  public staticrotbreakpoints: number[] = [this.rotbreakpoints[0][0], this.rotbreakpoints[1][0]]
  private panning: boolean = false

  public actions: ActionInfo[] = []
  public grabbedAction: ActionInfo = null;

  private robot_size: [number, number] = [30 * 0.0254, 30 * 0.0254]
  public field: Field
  public fields: Field[] = []

  public playing: boolean = false
  public playBarT: number = 0.0
  public hoverT: number = null

  public totalTime: number = null
  public optidata: OptiData = null
  public loading: boolean = false
  public draggingCircle: boolean = false

  public playspeed: number = 1

  private undoEventStack: ChangeEvent[] = []
  private redoEventStack: ChangeEvent[] = []

  public changeInProgress: ChangeEvent

  public showDisplaySettings: boolean = false
  private showForces: boolean = false
  private showVelocities: boolean = true

  private lastCalledTime

  public constraints: Constraint[] = []
  private globalConstraints: GlobalConstraint = {
    max_velocity: null,
    max_acceleration: null,
    max_current: 40,
    max_voltage: 12,
  }

  public selectedContext: Context = new GlobalContext(this.globalConstraints, this)

  private firstAddedConstraintNode: number = null
  private selectedConstraint: Constraint = null
  private grabbedConstraint: Constraint = null
  private grabbedConstraintIndex: 0 | 1 = null

  public pathOutOfDate: boolean = false

  private selectedStopPoint: StopPoint = null
  public stopPoints: StopPoint[] = []

  public keybindsenabled: boolean = true;

  constructor(private http: HttpClient) { }

  //one-dimensional six-point spline function (t varies from 0 to 1)
  private splinify(sixPoint: number[], t: number) {
    return sixPoint[0] * (-(t ** 5) + 5 * t ** 4 - 10 * t ** 3 + 10 * t ** 2 - 5 * t + 1)
      + sixPoint[1] * (5 * t ** 5 - 20 * t ** 4 + 30 * t ** 3 - 20 * t ** 2 + 5 * t)
      + sixPoint[2] * (-10 * t ** 5 + 30 * t ** 4 - 30 * t ** 3 + 10 * t ** 2)
      + sixPoint[3] * (10 * t ** 5 - 20 * t ** 4 + 10 * t ** 3)
      + sixPoint[4] * (-5 * t ** 5 + 5 * t ** 4)
      + sixPoint[5] * t ** 5
  }

  //magical rotation-getting functions
  private getRotationAtT(t: number): number {
    if (t == 0) {
      return this.rotbreakpoints[0][1]
    }
    let lastrot = 0
    let lastlen = 0
    for (let [len, rot] of this.rotbreakpoints) {
      if (len >= t) {
        return this.getRotationTBounded(lastrot, rot, lastlen, len, t)
      }
      lastrot = rot
      lastlen = len
    }

    throw Error("Rotation error: invalid t passed in")
  }

  private getRotationTBounded(r0: number, r1: number, t0: number, t1: number, t: number) {
    return this.getRotation(r0, r1, (t - t0) / (t1 - t0))
  }

  private getRotation(r0: number, r1: number, t: number): number {
    const A = 1120 * (r1 - r0)
    return A * (t ** 7 / 42 - t ** 6 / 12 + 9 * t ** 5 / 80 - 7 * t ** 4 / 96 + t ** 3 / 48) + r0
  }

  private getPosAtT(t: number): Point {
    let i = Math.round(t * splineSubdivisions * this.splines.length)
    let splineInd = Math.floor(i / splineSubdivisions)
    if ((t * this.splines.length) % 1 == 0 && t > 0) {
      return this.splines[splineInd - 1].control_points[5]
    }
    if (splineInd >= this.splines.length) {
      return this.splines[this.splines.length - 1].control_points[5]
    }
    return [this.splinify(this.splines[splineInd].control_points.map(e => e[0]), (t * this.splines.length) % 1),
    this.splinify(this.splines[splineInd].control_points.map(e => e[1]), (t * this.splines.length) % 1)]
  }

  private getRotationPoint(t: number, theta?: number): Point {
    let p = this.getPosAtT(t)
    if (theta === undefined) {
      theta = this.getRotationAtT(t)
    }
    return this.getRotationPointAtPos(p, theta)
  }

  private getRotationPointAtPos(p: Point, theta: number): Point {
    return [p[0] + this.field_to_pixels(this.robot_size[0]) * Math.cos(theta) / 2, p[1] + this.field_to_pixels(this.robot_size[0]) * Math.sin(theta) / 2]
  }

  private drawRobot(t: number, theta?: number) {
    let p = this.getPosAtT(t)
    if (theta === undefined) {
      theta = this.getRotationAtT(t)
    }

    this.drawRobotAtPos(p, theta)

  }

  private drawRobotAtPos(p: Point, theta: number): [Point, Point, Point, Point] {
    const rxoffs = this.field_to_pixels(this.robot_size[0]) / 2
    const ryoffs = this.field_to_pixels(this.robot_size[1]) / 2
    const bumper_radius = this.field_to_pixels(3.25 * 0.0254)
    const bxoffs = rxoffs + bumper_radius
    const byoffs = ryoffs + bumper_radius

    let rpoints: [Point, Point, Point, Point] = [[rxoffs, ryoffs], [-rxoffs, ryoffs], [-rxoffs, -ryoffs], [rxoffs, -ryoffs]]
    let bpoints = [[rxoffs, byoffs], [-rxoffs, byoffs], [-bxoffs, ryoffs], [-bxoffs, -ryoffs], [-rxoffs, -byoffs], [rxoffs, -byoffs], [bxoffs, -ryoffs], [bxoffs, ryoffs]]

    for (let point of rpoints) {
      let newPoint = [0, 0]
      newPoint[0] = point[0] * Math.cos(theta) - point[1] * Math.sin(theta)
      newPoint[1] = point[0] * Math.sin(theta) + point[1] * Math.cos(theta)
      point[0] = newPoint[0]
      point[1] = newPoint[1]
    }

    for (let point of bpoints) {
      let newPoint = [0, 0]
      newPoint[0] = point[0] * Math.cos(theta) - point[1] * Math.sin(theta)
      newPoint[1] = point[0] * Math.sin(theta) + point[1] * Math.cos(theta)
      point[0] = newPoint[0]
      point[1] = newPoint[1]
    }

    rpoints = rpoints.map(e => [e[0] + p[0], e[1] + p[1]]) as [Point, Point, Point, Point]
    bpoints = bpoints.map(e => [e[0] + p[0], e[1] + p[1]]) as [Point, Point, Point, Point]

    this.ctx.beginPath()
    this.ctx.moveTo(rpoints[0][0], rpoints[0][1])
    this.ctx.lineTo(rpoints[1][0], rpoints[1][1])
    this.ctx.lineTo(rpoints[2][0], rpoints[2][1])
    this.ctx.lineTo(rpoints[3][0], rpoints[3][1])
    this.ctx.lineTo(rpoints[0][0], rpoints[0][1])
    this.ctx.lineTo(rpoints[1][0], rpoints[1][1])
    this.ctx.stroke()

    this.ctx.beginPath();
    this.ctx.moveTo(bpoints[0][0], bpoints[0][1]);
    for(let i = 1; i < 8; i++) {
      this.ctx.lineTo(bpoints[i][0], bpoints[i][1])
    }
    this.ctx.lineTo(bpoints[0][0], bpoints[0][1])
    this.ctx.lineTo(bpoints[1][0], bpoints[1][1])
    this.ctx.stroke()

    // TODO: do this in a more principled way that lends itself better to future years
    let pivot_lines: [Point, Point] = [[this.field_to_pixels(20 * 0.0254), -this.field_to_pixels(2.5 * 0.0254)], [this.field_to_pixels(-20 * 0.0254), -this.field_to_pixels(2.5 * 0.0254)]]
    pivot_lines[0] = [pivot_lines[0][0] * Math.cos(theta) - pivot_lines[0][1] * Math.sin(theta),
      pivot_lines[0][0] * Math.sin(theta) + pivot_lines[0][1] * Math.cos(theta)]
    pivot_lines[1] = [pivot_lines[1][0] * Math.cos(theta) - pivot_lines[1][1] * Math.sin(theta),
      pivot_lines[1][0] * Math.sin(theta) + pivot_lines[1][1] * Math.cos(theta)]
    pivot_lines[0] = [pivot_lines[0][0] + p[0], pivot_lines[0][1] + p[1]]
    pivot_lines[1] = [pivot_lines[1][0] + p[0], pivot_lines[1][1] + p[1]]

    this.ctx.beginPath()
    this.ctx.moveTo(pivot_lines[0][0], pivot_lines[0][1])
    this.ctx.lineTo(pivot_lines[1][0], pivot_lines[1][1])
    this.ctx.stroke();

    return rpoints
  }

  private pointToField(pt: Point): Point {
    let p = this.ctx.getTransform().inverse().transformPoint({ x: pt[0], y: pt[1] })
    return [p.x, p.y]
  }

  public updateSplineCurve(spline: Spline) {
    spline.curve_points = []
    for (let i = 0; i <= splineSubdivisions; i += 1) {
      let t = i / splineSubdivisions
      let x = this.splinify(spline.control_points.map(e => e[0]), t)
      let y = this.splinify(spline.control_points.map(e => e[1]), t)
      spline.curve_points.push([x, y])
    }
  }
  private drawPoint(point: Point, stoppoint?: boolean, selected?: boolean) {
    let filled = true
    if (this.splines.length > 0 && point === this.splines[0].control_points[0]) {
      this.ctx.fillStyle = "green"
    } else if (this.splines.length > 0 && point === this.splines[this.splines.length - 1].control_points[5]) {
      this.ctx.fillStyle = "red"
    } else if (stoppoint) {
      this.ctx.fillStyle = "#FFC000"
    } else {
      this.ctx.lineWidth = 2
      filled = false
    }
    this.ctx.beginPath()
    this.ctx.arc(point[0], point[1], 5, 0, Math.PI * 2)
    if (filled) {
      this.ctx.fill()
    } else {
      this.ctx.stroke()
    }
    if (selected) {
      if (filled) {
        this.ctx.lineWidth = 1
        this.ctx.strokeStyle = "#00FFFF"
        this.ctx.stroke()
      } else {
        this.ctx.fillStyle = "#00FFFF"
        this.ctx.fill()
      }
    }
  }

  public updateCanvas() {
    this.ctx.save()
    this.ctx.fillStyle = "#F0DCF0"
    this.ctx.setTransform(1, 0, 0, 1, 0, 0)
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height)
    this.ctx.restore()
    if (this.field.imageElement) {
      let w = this.canvas.width;
      let h = this.field.imageElement.height * this.canvas.width / this.field.imageElement.width;
      this.ctx.drawImage(this.field.imageElement, -w / 2, -h / 2, w, h)
    }
    let splineColorIndex = -1
    for (let spline of this.splines) {
      splineColorIndex = (splineColorIndex + 1) % splineColors.length
      this.ctx.beginPath()
      this.ctx.moveTo(spline.curve_points[0][0], spline.curve_points[0][1])
      for (let point of spline.curve_points) {
        this.ctx.lineTo(point[0], point[1])
      }
      if (spline === this.grabbedSpline || spline === this.movableGrabbedSpline) {
        this.ctx.lineWidth = 2
        this.ctx.strokeStyle = "#00FFFF"
        this.ctx.stroke()
      }
      this.ctx.lineWidth = 1
      if (this.mode == "view") {
        this.ctx.strokeStyle = splineColors[splineColorIndex]
      } else {
        this.ctx.strokeStyle = "black"
      }
      this.ctx.stroke()


      //draw lines between control points
      this.ctx.lineWidth = 0.5
      this.ctx.beginPath()
      this.ctx.moveTo(spline.control_points[0][0], spline.control_points[0][1])
      this.ctx.lineTo(spline.control_points[1][0], spline.control_points[1][1])
      this.ctx.lineTo(spline.control_points[2][0], spline.control_points[2][1])
      this.ctx.stroke()
      this.ctx.beginPath()
      this.ctx.moveTo(spline.control_points[3][0], spline.control_points[3][1])
      this.ctx.lineTo(spline.control_points[4][0], spline.control_points[4][1])
      this.ctx.lineTo(spline.control_points[5][0], spline.control_points[5][1])
      this.ctx.stroke()

      for (let point of spline.control_points) {
        this.ctx.strokeStyle = splineColors[splineColorIndex]
        this.drawPoint(point, point == spline.control_points[0] && spline.prev_spline_link == "G0"
          || point == spline.control_points[5] && spline.next_spline_link == "G0",
          this.selectedStopPoint != null && ((point == spline.control_points[0] && this.splines[this.selectedStopPoint.next_spline_index] == spline)
            || (point == spline.control_points[5] && this.splines[this.selectedStopPoint.next_spline_index - 1] == spline)))
      }
    }
    //draw section constraints
    for (let constraint of this.constraints) {
      let startSpline = Math.floor(this.splines.length * constraint.selection[0])
      let endSpline = Math.floor(this.splines.length * constraint.selection[1])
      let startIndex = Math.ceil((this.splines.length * constraint.selection[0] - startSpline) * splineSubdivisions)
      let endIndex = Math.round((this.splines.length * constraint.selection[1] - endSpline) * splineSubdivisions)

      let sIndex = startSpline
      let pIndex = startIndex
      this.ctx.beginPath()
      this.ctx.strokeStyle = "#BBBBBB"
      if (this.mode == "edit section constraints") {
        this.ctx.strokeStyle = "#005050"
      }
      if (constraint == this.selectedConstraint) {
        this.ctx.strokeStyle = "#00FFFF"
      }
      this.ctx.lineWidth = 2.5
      let p = this.splines[sIndex].curve_points[pIndex]
      this.ctx.moveTo(p[0], p[1])

      pIndex += 1
      if (pIndex == splineSubdivisions) {
        pIndex = 0
        sIndex += 1
      }
      while (sIndex < endSpline || pIndex < endIndex) {
        let p = this.splines[sIndex].curve_points[pIndex]
        this.ctx.lineTo(p[0], p[1])

        pIndex += 1
        if (pIndex == splineSubdivisions) {
          pIndex = 0
          sIndex += 1
        }
      }
      this.ctx.stroke()

      this.ctx.fillStyle = this.ctx.strokeStyle
      this.ctx.beginPath()
      this.ctx.arc(p[0], p[1], 3, 0, Math.PI * 2)
      this.ctx.fill()
      pIndex -= 1
      if (pIndex == -1) {
        pIndex = splineSubdivisions - 1
        sIndex -= 1
      }
      p = this.splines[sIndex].curve_points[pIndex]
      this.ctx.beginPath()
      this.ctx.arc(p[0], p[1], 3, 0, Math.PI * 2)
      this.ctx.fill()
    }

    this.ctx.strokeStyle = "black"
    for (let point of this.addedPoints) {
      this.drawPoint(point)
    }

    if (this.splines.length != 0) {
      for (let rotpoint of this.rotbreakpoints) {
        if (this.mode == "edit rotation breakpoints") {
          this.ctx.strokeStyle = "black"
          this.ctx.fillStyle = "#DD00CC"
        } else {
          this.ctx.strokeStyle = "#BBBBBB80"
          this.ctx.fillStyle = "#BBBBBB80"
        }
        this.ctx.lineWidth = 2
        let [len, rot] = rotpoint
        this.drawRobot(len, rot)
        let front = this.getRotationPoint(len, rot)
        this.ctx.beginPath()
        this.ctx.arc(front[0], front[1], 3, 0, Math.PI * 2)
        this.ctx.fill()
        if (this.mode == "edit rotation breakpoints" && this.staticrotbreakpoints.indexOf(rotpoint[0]) === -1) {
          let p = this.getPosAtT(len)
          this.ctx.fillStyle = "black"
          this.ctx.beginPath()
          this.ctx.arc(p[0], p[1], 3, 0, Math.PI * 2)
          this.ctx.fill()
        }
      }
    }


    if (this.optidata != null) {
      this.ctx.strokeStyle = "#9955FF"
      this.ctx.lineWidth = 2


      let [x, y, r] = this.getRobotInfoAtTime(this.playBarT * this.totalTime, "positions")
      let rpoints = this.drawRobotAtPos([x, y], r)
      if (this.showForces) {
        let forces = this.getRobotInfoAtTime(this.playBarT * this.totalTime, "module_forces")
        for (let i = 0; i < 4; i++) {
          this.ctx.beginPath()
          this.ctx.moveTo(rpoints[i][0], rpoints[i][1])
          let offs = [
            forces[i][0] * Math.cos(r) - forces[i][1] * Math.sin(r),
            -forces[i][0] * Math.sin(r) - forces[i][1] * Math.cos(r)
          ]
          this.ctx.lineTo(rpoints[i][0] + offs[0] / 4, rpoints[i][1] + offs[1] / 4)
          this.ctx.stroke()
        }
      }
      if (this.showVelocities) {
        this.ctx.strokeStyle = "orange"
        let mod_vels = this.getRobotInfoAtTime(this.playBarT * this.totalTime, "mod_vels")
        for (let i = 0; i < 4; i++) {
          this.ctx.beginPath()
          this.ctx.moveTo(rpoints[i][0], rpoints[i][1])
          this.ctx.lineTo(rpoints[i][0] + mod_vels[i][0] * 5, rpoints[i][1] - mod_vels[i][1] * 5)
          this.ctx.stroke()
        }
      }

      let front = this.getRotationPointAtPos([x, y], r)
      this.ctx.fillStyle = "#9955FF"
      this.ctx.beginPath()
      this.ctx.arc(front[0], front[1], 3, 0, Math.PI * 2)
      this.ctx.fill()
      this.ctx.beginPath()
      this.ctx.arc(x, y, 3, 0, Math.PI * 2)
      this.ctx.fill()
    }

    //draw actions
    for (let action of this.actions) {
      //draw a point
      let point = this.getPosAtT(action.location)
      this.ctx.fillStyle = "#00A040"
      this.ctx.beginPath()
      this.ctx.arc(point[0], point[1], 5, 0, Math.PI * 2)
      this.ctx.fill()

      //draw the label
      this.ctx.textAlign = "center"
      this.ctx.textBaseline = "top"
      this.ctx.fillStyle = "#008020"
      this.ctx.fillText(action.name, point[0], point[1] + 3);
    }

    if (this.closeCurvePoint !== null && this.grabbedRotPointRot === null && this.grabbedRotPointLen === null && this.splines.length > 0) {
      this.ctx.strokeStyle = "black"
      this.ctx.lineWidth = 2

      this.drawRobot(this.closeCurveT)
      this.ctx.fillStyle = "black"
      this.ctx.beginPath()
      this.ctx.arc(this.closeCurvePoint[0], this.closeCurvePoint[1], 3, 0, Math.PI * 2)
      this.ctx.fill()
      let front = this.getRotationPoint(this.closeCurveT)
      this.ctx.fillStyle = "#DD9900"
      this.ctx.beginPath()
      this.ctx.arc(front[0], front[1], 3, 0, Math.PI * 2)
      this.ctx.fill()
    }
  }

  private switchModes(mode: Mode) {
    switch (mode) {
      case "add G0 spline":
        this.pointsToAdd = 5
        break
      case "add G1 spline":
        this.pointsToAdd = 4
        break
      default:
        this.pointsToAdd = 0
        break
    }
    this.selectedConstraint = null
    this.selectedContext = new GlobalContext(this.globalConstraints, this)
    this.addedPoints = []
    this.mode = mode
    this.updateCanvas()
  }

  public movePoint(spline: Spline, point: Point, destination: Point) {
    let dispX = destination[0] - point[0]
    let dispY = destination[1] - point[1]
    if (point === spline.control_points[0]) {
      spline.control_points[0][0] += dispX
      spline.control_points[1][0] += dispX
      spline.control_points[2][0] += dispX
      spline.control_points[0][1] += dispY
      spline.control_points[1][1] += dispY
      spline.control_points[2][1] += dispY
      if (spline.prev_spline) {
        spline.prev_spline.control_points[4][0] += dispX
        spline.prev_spline.control_points[3][0] += dispX
        spline.prev_spline.control_points[4][1] += dispY
        spline.prev_spline.control_points[3][1] += dispY
        this.updateSplineCurve(spline.prev_spline)
      }
    } else if (point === spline.control_points[1]) {
      spline.control_points[1][0] += dispX
      spline.control_points[2][0] += dispX
      spline.control_points[1][1] += dispY
      spline.control_points[2][1] += dispY
      if (spline.prev_spline && spline.prev_spline_link == "G1") {
        spline.prev_spline.control_points[4][0] -= dispX
        spline.prev_spline.control_points[3][0] -= dispX
        spline.prev_spline.control_points[4][1] -= dispY
        spline.prev_spline.control_points[3][1] -= dispY
        this.updateSplineCurve(spline.prev_spline)
      }
    } else if (point === spline.control_points[2]) {
      spline.control_points[2][0] += dispX
      spline.control_points[2][1] += dispY
    } else if (point === spline.control_points[3]) {
      spline.control_points[3][0] += dispX
      spline.control_points[3][1] += dispY
    } else if (point === spline.control_points[4]) {
      spline.control_points[4][0] += dispX
      spline.control_points[3][0] += dispX
      spline.control_points[4][1] += dispY
      spline.control_points[3][1] += dispY
      if (spline.next_spline && spline.next_spline_link == "G1") {
        spline.next_spline.control_points[1][0] -= dispX
        spline.next_spline.control_points[2][0] -= dispX
        spline.next_spline.control_points[1][1] -= dispY
        spline.next_spline.control_points[2][1] -= dispY
        this.updateSplineCurve(spline.next_spline)
      }
    } else if (point === spline.control_points[5]) {
      spline.control_points[5][0] += dispX
      spline.control_points[4][0] += dispX
      spline.control_points[3][0] += dispX
      spline.control_points[5][1] += dispY
      spline.control_points[4][1] += dispY
      spline.control_points[3][1] += dispY
      if (spline.next_spline) {
        spline.next_spline.control_points[1][0] += dispX
        spline.next_spline.control_points[2][0] += dispX
        spline.next_spline.control_points[1][1] += dispY
        spline.next_spline.control_points[2][1] += dispY
        this.updateSplineCurve(spline.next_spline)
      }
    }
    this.updateSplineCurve(spline)
  }

  ngAfterViewInit() {
    this.canvas = this.canvasref.nativeElement
    this.ctx = this.canvas.getContext("2d")
    this.playbarcircle = this.playbarcircleref.nativeElement
    this.playbar = this.playbarref.nativeElement
    this.displayForces = this.displayForcesRef.nativeElement
    this.displayVelocities = this.displayVelocitiesRef.nativeElement

    this.http.get('/fields.json').subscribe((data: Field[]) => {
      this.fields = data;

      for (let field of this.fields) {
        field.imageElement = new Image()
        field.imageElement.src = field.imagePath
      }

      if (this.fields.length > 0) {
        this.fields[0].imageElement.addEventListener("load", () => {
          this.field = this.fields[0]
          let w = this.canvas.width
          let h = this.field.imageElement.height * this.canvas.width / this.field.imageElement.width
          this.ctx.translate(w/2, h/2)
          this.updateCanvas()
        })
      }
    });

    this.canvas.addEventListener("mouseleave", (ev) => {
      this.closeCurvePoint = null
      this.closeCurveT = null
      this.updateCanvas()
    })

    this.canvas.addEventListener("mouseup", (ev) => {
      if ((this.mode == "add G1 spline" || this.mode == "add G0 spline") && !this.panning) {
        this.addedPoints.push(this.pointToField([ev.offsetX, ev.offsetY]))
        this.undonePointsStack = []
        this.pointsToAdd--
        if (this.pointsToAdd === 0) {
          let spline: Spline
          if (this.splines.length === 0) {
            spline = { control_points: this.addedPoints, curve_points: [] } as Spline
            this.stopPoints.push({ delay: 0, next_spline_index: 0 })
          } else {
            let prevSpline = this.splines[this.splines.length - 1]
            if (this.mode == "add G0 spline") {
              spline = { control_points: [prevSpline.control_points[5], ...this.addedPoints], curve_points: [] } as Spline
              prevSpline.next_spline = spline
              prevSpline.next_spline_link = "G0"
              spline.prev_spline = prevSpline
              spline.prev_spline_link = "G0"
            } else if (this.mode == "add G1 spline") {
              let firstFourPoints: Point[] = []
              firstFourPoints.push(prevSpline.control_points[5])
              firstFourPoints.push([prevSpline.control_points[5][0] * 2 - prevSpline.control_points[4][0],
              prevSpline.control_points[5][1] * 2 - prevSpline.control_points[4][1]])
              spline = { control_points: [...firstFourPoints, ...this.addedPoints], curve_points: [] } as Spline
              prevSpline.next_spline = spline
              prevSpline.next_spline_link = "G1"
              spline.prev_spline = prevSpline
              spline.prev_spline_link = "G1"
            }
          }
          this.updateSplineCurve(spline)
          this.splines.push(spline)
          for (let pair of this.rotbreakpoints) {
            if (pair[0] != 1) {
              pair[0] = pair[0] * (this.splines.length - 1) / this.splines.length
            }
          }
          for (let c of this.constraints) {
            c.selection[0] *= (this.splines.length - 1) / (this.splines.length)
            c.selection[1] *= (this.splines.length - 1) / (this.splines.length)
          }
          for (let a of this.actions) {
            a.location *= (this.splines.length - 1) / this.splines.length
          }
          if (this.mode == "add G0 spline") {
            this.addrotbreakpoint((this.splines.length - 1) / this.splines.length, false, this.rotbreakpoints[this.rotbreakpoints.length - 1][1])
            this.staticrotbreakpoints.push(this.rotbreakpoints[this.rotbreakpoints.length - 2][0])
            this.stopPoints.push({ delay: 0, next_spline_index: this.splines.length - 1 })
          }
          this.pathOutOfDate = true
          this.finalizeChange()
          this.switchModes("view")
        }
        this.updateCanvas()
      } else if (this.mode == "edit rotation breakpoints" && !this.panning) {
        if (this.closeCurveT !== null && this.grabbedRotPointLen === null && this.grabbedRotPointRot === null) {
          //rotbreakpoints MUST BE SORTED
          this.addrotbreakpoint(this.closeCurveT, true)
        }
      } else if (this.mode == "edit section constraints" && !this.panning) {
        //they call me the section the way I constraint.
        if (this.grabbedConstraint == null) {
          //check for nearby section constraints
          let selecting = false
          for (let constraint of this.constraints) {
            if (this.closeCurveT <= constraint.selection[1] && this.closeCurveT >= constraint.selection[0]) {
              selecting = true
              this.selectedConstraint = constraint
              this.selectedContext = new ConstraintContext(constraint, this)
            }
          }
          if (!selecting) {
            //add a new point if not selecting an existing one
            if (this.closeCurveT != null) {
              if (this.firstAddedConstraintNode == null) {
                this.firstAddedConstraintNode = this.closeCurveT
              } else {
                let newConstraint: Constraint = {
                  selection: this.firstAddedConstraintNode < this.closeCurveT ? [this.firstAddedConstraintNode, this.closeCurveT] : [this.closeCurveT, this.firstAddedConstraintNode],
                  max_velocity: this.globalConstraints.max_velocity,
                  max_acceleration: this.globalConstraints.max_acceleration,
                  max_current: this.globalConstraints.max_current,
                  max_voltage: this.globalConstraints.max_voltage,
                }
                this.constraints.push(newConstraint)
                this.selectedConstraint = newConstraint

                this.selectedContext = new ConstraintContext(newConstraint, this)
                this.firstAddedConstraintNode = null
              }
            } else {
              this.selectedContext = new GlobalContext(this.globalConstraints, this)
              this.selectedConstraint = null
              this.grabbedConstraint = null
            }
          }
        }
        this.grabbedConstraint = null
      } else if (this.mode == "edit actions" && !this.panning) {
        if (this.grabbedAction != null) {
          this.selectedContext = new ActionContext(this.grabbedAction, this)
        } else if (this.closeCurveT != null) {
          this.actions.push({
            location: this.closeCurveT,
            name: "unnamed action"
          })
        }
      }
      this.grabbedAction = null;
      this.selectedStopPoint = null
      if (this.grabbedPoint != null) {
        this.pathOutOfDate = true
        this.finalizeChange()
        //Selection time if possible
        let index = this.splines.length
        for (let i = 0; i < this.splines.length; i++) {
          if (this.splines[i].control_points[5] == this.grabbedPoint) {
            index = i + 1
            break
          } else if (this.splines[i].control_points[0] == this.grabbedPoint) {
            index = i
          }
        }
        if (index < this.splines.length) {
          if (this.splines[index].prev_spline_link != "G1") {
            this.selectedStopPoint = this.stopPoints.filter(e => e.next_spline_index == index)[0]
            this.selectedContext = new StopPointContext(this.selectedStopPoint, this)
          }
        }

        this.changeInProgress = null
      }
      this.grabbedConstraint = null
      this.grabbedPoint = null
      this.grabbedSpline = null
      this.movableGrabbedSpline = null
      //resort to violence (sorting)
      if (this.grabbedRotPointLen !== null) {
        this.rotbreakpoints.sort((p1, p2) => p1[0] - p2[0])
        this.fixrotation()
      }
      if (this.grabbedRotPointLen !== null || this.grabbedRotPointRot !== null) {
        this.pathOutOfDate = true
        this.finalizeChange()
      }
      this.grabbedRotPointLen = null
      this.grabbedRotPointRot = null
      this.panning = false
      if (this.selectedContext instanceof StopPointContext && this.selectedStopPoint == null) {
        this.selectedContext = new GlobalContext(this.globalConstraints, this)
      }
      this.updateCanvas()
    })

    this.canvas.addEventListener("mousemove", (ev) => {
      if (this.draggingCircle) {
        return
      }
      let prevCloseCurveT = this.closeCurveT
      this.closeCurvePoint = null
      this.closeCurveT = null
      let truePoint = this.pointToField([ev.offsetX, ev.offsetY])
      //robot highlighting
      let minDistSq = (this.grabbedRotPointLen != null || this.grabbedRotPointRot != null) ? Infinity : (100 / this.ctx.getTransform().a) ** 2
      for (let ind = 0; ind < this.splines.length; ind++) {
        let spline = this.splines[ind]
        for (let i = 0; i < spline.curve_points.length; i++) {
          let point = spline.curve_points[i]
          if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= minDistSq) {
            this.closeCurvePoint = point
            this.closeCurveT = (i + splineSubdivisions * ind) / (this.splines.length * splineSubdivisions)
            minDistSq = (point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2
          }
        }
      }
      if (this.optidata != null) {
        this.hoverT = this.closeCurveT ? this.getTimeFromT(this.closeCurveT) / this.totalTime : null
      }

      if (ev.buttons === 1) {
        //handle dragging grabbed things
        if (this.grabbedPoint !== null) {
          this.movePoint(this.grabbedSpline, this.grabbedPoint, truePoint)
        } else if (this.grabbedRotPointLen !== null) {
          this.rotbreakpoints[this.grabbedRotPointLen][0] = this.closeCurveT
        } else if (this.grabbedRotPointRot !== null) {
          let p = this.getPosAtT(this.rotbreakpoints[this.grabbedRotPointRot][0])
          let ang = Math.atan2(truePoint[1] - p[1], truePoint[0] - p[0])
          this.rotbreakpoints[this.grabbedRotPointRot][1] = ang
          this.fixrotation()
        } else if (this.grabbedAction !== null) {
          this.grabbedAction.location = this.closeCurveT;
        } else if (this.grabbedConstraint !== null) {
          this.grabbedConstraint.selection[this.grabbedConstraintIndex] = this.closeCurveT
        } else {
          this.ctx.translate(ev.movementX / this.ctx.getTransform().a, ev.movementY / this.ctx.getTransform().d)
          this.panning = true
        }
        this.updateCanvas()
      } else if (this.closeCurveT != prevCloseCurveT) {
        this.updateCanvas()
      }
    })

    this.canvas.addEventListener("mousedown", (ev) => {
      if (this.mode == "view") {
        let truePoint = this.pointToField([ev.offsetX, ev.offsetY])
        for (let spline of this.splines) {
          for (let point of spline.control_points) {
            if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
              this.changeInProgress = new ChangeEvent(this)
              this.grabbedPoint = point
              this.grabbedSpline = spline
              if (point == spline.control_points[0] || point == spline.control_points[1]) {
                this.movableGrabbedSpline = spline.prev_spline_link == "G1" ? spline.prev_spline : null
              } else if (point == spline.control_points[4] || point == spline.control_points[5]) {
                this.movableGrabbedSpline = spline.next_spline_link == "G1" ? spline.next_spline : null
              }
              return
            }
          }
        }
      } else if (this.mode == "edit rotation breakpoints") {
        let truePoint = this.pointToField([ev.offsetX, ev.offsetY])
        for (let i = 0; i < this.rotbreakpoints.length; i++) {
          let [len, rot] = this.rotbreakpoints[i]
          if (this.staticrotbreakpoints.indexOf(this.rotbreakpoints[i][0]) === -1) {
            let point = this.getPosAtT(len)
            if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
              this.grabbedRotPointLen = i
              this.grabbedRotPointRot = null
              this.changeInProgress = new ChangeEvent(this)
              return
            }
          }
          let rpoint = this.getRotationPoint(len, rot)
          if ((rpoint[0] - truePoint[0]) ** 2 + (rpoint[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
            this.grabbedRotPointLen = null
            this.grabbedRotPointRot = i
            this.changeInProgress = new ChangeEvent(this)
            return
          }
        }
      } else if (this.mode == "edit section constraints") {
        let truePoint = this.pointToField([ev.offsetX, ev.offsetY])
        for (let i = 0; i < this.constraints.length; i++) {
          let point = this.getPosAtT(this.constraints[i].selection[0])
          if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
            this.grabbedConstraint = this.constraints[i]
            this.grabbedConstraintIndex = 0
          } else {
            point = this.getPosAtT(this.constraints[i].selection[1])
            if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
              this.grabbedConstraint = this.constraints[i]
              this.grabbedConstraintIndex = 1
            }
          }
        }
      } else if (this.mode == "edit actions") {
        let truePoint = this.pointToField([ev.offsetX, ev.offsetY])
        for (let action of this.actions) {
          let point = this.getPosAtT(action.location)
          if ((point[0] - truePoint[0]) ** 2 + (point[1] - truePoint[1]) ** 2 <= (5 + 5 / this.ctx.getTransform().a) ** 2) {
            this.grabbedAction = action
          }
        }
      }
    })

    let handleScroll = (ev: any) => {
      var delta = ev.wheelDelta ? ev.wheelDelta / 40 : ev.detail ? -ev.detail : 0
      if (delta) {
        let pt = this.ctx.getTransform().inverse().transformPoint({ x: ev.offsetX, y: ev.offsetY })
        let newTransform = DOMMatrix.fromMatrix(this.ctx.getTransform())
        newTransform.translateSelf(pt.x, pt.y)
        let factor = 1.05 ** delta
        newTransform.scaleSelf(factor, factor)
        if (newTransform.a < 0.25) {
          newTransform.a = 0.25
          newTransform.d = 0.25
        } else if (newTransform.a > 10) {
          newTransform.a = 10
          newTransform.d = 10
        }
        newTransform.translateSelf(-pt.x, -pt.y)
        this.ctx.setTransform(newTransform)
        this.updateCanvas()
      }
      return ev.preventDefault() && false
    }
    this.canvas.addEventListener('DOMMouseScroll', handleScroll, false)
    this.canvas.addEventListener('mousewheel', handleScroll, false)

    document.addEventListener("mousemove", (ev) => {
      if (ev.buttons == 1 && this.draggingCircle && this.optidata != null) {
        let barRect = this.playbar.getBoundingClientRect()
        let x = ev.clientX
        x -= barRect.x + 10
        x /= barRect.width - 20
        x = x < 0 ? 0 : (x > 1 ? 1 : x)
        this.playBarT = x
        this.updatePlayCircle()
        this.updateCanvas()
        ev.stopImmediatePropagation()
      }
    })
    document.addEventListener("mouseup", (ev) => {
      this.draggingCircle = false
    })

    document.addEventListener("keydown", (ev) => {
      if (this.keybindsenabled) {
        switch (ev.key) {
          case " ": {
            this.togglePlay()
            break
          }
          case "a": {
            this.addG1Spline()
            break
          }
          case "s": {
            this.addG0Spline()
            break
          }
          case "r": {
            this.editRotPoints()
            break
          }
          case "e": {
            this.sendPath()
            break
          }
          case "d": {
            this.nextPlaySpeed()
            break
          }
          case "z": {
            if (ev.getModifierState("Control")) {
              //mac users: no command-z for you (Accel key deprecated for some reason)
              this.undo()
            }
            break
          }
          case "y": {
            if (ev.getModifierState("Control")) {
              this.redo()
            }
            break
          }
        }
      }
    })

    let perFrame = () => {
      let t = Date.now()
      let dt = (t - this.lastCalledTime) / 1000
      this.lastCalledTime = t
      if (this.playing && this.optidata != null) {
        if (this.playBarT >= 1) {
          this.playBarT = 0
        } else {
          this.playBarT += dt / this.totalTime * this.playspeed
          if (this.playBarT > 1) {
            this.playBarT = 1
          }
        }
        this.updatePlayCircle()
        this.updateCanvas()
      }
      setTimeout(perFrame, 1000 / 60 - (Date.now() - t))
    }
    setTimeout(perFrame, 1)
  }

  public getTimeFromT(t: number): number {
    if (t <= 0) {
      return 0
    }
    if (t >= 1) {
      return this.totalTime
    }

    let dataIndex = t * (this.optidata.times.length - 1)
    let s = Math.floor(dataIndex)
    let e = Math.ceil(dataIndex)
    if (s == e) {
      return this.optidata.times[s]
    }
    let sfac = e - dataIndex
    let efac = dataIndex - s
    return sfac * this.optidata.times[s] + efac * this.optidata.times[e]
  }

  public downloadPath() {
    this.downloadAsJson(this.getPathJson(), "swerve-path")
  }

  private downloadAsJson(exportObj: any, exportName: string) {
    let dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(exportObj))
    let downloadAnchorNode = document.createElement('a')
    downloadAnchorNode.setAttribute("href", dataStr)
    downloadAnchorNode.setAttribute("download", exportName + ".json")
    document.body.appendChild(downloadAnchorNode); // required for firefox
    downloadAnchorNode.click()
    downloadAnchorNode.remove()
  }

  public openFileDialog() {
    let fileAnchorNode: HTMLInputElement = document.createElement('input')
    fileAnchorNode.type = "file"
    document.body.appendChild(fileAnchorNode); // required for firefox
    fileAnchorNode.click()
    fileAnchorNode.addEventListener("change", (ev) => {
      this.loadFile(fileAnchorNode.files[0])
    })
    fileAnchorNode.remove()
  }

  public async loadFile(file: File) {
    if (file.type != "application/json") {
      alert("Please upload a JSON file.")
    } else {
      let data = await file.arrayBuffer()
      let dec = new TextDecoder("utf-8")
      let filedata = JSON.parse(dec.decode(new Uint8Array(data)))
      let paths = filedata.paths
      let splineCount = 0
      this.splines = []
      this.constraints = []
      this.rotbreakpoints = []
      this.staticrotbreakpoints = []
      for (let path of paths) {
        splineCount += path.splines.length
      }
      let currSplineIndex = 0
      let lastSpline: Spline = null
      for (let path of paths) {
        //add splines
        let prevSpline: Spline = null
        for (let spline of path.splines) {
          let s: Spline = {
            //negate y to fix coordinate system issues
            control_points: spline.map(e => [this.field_to_pixels(e[0]), -this.field_to_pixels(e[1])]),
            curve_points: [],
            prev_spline_link: prevSpline ? "G1" : lastSpline ? "G0" : null,
            prev_spline: prevSpline ? prevSpline : lastSpline
          }
          if (prevSpline) {
            prevSpline.next_spline = s
            prevSpline.next_spline_link = "G1"
          } else if (lastSpline) {
            lastSpline.next_spline = s
            lastSpline.next_spline_link = "G0"
            //add stoppoint
            this.stopPoints.push({
              next_spline_index: this.splines.length,
              delay: path.startDelay
            })
          } else {
            this.stopPoints.push({
              next_spline_index: this.splines.length,
              delay: path.startDelay
            })
          }
          this.updateSplineCurve(s)
          this.splines.push(s)
          prevSpline = s
          lastSpline = s
        }
        //add rotbreakpoints
        let newrbps = path.rot_breakpoints
        //negate theta to fix coordinate system issues
        newrbps = newrbps.map(e => [(e[0] * path.splines.length + currSplineIndex) / splineCount, -e[1]])
        this.rotbreakpoints.push(...newrbps)
        //add to static rotbreakpoints
        this.staticrotbreakpoints.push(currSplineIndex / splineCount)
        //add constraints
        let newconstraints: Constraint[] = path.constraints
        newconstraints.forEach(e => {
          e.selection = e.selection.map(v => (v * path.splines.length + currSplineIndex) / splineCount) as [number, number]
        })
        this.constraints.push(...newconstraints)

        currSplineIndex += path.splines.length
      }

      console.log({ "things": this.stopPoints })
      this.staticrotbreakpoints.push(1)
      //handle duplicate edge rotbreakpoints
      this.rotbreakpoints = this.rotbreakpoints.filter((v, i, a) => i == 0 || v[0] != a[i - 1][0])

      //re-merge split constraints across paths
      console.log("STARTING MERGING")
      console.log("Current:")
      console.log(this.constraints)
      if (this.constraints.length > 0) {
        let newconstraints: Constraint[] = [this.constraints[0]]
        for (let i = 1; i < this.constraints.length; i++) {
          let curr_constraint = this.constraints[i]
          let prev_constraint = this.constraints[i - 1]
          if (curr_constraint.max_acceleration == prev_constraint.max_acceleration
            && curr_constraint.max_current == prev_constraint.max_current
            && curr_constraint.max_velocity == prev_constraint.max_velocity
            && curr_constraint.max_voltage == prev_constraint.max_voltage
            && Math.abs(prev_constraint.selection[1] - curr_constraint.selection[0]) < 0.001) {
            prev_constraint.selection[1] = curr_constraint.selection[1]
            console.log("merging index " + i)
          } else {
            newconstraints.push(curr_constraint)
            console.log("adding index " + i)
          }
        }
        this.constraints = newconstraints
      }
      console.log("NEW CONSTRAINTS:")
      console.log(this.constraints)
      console.log("END MERGE")
      //re-merge spline endpoints - prevents spline decoupling
      for (let i = 1; i < this.splines.length; i++) {
        let curr_spline = this.splines[i]
        let prev_spline = this.splines[i - 1]
        curr_spline.control_points[0] = prev_spline.control_points[5]
      }

      this.globalConstraints = filedata.global_constraints
      this.field = this.fields.filter(e => e.year == filedata.metadata.field)[0]
      this.field ??= this.fields[0]

      this.actions = filedata.actions

      console.log(this.splines)
      console.log(this.rotbreakpoints)
      console.log(this.staticrotbreakpoints)
      console.log(this.globalConstraints)
      console.log(this.stopPoints)

      this.updateCanvas()
      this.selectedContext = new GlobalContext(this.globalConstraints, this)
    }
  }

  private field_to_pixels(n: number) {
    return n * this.canvas.width / this.field.field_size[0]
  }

  private pixels_to_field(n: number) {
    return n * this.field.field_size[0] / this.canvas.width
  }

  private getPathJson() {
    let paths = []
    let curPath: {
      splines: SplineControl[],
      rot_breakpoints: [number, number][],
      constraints: Constraint[],
      startDelay: number,
    } = {
      splines: [],
      rot_breakpoints: [[this.rotbreakpoints[0][0], -this.rotbreakpoints[0][1]]],
      constraints: [],
      startDelay: this.stopPoints[0].delay, /*(BREAKPOINT)*/
    }
    for (let i = 0; i < this.splines.length; i++) {
      //negate y to fix coordinate system
      curPath.splines.push(this.splines[i].control_points.map(e => [this.pixels_to_field(e[0]), -this.pixels_to_field(e[1])]) as SplineControl)
      let stlow = i / this.splines.length
      let sthigh = (i + 1) / this.splines.length
      //negate theta to fix coordinate system
      curPath.rot_breakpoints.push(...this.rotbreakpoints.filter(e => stlow < e[0] && e[0] <= sthigh).map(e => [e[0], -e[1]] as [number, number]))
      curPath.constraints.push(...this.constraints.filter(e =>
        e.selection[0] < sthigh && e.selection[1] > stlow
        && !curPath.constraints.includes(e)
      ))
      if (this.splines[i].next_spline_link != "G1") {
        //normalize the rot breakpoints and constraints
        let lpoint = curPath.rot_breakpoints[curPath.rot_breakpoints.length - 1]
        let min_rbp = curPath.rot_breakpoints[0][0]
        let max_rbp = lpoint[0]
        curPath.rot_breakpoints = curPath.rot_breakpoints.map(e => [(e[0] - min_rbp) / (max_rbp - min_rbp), e[1]])

        curPath.constraints = curPath.constraints.map(e => {
          return {
            selection: e.selection.map(e => (e - min_rbp) / (max_rbp - min_rbp)).map(e => e < 0 ? 0 : e > 1 ? 1 : e) as [number, number],
            max_voltage: e.max_voltage,
            max_current: e.max_current,
            max_acceleration: e.max_acceleration,
            max_velocity: e.max_velocity
          }
        })

        paths.push(curPath)
        if (this.splines[i].next_spline_link == "G0") {
          curPath = { splines: [], rot_breakpoints: [lpoint], constraints: [], startDelay: this.stopPoints[paths.length].delay }
        }
      }
    }
    console.log(this.rotbreakpoints)
    return { paths, global_constraints: this.globalConstraints, metadata: { field: this.field.year }, actions: this.actions }
  }

  public flip() {
    const width = this.field_to_pixels(this.field.field_size[0])
    let lastFlippedPoint = null
    for (let spline of this.splines) {
      for (let point of spline.control_points) {
        if (point != lastFlippedPoint) {
          point[0] = width - point[0]
        }
        lastFlippedPoint = point
      }
      this.updateSplineCurve(spline)
    }
    for (let rpoint of this.rotbreakpoints) {
      rpoint[1] = Math.PI - rpoint[1]
    }
    this.updateCanvas()
  }

  public sendPath() {
    this.optidata = null
    this.totalTime = null
    this.playing = false
    this.loading = true
    this.playBarT = 0
    this.pathOutOfDate = false
    this.updatePlayCircle()
    const headers: Headers = new Headers()
    headers.set('Content-Type', 'application/json')
    headers.set('Accept', 'application/json')
    let paths = this.getPathJson()
    const request = new Request('/api/solve', {
      method: "POST",
      headers,
      body: JSON.stringify(paths)
    })
    let stoppoints = structuredClone(this.stopPoints)

    fetch(request).then(async (res) => {
      let paths: OptiData[] = await res.json()

      let lastTime = stoppoints[0].delay
      let optidata = {
        driving_currents: [],
        voltages: [],
        accelerations: [],
        velocities: [],
        positions: [],
        times: [],
        module_forces: [],
        lat_forces: [],
        ang_vels: [],
        mod_vels: []
      }
      let i = 1
      console.log(stoppoints)
      for (let opti of paths) {
        //add 2 zeroes to the start
        for (let key in optidata) {
          if (opti[key][0] instanceof Array) {
            if (opti[key][0][0] instanceof Array) {
              optidata[key].push(opti[key][0].map(e => e.map(_ => 0)))
              optidata[key].push(opti[key][0].map(e => e.map(_ => 0)))
            } else {
              optidata[key].push(opti[key][0].map(_ => 0))
              optidata[key].push(opti[key][0].map(_ => 0))
            }
          } else {
            optidata[key].push(0)
            optidata[key].push(0)
          }
        }

        //replace the position zeroes with the next value
        optidata.positions[optidata.positions.length - 1] = opti.positions[0].map((v, i) => i == 2 ? -v : this.field_to_pixels((1 - i * 2) * v))
        optidata.positions[optidata.positions.length - 2] = opti.positions[0].map((v, i) => i == 2 ? -v : this.field_to_pixels((1 - i * 2) * v))
        console.log(opti)

        //replace the initial time with the previous time, if possible
        if (optidata.times.length != 2) {
          optidata.times[optidata.times.length - 2] = optidata.times[optidata.times.length - 3]
        }
        optidata.times[optidata.times.length - 1] = lastTime

        optidata.driving_currents.push(...opti.driving_currents)
        optidata.voltages.push(...opti.voltages)
        optidata.accelerations.push(...opti.accelerations)
        optidata.velocities.push(...opti.velocities)
        optidata.positions.push(...opti.positions.map(e => e.map((v, i) => i == 2 ? -v : this.field_to_pixels((1 - i * 2) * v))))
        optidata.times.push(...opti.times.map(e => e + lastTime))
        optidata.module_forces.push(...opti.module_forces)
        optidata.lat_forces.push(...opti.lat_forces)
        optidata.ang_vels.push(...opti.ang_vels)
        optidata.mod_vels.push(...opti.mod_vels)

        if (i < stoppoints.length) {
          lastTime = optidata.times[optidata.times.length - 1] + stoppoints[i].delay
        } else {
          lastTime = optidata.times[optidata.times.length - 1]
        }

        i++
        console.log("RAA")
        //bring keys up to time length
        //everything is assumed 0 at end (position is already the correct length, so no jumps there)
        //no more off-by-one errors! surely this will cause no future problems (cope)
        console.log(optidata)
        let finallen = optidata.times.length
        for (let key in optidata) {
          while (optidata[key].length < finallen) {
            if (optidata[key][0] instanceof Array) {
              if (optidata[key][0][0] instanceof Array) {
                optidata[key].push(optidata[key][0].map(e => e.map(_ => 0)))
              } else {
                optidata[key].push(optidata[key][0].map(_ => 0))
              }
            } else {
              optidata[key].push(0)
            }
          }
        }


      }
      console.log(optidata)
      this.optidata = optidata
      this.totalTime = lastTime
      this.loading = false
      this.updateCanvas()
    })
    this.updateCanvas()
  }

  private addrotbreakpoint(t: number, c: boolean, r?: number) {
    if (c) {
      this.changeInProgress = new ChangeEvent(this)
    }
    if (r === undefined) {
      r = this.getRotationAtT(t)
    }
    for (let i = 0; ; i++) {
      if (i >= this.rotbreakpoints.length || this.rotbreakpoints[i][0] >= t) {
        if (this.rotbreakpoints[i][0] != t) {
          this.rotbreakpoints.splice(i, 0, [t, r])
        }
        break
      }
    }
    if (c) {
      this.pathOutOfDate = true
      this.finalizeChange()
    }
  }

  private fixrotation() {
    let p0 = -1
    for (let pair of this.rotbreakpoints) {
      if (pair[0] != 0) {
        while (pair[1] < p0 - Math.PI) {
          pair[1] += 2 * Math.PI
        }
        while (pair[1] > p0 + Math.PI) {
          pair[1] -= 2 * Math.PI
        }
      }
      p0 = pair[1]
    }

    console.log(this.rotbreakpoints);
  }


  private getRobotInfoAtTime(time: number, key: keyof OptiData): any {
    if (key == "times") {
      return time
    }
    if (time >= this.totalTime) {
      return this.optidata[key][this.optidata.positions.length - 1]
    }
    if (time < 0) {
      return this.optidata[key][0]
    }
    let start = 0
    let end = this.optidata.times.length - 1
    while (end - start > 1) {
      let mid = Math.floor((1 + start + end) / 2)
      if (time < this.optidata.times[mid]) {
        end = mid
      } else if (time > this.optidata.times[mid]) {
        start = mid
      } else {
        return this.optidata[key][mid]
      }
    }

    let dtime = this.optidata.times[end] - this.optidata.times[start]
    let sfactor = (this.optidata.times[end] - time) / dtime
    let efactor = (time - this.optidata.times[start]) / dtime
    let sublists = Array.isArray(this.optidata[key][start][0])
    if (!sublists) {
      return this.optidata[key][start].map((v, i) => (v as number) * sfactor + (this.optidata[key][end][i] as number) * efactor)
    } else {
      return this.optidata[key][start].map((e, ei) => (e as [number, number]).map((v, i) => v * sfactor + this.optidata[key][end][ei][i] * efactor))
    }
  }

  private updatePlayCircle() {
    (this.playbarcircle as any).cx.baseVal.value = 10 + this.playBarT * (this.playbar.clientWidth - 20)
  }

  public togglePlay() {
    if (this.optidata != null) {
      this.playing = !this.playing
    }
  }

  public addG1Spline() {
    if (this.mode == "add G1 spline") {
      this.switchModes("view")
      this.changeInProgress = null
    } else {
      this.switchModes("add G1 spline")
      this.changeInProgress = new ChangeEvent(this)
      if (this.splines.length === 0) {
        this.pointsToAdd = 6
      }
    }
  }

  public addG0Spline() {
    if (this.mode == "add G0 spline") {
      this.switchModes("view")
      this.changeInProgress = null
    } else {
      this.switchModes("add G0 spline")
      this.changeInProgress = new ChangeEvent(this)
    }
  }

  public clear() {
    this.splines = []
    this.closeCurvePoint = null
    this.closeCurveT = null
    this.rotbreakpoints = [[0, 0], [1, Math.PI]]
    this.staticrotbreakpoints = [this.rotbreakpoints[0][0], this.rotbreakpoints[1][0]]
    this.switchModes("view")
  }

  public clearRotPoints() {
    this.changeInProgress = new ChangeEvent(this)
    this.rotbreakpoints = this.rotbreakpoints.filter(e => this.staticrotbreakpoints.indexOf(e[0]) !== -1)
    this.grabbedRotPointLen = null
    this.grabbedRotPointRot = null
    this.finalizeChange()
    this.updateCanvas()
  }

  public clearSectionConstraints() {
    this.changeInProgress = new ChangeEvent(this)
    this.constraints = []
    this.selectedConstraint = null
    this.selectedContext = new GlobalContext(this.globalConstraints, this)
    this.finalizeChange()
    this.updateCanvas()
  }

  public editRotPoints() {
    if (this.mode == "edit rotation breakpoints") {
      this.switchModes("view")
    } else {
      this.switchModes("edit rotation breakpoints")
    }
    this.updateCanvas()
  }

  public editActions() {
    if (this.mode == "edit actions") {
      this.switchModes("view")
    } else {
      this.switchModes("edit actions")
    }
    this.updateCanvas()
  }

  public clearActions() {
    this.actions = []
    this.updateCanvas()
  }

  public editSectionConstraints() {
    if (this.mode == "edit section constraints") {
      this.switchModes("view")
    } else {
      this.switchModes("edit section constraints")
    }
    this.updateCanvas()
  }

  public removeConstraint(constraint: Constraint) {
    this.constraints = this.constraints.filter(e => e != constraint)
    if (this.selectedConstraint == constraint) {
      this.selectedConstraint = null
      this.selectedContext = new GlobalContext(this.globalConstraints, this)
    }
  }

  public nextPlaySpeed() {
    if (this.playspeed == 1) {
      this.playspeed = 0.5
    }
    else if (this.playspeed == 0.5) {
      this.playspeed = 0.25
    }
    else if (this.playspeed == 0.25) {
      this.playspeed = 0.01
    }
    else if (this.playspeed == 0.01) {
      this.playspeed = 4
    }
    else if (this.playspeed == 4) {
      this.playspeed = 2
    } else {
      this.playspeed = 1
    }
  }

  private undonePointsStack: Point[] = []

  public undo() {
    if (this.addedPoints.length > 0) {
      this.undonePointsStack.push(this.addedPoints.pop())
      this.pointsToAdd++
    } else {
      if (this.undoEventStack.length > 0) {
        let ev = this.undoEventStack.pop()
        ev.undo(this)
        this.redoEventStack.push(ev)
      }
    }
    this.updateCanvas()
  }

  public redo() {
    if (this.undonePointsStack.length > 0) {
      this.addedPoints.push(this.undonePointsStack.pop())
      this.pointsToAdd--
    } else {
      if (this.redoEventStack.length > 0) {
        let ev = this.redoEventStack.pop()
        ev.redo(this)
        this.undoEventStack.push(ev)
      }
    }
    this.updateCanvas()
  }

  public updateHoverT(h: number) {
    this.hoverT = h
    if (h != null) {
      this.closeCurveT = this.getTFromTime(h * this.totalTime)
      this.closeCurvePoint = this.getPosAtT(this.closeCurveT)
    } else {
      this.closeCurveT = null
      this.closeCurvePoint = null
    }
    this.updateCanvas()
  }

  public getTFromTime(time: number): number {
    if (time >= this.totalTime) {
      return 1
    }
    if (time <= 0) {
      return 0
    }
    let start = 0
    let end = this.optidata.times.length - 1
    while (end - start > 1) {
      let mid = Math.floor((1 + start + end) / 2)
      if (time < this.optidata.times[mid]) {
        end = mid
      } else if (time > this.optidata.times[mid]) {
        start = mid
      } else {
        return this.optidata.times[mid]
      }
    }
    let dt = this.optidata.times[end] - this.optidata.times[start]
    let sfac = (time - this.optidata.times[start]) / dt
    return (start + sfac) / (this.optidata.times.length - 1)
  }

  public toggleForceCheckbox() {
    this.showForces = !this.showForces
    this.displayForces.checked = this.showForces
    this.updateCanvas()
  }
  public toggleVelocityCheckbox() {
    this.showVelocities = !this.showVelocities
    this.displayVelocities.checked = this.showVelocities
    this.updateCanvas()
  }

  public getTimeSignature() {
    if (this.totalTime === null) {
      return "-:--.---/-:--.---"
    }
    let currTime: string = "" + Math.floor(this.playBarT * this.totalTime / 60) + ":" + ("0" + ((this.playBarT * this.totalTime) % 60).toFixed(3)).slice(-6)
    let totTime: string = "" + Math.floor(this.totalTime / 60) + ":" + ("0" + (this.totalTime % 60).toFixed(3)).slice(-6)
    return currTime + "/" + totTime
  }

  public handlefieldchange(ev: Event) {
    for (let field of this.fields) {
      if (field.year == parseInt((ev.target as HTMLSelectElement).value)) {
        this.field = field
        this.updateCanvas()
        return
      }
    }
  }

  public finalizeChange() {
    this.redoEventStack = []
    this.changeInProgress.finalize(this)
    this.undoEventStack.push(this.changeInProgress)
    this.changeInProgress = null
  }
}
