import { Component, ViewChild, ElementRef, Input, AfterViewInit, Output, EventEmitter } from '@angular/core';
import { OptiData } from '../ui/type';

const axisoffset = 30;
const colors = {
  speed: "blue",
  rot_vel: "red",
  accel: "orange",
  min_curr: "teal",
  max_curr: "magenta"
}

@Component({
  selector: 'data-graph',
  templateUrl: './graph.ng.html',
  styleUrls: ['../../common.css'],
})
export class Graph implements AfterViewInit {
  @ViewChild("graphcanvas")
  public canvasref: ElementRef<HTMLCanvasElement>
  private canvas: HTMLCanvasElement
  private ctx: CanvasRenderingContext2D

  @ViewChild("divtosize")
  public divtosizeref: ElementRef<HTMLDivElement>;
  private divtosize: HTMLDivElement;

  @ViewChild("infodiv")
  public infodivref: ElementRef<HTMLDivElement>;
  private infodiv: HTMLDivElement;

  @Input() data: OptiData;

  @Input() positionT: number;
  @Input() hoverT: number
  @Input() outOfDate: boolean;

  @Output() onHover = new EventEmitter<number>()

  public hoverInfo: { text: string, color: string, superscript?: number }[] = [];

  private updateCanvas() {
    if (this.data != null) {
      const maxtime = this.data.times[this.data.times.length - 1]
      this.ctx.fillStyle = "white";
      this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

      //draw centerline
      this.ctx.strokeStyle = "gray"
      this.ctx.lineWidth = 2;
      this.ctx.beginPath();
      this.ctx.moveTo(axisoffset, this.canvas.height / 2)
      this.ctx.lineTo(this.canvas.width - axisoffset, this.canvas.height / 2)
      this.ctx.stroke();

      //draw axes

      this.ctx.strokeStyle = "black";
      this.ctx.lineWidth = 2;


      this.ctx.beginPath();
      this.ctx.moveTo(axisoffset, axisoffset);
      this.ctx.lineTo(axisoffset, this.canvas.height - axisoffset);
      this.ctx.lineTo(this.canvas.width - axisoffset, this.canvas.height - axisoffset);
      this.ctx.stroke();

      //draw X tick marks
      //get subdivision of time
      //must be less than 50 tick marks
      //must be some power of 10 times 1, 2, or 5
      const maxTickMarks = 50;
      let subdivision = 2;
      let scaleExponent = -2; //assuming that we will never have a path that is less than 1 sec long
      while (maxtime / (subdivision * 10 ** scaleExponent) > maxTickMarks) {
        if (subdivision == 1) {
          subdivision = 2;
        } else if (subdivision == 2) {
          subdivision = 5;
        } else {
          subdivision = 1;
          scaleExponent += 1;
        }
      }
      let timeDivision = subdivision * 10 ** scaleExponent;
      let currTime = 0;
      this.ctx.strokeStyle = "black";
      this.ctx.lineWidth = 1;
      this.ctx.textAlign = "center"
      this.ctx.textBaseline = "top"
      this.ctx.fillStyle = "black"
      while (currTime < maxtime) {
        this.ctx.beginPath();
        let x = currTime / maxtime * (this.canvas.width - axisoffset * 2) + axisoffset
        this.ctx.moveTo(x, this.canvas.height - axisoffset)
        this.ctx.lineTo(x, this.canvas.height - axisoffset + 3)
        this.ctx.stroke();
        this.ctx.fillText(currTime.toFixed(-scaleExponent), x, this.canvas.height - axisoffset + 5)
        currTime += timeDivision
      }
      this.ctx.fillText("Time (sec)", this.canvas.width / 2, this.canvas.height - axisoffset + 20);

      //draw lines
      this.ctx.lineWidth = 1;
      this.ctx.strokeStyle = colors.speed
      this.drawLine(this.data.times, this.data.velocities.map(e => (e[0] ** 2 + e[1] ** 2) ** 0.5), maxtime, 10)
      this.ctx.strokeStyle = colors.rot_vel
      this.drawLine(this.data.times, this.data.velocities.map(e => e[2] + 10), maxtime, 20)
      this.ctx.strokeStyle = colors.accel
      this.drawLine(this.data.times, this.data.accelerations.map(e => (e[0] ** 2 + e[1] ** 2) ** 0.5), maxtime, 25)
      this.ctx.strokeStyle = colors.min_curr
      this.drawLine(this.data.times, this.data.driving_currents.map(e => Math.min(...e) + 40), maxtime, 80)
      this.ctx.strokeStyle = colors.max_curr
      this.drawLine(this.data.times, this.data.driving_currents.map(e => Math.max(...e) + 40), maxtime, 80)

      //draw hovering line
      this.ctx.strokeStyle = "#9955FF"
      this.ctx.lineWidth = 2;
      this.ctx.beginPath();
      this.ctx.moveTo(axisoffset + this.positionT * (this.canvas.width - 2 * axisoffset), axisoffset)
      this.ctx.lineTo(axisoffset + this.positionT * (this.canvas.width - 2 * axisoffset), this.canvas.height - axisoffset)
      this.ctx.stroke();
      this.ctx.strokeStyle = "black"
      this.ctx.lineWidth = 2;
      this.ctx.beginPath();
      this.ctx.moveTo(axisoffset + this.hoverT * (this.canvas.width - 2 * axisoffset), axisoffset)
      this.ctx.lineTo(axisoffset + this.hoverT * (this.canvas.width - 2 * axisoffset), this.canvas.height - axisoffset)
      this.ctx.stroke();
    }
    this.updateInfoDiv()
  }

  private drawLine(xData: number[], yData: number[], xMax: number, yMax: number) {
    this.ctx.beginPath();
    this.ctx.moveTo(xData[0] / xMax * (this.canvas.width - 2 * axisoffset) + axisoffset,
      this.canvas.height - axisoffset - yData[0] / yMax * (this.canvas.height - 2 * axisoffset))
    for (let i = 1; i < yData.length; i++) {
      let x = xData[i]
      this.ctx.lineTo(x / xMax * (this.canvas.width - 2 * axisoffset) + axisoffset, this.canvas.height - axisoffset - yData[i] / yMax * (this.canvas.height - 2 * axisoffset))
    }
    this.ctx.stroke();
  }

  ngAfterViewInit() {
    this.canvas = this.canvasref.nativeElement;
    this.ctx = this.canvas.getContext("2d")
    this.divtosize = this.divtosizeref.nativeElement
    this.canvas.width = this.divtosize.getBoundingClientRect().width - 4
    this.infodiv = this.infodivref.nativeElement;

    this.canvas.addEventListener("mousemove", ev => {
      let t = (ev.offsetX - axisoffset) / (this.canvas.width - 2 * axisoffset);
      t = t < 0 ? 0 : t;
      t = t > 1 ? 1 : t;
      this.onHover.emit(t);
    })

    this.canvas.addEventListener("mouseleave", ev => {
      this.onHover.emit(null);
    })
  }

  ngOnChanges() {
    if (this.data != null) {
      this.updateCanvas();
      this.updateInfoDiv();
    }
  }

  private updateInfoDiv() {
    this.infodiv.style.display = this.hoverT == null ? "none" : "block";
    if (this.hoverT != null) {
      let text = []
      //get all the data and stuff
      const time = this.hoverT * this.data.times[this.data.times.length - 1]
      let start = 0;
      let end = this.data.times.length - 1;
      while (end - start > 1) {
        let mid = Math.floor((start + end + 1) / 2)
        if (time < this.data.times[mid]) {
          end = mid;
        } else if (time > this.data.times[mid]) {
          start = mid;
        } else {
          start = mid;
          end = mid;
        }
      }

      let dt = this.data.times[end] - this.data.times[start]
      let sfac, efac;
      if (dt == 0) {
        sfac = 0;
        efac = 1;
      } else {
        sfac = (this.data.times[end] - time) / dt;
        efac = (time - this.data.times[start]) / dt;
      }
      let pdata: { [key in keyof OptiData]: number | number[] | number[][] } = {} as { [key in keyof OptiData]: number };
      for (let key in this.data) {
        let subarr = Array.isArray(this.data[key][start]);
        if (subarr) {
          let subsubarr = Array.isArray(this.data[key][start][0])
          if (subsubarr) {
            pdata[key] = this.data[key][start].map((e, i) => e.map((v, vi) => v * sfac + this.data[key][end][i][vi] * efac))
          } else {
            pdata[key] = this.data[key][start].map((e, i) => e * sfac + this.data[key][end][i] * efac)
          }
        } else {
          pdata[key] = this.data[key][start] * sfac + this.data[key][end] * efac
        }
      }

      text.push({ text: "time: " + (pdata.times as number).toFixed(3) + " s\n", color: "black" });
      text.push({ text: "speed: " + (((pdata.velocities as number[])[0] ** 2 + (pdata.velocities as number[])[1] ** 2) ** 0.5).toFixed(2) + " m/s", color: colors.speed });
      text.push({ text: "rotational velocity: " + (pdata.velocities as number[])[2].toFixed(3) + " rad/s", color: colors.rot_vel });
      text.push({ text: "acceleration: " + (((pdata.accelerations as number[])[0] ** 2 + (pdata.velocities as number[])[1] ** 2) ** 0.5).toFixed(2) + " m/s", color: colors.accel, superscript: 2 });
      text.push({ text: "maximum current: " + Math.max(...pdata.driving_currents as number[]).toFixed(1) + " A", color: colors.max_curr })
      text.push({ text: "minimum current: " + Math.min(...pdata.driving_currents as number[]).toFixed(1) + " A", color: colors.min_curr })

      this.hoverInfo = text;

      if (time > this.data.times[this.data.times.length - 1] / 2) {
        this.infodiv.style.translate = (axisoffset + this.hoverT * (this.canvas.width - 2 * axisoffset) + 3 - this.infodiv.getBoundingClientRect().width) + "px 10px"
      } else {
        this.infodiv.style.translate = (axisoffset + this.hoverT * (this.canvas.width - 2 * axisoffset) + 1) + "px 10px"
      }
    }
  }
}
