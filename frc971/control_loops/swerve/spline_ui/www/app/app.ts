import {Component} from '@angular/core';

@Component({
  selector: 'spline-ui-app',
  templateUrl: './app.ng.html',
  styleUrls: ['../app/common.css'],
})
export class App {
  fileName: string = 'spline.json';
  year: number = 2024;
  splineCount: number = 0;
  splineX: number[] = [0, 0, 0, 0, 0, 0];
  splineY: number[] = [0, 0, 0, 0, 0, 0];
  longConstraint: number = 10;
  latConstraint: number = 10;
  voltageConstraint: number = 10;

  save() {
    //make a websocket call to save the spline
    const websocket = new WebSocket('ws://localhost:1180/ws');
    websocket.addEventListener('open', () => {
      const splineJson = {
        spline_count: this.splineCount,
        spline_x: this.splineX,
        spline_y: this.splineY,
        constraints: [
          {
            constraint_type: "LONGITUDINAL_ACCELERATION",
            value: this.longConstraint
          },
          {
            constraint_type: "LATERAL_ACCELERATION",
            value: this.latConstraint
          },
          {
            constraint_type: "VOLTAGE",
            value: this.voltageConstraint
          }
        ]
      };
      const splineData = JSON.stringify(splineJson, null, 4);
      const filePath = this.getPath(this.year) + this.fileName;
      websocket.send(filePath + '\n' + splineData);
    });
  }

  // copied from //frc971/control_loops/python/constants.py
  // returns the path to the spline jsons for the given year
  getPath(year: number): string {
    if(year == 2020 || year == 2021) {
      return "/y2020/actors/splines/";
    }
    else if(year == 2022) {
      return "/y2022/actors/splines/";
    }
    else if(year == 2023 || year == 2024) {
      return "/y" + year + "/autonomous/splines/";
    }
    else {
      return "/frc971/control_loops/python/spline_jsons/";
    }
  }
}