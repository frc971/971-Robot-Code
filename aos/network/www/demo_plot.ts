// This file provides a basic demonstration of the plotting functionality.
// The plotDemo() function provided here is called by
// //frc971/analysis:plot_index.ts
// To view the demo plot, run
// bazel run -c opt //frc971/analysis:live_web_plotter_demo
// And then navigate to
// http://localhost:8080/?plot=Demo
// The plot=Demo isn't structly necessary, but ensures that this plot is
// selected by default--otherwise, you'll need to select "Demo" from the
// drop-down menu.
//
// This example shows how to:
// (a) Make use of the AosPlotter to plot a shmem message as a time-series.
// (b) Define your own custom plot with whatever data you want.
import {AosPlotter} from 'org_frc971/aos/network/www/aos_plotter';
import {Plot} from 'org_frc971/aos/network/www/plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;

export function plotDemo(conn: Connection, parentDiv: Element): void {
  const width = 900;
  const height = 400;

  const benchmarkDiv = document.createElement('div');
  benchmarkDiv.style.top = height.toString();
  benchmarkDiv.style.left = '0';
  benchmarkDiv.style.position = 'absolute';
  parentDiv.appendChild(benchmarkDiv);

  const benchmarkPlot = new Plot(benchmarkDiv, width, height);

  const aosPlotter = new AosPlotter(conn);

  {
    // Setup a plot that just shows the PID of each timing report message.
    // For the basic live_web_plotter_demo, this will be a boring line showing
    // just the PID of the proxy process. On a real system, or against a logfile,
    // this would show the PIDs of all active processes.
    const timingReport =
        aosPlotter.addMessageSource('/aos', 'aos.timing.Report');
    const timingPlot =
        aosPlotter.addPlot(parentDiv, [0, 0], [width, height]);
    timingPlot.plot.getAxisLabels().setTitle('Timing Report PID');
    timingPlot.plot.getAxisLabels().setYLabel('PID');
    timingPlot.plot.getAxisLabels().setXLabel('Monotonic Send Time (sec)');
    const msgLine = timingPlot.addMessageLine(timingReport, ['pid']);
    msgLine.setDrawLine(false);
    msgLine.setPointSize(5);
  }

  // Set up and draw the benchmarking plot.
  benchmarkPlot.getAxisLabels().setTitle(
      'Benchmarking plot (1M points per line)');
  const line1 = benchmarkPlot.getDrawer().addLine();
  // For demonstration purposes, make line1 only have points and line2 only have
  // lines.
  line1.setDrawLine(false);
  line1.setLabel('LINE ONE');
  const line2 = benchmarkPlot.getDrawer().addLine();
  line2.setPointSize(0);
  line2.setLabel('LINE TWO');
  const NUM_POINTS = 1000000;
  const points1 = [];
  const points2 = [];
  for (let ii = 0; ii < NUM_POINTS; ++ii) {
    points1.push(ii);
    points2.push(ii);
    points1.push(Math.sin(ii * 10 / NUM_POINTS));
    points2.push(Math.cos(ii * 10 / NUM_POINTS));
  }
  line1.setPoints(new Float32Array(points1));
  line2.setPoints(new Float32Array(points2));
}
