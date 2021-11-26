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
import {Plot, Point} from 'org_frc971/aos/network/www/plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;

export function plotDemo(conn: Connection, parentDiv: Element): void {
  const width = AosPlotter.DEFAULT_WIDTH;
  const height = AosPlotter.DEFAULT_HEIGHT;

  const benchmarkDiv = document.createElement('div');
  benchmarkDiv.style.width = width.toString() + "px";
  benchmarkDiv.style.height = height.toString() + "px";
  benchmarkDiv.style.position = 'relative';
  parentDiv.appendChild(benchmarkDiv);

  const benchmarkPlot = new Plot(benchmarkDiv);

  const aosPlotter = new AosPlotter(conn);

  {
    // Setup a plot that shows some arbitrary PDP current values.
    const pdpValues =
        aosPlotter.addMessageSource('/aos', 'frc971.PDPValues');
    const timingPlot = aosPlotter.addPlot(parentDiv);
    timingPlot.plot.getAxisLabels().setTitle('Current Values');
    timingPlot.plot.getAxisLabels().setYLabel('Current (Amps)');
    timingPlot.plot.getAxisLabels().setXLabel('Monotonic Send Time (sec)');
    // Displays points for every single current sample at each time-point.
    const allValuesLine = timingPlot.addMessageLine(pdpValues, ['currents[]']);
    allValuesLine.setDrawLine(false);
    allValuesLine.setPointSize(5);
    // Displays a line for the current along channel 1.
    const singleValueLine = timingPlot.addMessageLine(pdpValues, ['currents[1]']);
    singleValueLine.setDrawLine(true);
    singleValueLine.setPointSize(0);
    const voltageLine = timingPlot.addMessageLine(pdpValues, ['voltage']);
    voltageLine.setPointSize(0);
  }

  {
    const timingReport =
        aosPlotter.addMessageSource('/aos', 'aos.timing.Report');
    // Setup a plot that just shows some arbitrary timing data.
    const timingPlot = aosPlotter.addPlot(parentDiv);
    timingPlot.plot.getAxisLabels().setTitle('Timing Report Wakeups');
    timingPlot.plot.getAxisLabels().setYLabel('PID');
    timingPlot.plot.getAxisLabels().setXLabel('Monotonic Send Time (sec)');
    // Show *all* the wakeup latencies for all timers.
    const allValuesLine = timingPlot.addMessageLine(
        timingReport, ['timers[]', 'wakeup_latency', 'average']);
    allValuesLine.setDrawLine(false);
    allValuesLine.setPointSize(5);
    // Show *all* the wakeup latencies for the first timer in each timing report
    // (this is not actually all that helpful unless you were to also filter by
    // PID).
    const singleValueLine = timingPlot.addMessageLine(
        timingReport, ['timers[0]', 'wakeup_latency', 'average']);
    singleValueLine.setDrawLine(true);
    singleValueLine.setPointSize(0);
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
    points1.push(new Point(ii, Math.sin(ii * 10 / NUM_POINTS)));
    points2.push(new Point(ii, Math.cos(ii * 10 / NUM_POINTS)));
  }
  line1.setPoints(points1);
  line2.setPoints(points2);
}
