// Provides a plot which handles plotting the plot defined by a
// frc971.analysis.Plot message.
import * as configuration from 'org_frc971/aos/configuration_generated';
import * as plot_data from 'org_frc971/frc971/analysis/plot_data_generated';
import {MessageHandler, TimestampedMessage} from 'org_frc971/aos/network/www/aos_plotter';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import {Plot, Point} from 'org_frc971/aos/network/www/plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';

import Connection = proxy.Connection;
import Schema = configuration.reflection.Schema;
import PlotFb = plot_data.frc971.analysis.Plot;

export function plotData(conn: Connection, parentDiv: Element) {
  // Set up a selection box to allow the user to choose between plots to show.
  const plotSelect = document.createElement('select');
  parentDiv.appendChild(plotSelect);
  const plots = new Map<string, HTMLElement>();
  const invalidSelectValue = 'null';
  plotSelect.addEventListener('input', () => {
    for (const plot of plots.values()) {
      plot.style.display = 'none';
    }
    if (plotSelect.value == invalidSelectValue) {
      return;
    }
    plots.get(plotSelect.value).style.display = 'block';
  });
  plotSelect.add(new Option('Select Plot', invalidSelectValue));

  const plotDiv = document.createElement('div');
  parentDiv.appendChild(plotDiv);

  conn.addReliableHandler(
      '/analysis', 'frc971.analysis.Plot', (data: Uint8Array, time: number) => {
        const plotFb = PlotFb.getRootAsPlot(
            new ByteBuffer(data) as unknown as flatbuffers.ByteBuffer);
        const name = (!plotFb.title()) ? 'Plot ' + plots.size : plotFb.title();
        const div = document.createElement('div');
        div.style.display = 'none';
        plots.set(name, div);
        plotDiv.appendChild(div);
        plotSelect.add(new Option(name, name));

        const linkedXAxes: Plot[] = [];

        for (let ii = 0; ii < plotFb.figuresLength(); ++ii) {
          const figure = plotFb.figures(ii);
          const figureDiv = document.createElement('div');
          figureDiv.style.width = figure.position().width().toString() + "px";
          figureDiv.style.height = figure.position().height().toString() + "px";
          figureDiv.style.position = 'relative';
          div.appendChild(figureDiv);
          const plot = new Plot(figureDiv);

          if (figure.title()) {
            plot.getAxisLabels().setTitle(figure.title());
          }
          if (figure.xlabel()) {
            plot.getAxisLabels().setXLabel(figure.xlabel());
          }
          if (figure.ylabel()) {
            plot.getAxisLabels().setYLabel(figure.ylabel());
          }
          if (figure.shareXAxis()) {
            for (const other of linkedXAxes) {
              plot.linkXAxis(other);
            }
            linkedXAxes.push(plot);
          }

          for (let jj = 0; jj < figure.linesLength(); ++jj) {
            const lineFb = figure.lines(jj);
            const line = plot.getDrawer().addLine();
            if (lineFb.label()) {
              line.setLabel(lineFb.label());
            }
            const points = [];
            for (let kk = 0; kk < lineFb.pointsLength(); ++kk) {
              const point = lineFb.points(kk);
              points.push(new Point(point.x(), point.y()));
            }
            if (lineFb.color()) {
              line.setColor(
                  [lineFb.color().r(), lineFb.color().g(), lineFb.color().b()]);
            }
            line.setPoints(points);
          }
        }
      });
}
