// Provides a plot which handles plotting the plot defined by a
// frc971.analysis.Plot message.
import * as configuration from 'org_frc971/aos/configuration_generated';
import * as plot_data from 'org_frc971/frc971/analysis/plot_data_generated';
import {MessageHandler, TimestampedMessage} from 'org_frc971/aos/network/www/aos_plotter';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import {Plot} from 'org_frc971/aos/network/www/plotter';
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
  plotDiv.style.position = 'absolute';
  plotDiv.style.top = '30';
  plotDiv.style.left = '0';
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
          figureDiv.style.top = figure.position().top().toString();
          figureDiv.style.left = figure.position().left().toString();
          figureDiv.style.position = 'absolute';
          div.appendChild(figureDiv);
          const plot = new Plot(
              figureDiv, figure.position().width(), figure.position().height());

          if (figure.title()) {
            plot.getAxisLabels().setTitle(figure.title());
          }
          if (figure.xlabel()) {
            plot.getAxisLabels().setXLabel(figure.xlabel());
          }
          if (figure.ylabel()) {
            plot.getAxisLabels().setYLabel(figure.xlabel());
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
            const points = new Float32Array(lineFb.pointsLength() * 2);
            for (let kk = 0; kk < lineFb.pointsLength(); ++kk) {
              points[kk * 2] = lineFb.points(kk).x();
              points[kk * 2 + 1] = lineFb.points(kk).y();
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
