// This file provides an index of all standard plot configs.
// In the future, this may be split into more pieces (e.g., to have
// year-specific indices). For now, the pattern for creating a new plot
// is that you provide an exported function (a la the plot*() functions imported
// below) which, when called, will generate the plot in the provided div.
// This file handles providing a master list of all known plots so that
// the user can just open a single web-page and select the plot that they want
// from a drop-down. A given plot will not be loaded until it has been selected
// once, at which point it will stay loaded. This means that if the user wants
// to switch between several plot configs, they don't incur any performance
// penalty associated with swapping.
// By default, no plot is selected, but the plot= URL parameter may be used
// to specify a specific plot, so that people can create links to a specific
// plot.
// The plot*() functions are called *after* we have already received a valid
// config from the web server, so config handlers do not need to be used.
//
// The exact setup of this will be in flux as we add more configs and figure out
// what setups work best--we will likely end up with separate index files for
// each robot year, and may even end up allowing plots to be specified solely
// using JSON rather than requiring people to write a script just to create
// a plot.
import * as configuration from 'org_frc971/aos/configuration_generated';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import {plotImu} from 'org_frc971/frc971/wpilib/imu_plotter';
import {plotDrivetrain} from 'org_frc971/frc971/control_loops/drivetrain/drivetrain_plotter';
import {plotSpline} from 'org_frc971/frc971/control_loops/drivetrain/spline_plotter';
import {plotDownEstimator} from 'org_frc971/frc971/control_loops/drivetrain/down_estimator_plotter';
import {plotRobotState} from
    'org_frc971/frc971/control_loops/drivetrain/robot_state_plotter'
import {plotFinisher} from
    'org_frc971/y2020/control_loops/superstructure/finisher_plotter'
import {plotTurret} from
    'org_frc971/y2020/control_loops/superstructure/turret_plotter'
import {plotAccelerator} from
    'org_frc971/y2020/control_loops/superstructure/accelerator_plotter'
import {plotHood} from
    'org_frc971/y2020/control_loops/superstructure/hood_plotter'
import {plotDemo} from 'org_frc971/aos/network/www/demo_plot';
import {plotData} from 'org_frc971/frc971/analysis/plot_data_utils';

import Connection = proxy.Connection;
import Configuration = configuration.aos.Configuration;

const rootDiv = document.createElement('div');
document.body.appendChild(rootDiv);

const helpDiv = document.createElement('div');
rootDiv.appendChild(helpDiv);
helpDiv.innerHTML =
    'Help: click + drag to pan, double click to reset, scroll to zoom, ' +
    'right-click + drag to zoom to rectangle, Esc to cancel. ' +
    'Hold the x/y keys to only pan/zoom along the x/y axes.';

class PlotState {
  public readonly div: HTMLElement;
  private initialized = false;
  constructor(
      parentDiv: HTMLElement,
      private readonly initializer:
          (conn: Connection, element: Element) => void) {
    this.div = document.createElement('div');
    parentDiv.appendChild(this.div);
    this.hide();
  }
  initialize(conn: Connection): void {
    if (!this.initialized) {
      this.initializer(conn, this.div);
      this.initialized = true;
    }
  }
  hide(): void {
    this.div.style.display = "none";
  }
  show(): void {
    this.div.style.display = "block";
  }
}

const plotSelect = document.createElement('select');
rootDiv.appendChild(plotSelect);

const plotDiv = document.createElement('div');
plotDiv.style.top = (plotSelect.getBoundingClientRect().bottom + 10).toString();
plotDiv.style.left = '0';
plotDiv.style.position = 'absolute';
rootDiv.appendChild(plotDiv);

// The master list of all the plots that we provide. For a given config, it
// is possible that not all of these plots will be usable depending on the
// presence of certain channels.
const plotIndex = new Map<string, PlotState>([
  ['Demo', new PlotState(plotDiv, plotDemo)],
  ['IMU', new PlotState(plotDiv, plotImu)],
  ['Drivetrain', new PlotState(plotDiv, plotDrivetrain)],
  ['Spline Debug', new PlotState(plotDiv, plotSpline)],
  ['Down Estimator', new PlotState(plotDiv, plotDownEstimator)],
  ['Robot State', new PlotState(plotDiv, plotRobotState)],
  ['Finisher', new PlotState(plotDiv, plotFinisher)],
  ['Accelerator', new PlotState(plotDiv, plotAccelerator)],
  ['Hood', new PlotState(plotDiv, plotHood)],
  ['Turret', new PlotState(plotDiv, plotTurret)],
  ['C++ Plotter', new PlotState(plotDiv, plotData)],
]);

const invalidSelectValue = 'null';
function getDefaultPlot(): string {
  const urlParams = (new URL(document.URL)).searchParams;
  const urlParamKey = 'plot';
  if (!urlParams.has(urlParamKey)) {
    return invalidSelectValue;
  }
  const desiredPlot = urlParams.get(urlParamKey);
  if (!plotIndex.has(desiredPlot)) {
    return invalidSelectValue;
  }
  return desiredPlot;
}

const conn = new Connection();

let reloadOnChange = false;

conn.connect();

conn.addConfigHandler((config: Configuration) => {
  plotSelect.add(new Option("Select Plot", invalidSelectValue));
  for (const name of plotIndex.keys()) {
    plotSelect.add(new Option(name, name));
  }
  plotSelect.addEventListener('input', () => {
    for (const plot of plotIndex.values()) {
      plot.hide();
    }
    if (plotSelect.value == invalidSelectValue) {
      return;
    }
    plotIndex.get(plotSelect.value).initialize(conn);
    plotIndex.get(plotSelect.value).show();
    // Set the URL so that if you reload you get back to this plot.
    window.history.replaceState(
        null, null, '?plot=' + encodeURIComponent(plotSelect.value));
    if (reloadOnChange) {
      window.location.reload();
    }
    reloadOnChange = true;
  });
  plotSelect.value = getDefaultPlot();
  // Force the event to occur once at the start.
  plotSelect.dispatchEvent(new Event('input'));
});
