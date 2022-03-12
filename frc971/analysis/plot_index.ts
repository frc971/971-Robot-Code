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
import {plotFinisher as plot2020Finisher} from
    'org_frc971/y2020/control_loops/superstructure/finisher_plotter'
import {plotTurret as plot2020Turret} from
    'org_frc971/y2020/control_loops/superstructure/turret_plotter'
import {plotLocalizer as plot2020Localizer} from
    'org_frc971/y2020/control_loops/drivetrain/localizer_plotter'
import {plotAccelerator as plot2020Accelerator} from
    'org_frc971/y2020/control_loops/superstructure/accelerator_plotter'
import {plotHood as plot2020Hood} from
    'org_frc971/y2020/control_loops/superstructure/hood_plotter'
import {plotSuperstructure as plot2021Superstructure} from
    'org_frc971/y2021_bot3/control_loops/superstructure/superstructure_plotter';
import {plotTurret as plot2022Turret} from
    'org_frc971/y2022/control_loops/superstructure/turret_plotter'
import {plotSuperstructure as plot2022Superstructure} from
    'org_frc971/y2022/control_loops/superstructure/superstructure_plotter'
import {plotCatapult as plot2022Catapult} from
    'org_frc971/y2022/control_loops/superstructure/catapult_plotter'
import {plotIntakeFront as plot2022IntakeFront, plotIntakeBack as plot2022IntakeBack} from
    'org_frc971/y2022/control_loops/superstructure/intake_plotter'
import {plotClimber as plot2022Climber} from
    'org_frc971/y2022/control_loops/superstructure/climber_plotter'
import {plotLocalizer as plot2022Localizer} from
    'org_frc971/y2022/localizer/localizer_plotter'
import {plotVision as plot2022Vision} from
    'org_frc971/y2022/vision/vision_plotter'
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
  ['2020 Finisher', new PlotState(plotDiv, plot2020Finisher)],
  ['2020 Accelerator', new PlotState(plotDiv, plot2020Accelerator)],
  ['2020 Hood', new PlotState(plotDiv, plot2020Hood)],
  ['2020 Turret', new PlotState(plotDiv, plot2020Turret)],
  ['2020 Localizer', new PlotState(plotDiv, plot2020Localizer)],
  ['2022 Localizer', new PlotState(plotDiv, plot2022Localizer)],
  ['2022 Vision', new PlotState(plotDiv, plot2022Vision)],
  ['2022 Superstructure', new PlotState(plotDiv, plot2022Superstructure)],
  ['2022 Catapult', new PlotState(plotDiv, plot2022Catapult)],
  ['2022 Intake Front', new PlotState(plotDiv, plot2022IntakeFront)],
  ['2022 Intake Back', new PlotState(plotDiv, plot2022IntakeBack)],
  ['2022 Climber', new PlotState(plotDiv, plot2022Climber)],
  ['2022 Turret', new PlotState(plotDiv, plot2022Turret)],
  ['C++ Plotter', new PlotState(plotDiv, plotData)],
  ['Y2021 3rd Robot Superstructure', new PlotState(plotDiv, plot2021Superstructure)],
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
