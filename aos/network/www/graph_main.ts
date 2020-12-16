// This file creates a set of two demonstration plots for testing out the
// plotting utility.
// To run this and see a demo, run:
// bazel run -c opt //aos/network/www:web_proxy_demo
// And then open localhost:8080/graph.html
//
// The plots are just the plots of the handler latency / run times of the first
// timer in the the /aos timing report message (this message was chosen because
// it is already being published by the web proxy process, so the demo requires
// very little setup).
import * as configuration from 'org_frc971/aos/configuration_generated';
import {Line, Plot} from 'org_frc971/aos/network/www/plotter';
import * as proxy from 'org_frc971/aos/network/www/proxy';
import * as web_proxy from 'org_frc971/aos/network/web_proxy_generated';
import * as reflection from 'org_frc971/aos/network/www/reflection'
import * as flatbuffers_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';

import Channel = configuration.aos.Channel;
import Connection = proxy.Connection;
import Configuration = configuration.aos.Configuration;
import Parser = reflection.Parser;
import Table = reflection.Table;
import SubscriberRequest = web_proxy.aos.web_proxy.SubscriberRequest;
import ChannelRequest = web_proxy.aos.web_proxy.ChannelRequest;
import TransferMethod = web_proxy.aos.web_proxy.TransferMethod;

const width = 900;
const height = 400;

const helpDiv = document.createElement('div');
helpDiv.style.top = '0';
helpDiv.style.left = '0';
helpDiv.style.position = 'absolute';
document.body.appendChild(helpDiv);
helpDiv.innerHTML =
    'Help: click + drag to pan, double click to reset, scroll to zoom. ' +
    'Hold the x/y keys to only pan/zoom along the x/y axes.<br>' +
    'X-axes of the top two plots are linked together.';

const div = document.createElement('div');
div.style.top = '40';
div.style.left = '0';
div.style.position = 'absolute';
document.body.appendChild(div);

const div2 = document.createElement('div');
div2.style.top = (parseFloat(div.style.top) + height).toString();
div2.style.left = '0';
div2.style.position = 'absolute';
document.body.appendChild(div2);

const benchmarkDiv = document.createElement('div');
benchmarkDiv.style.top = (parseFloat(div2.style.top) + height).toString();
benchmarkDiv.style.left = '0';
benchmarkDiv.style.position = 'absolute';
document.body.appendChild(benchmarkDiv);

const handlerTimePlot = new Plot(div, width, height);
const latencyPlot = new Plot(div2, width, height);
const benchmarkPlot = new Plot(benchmarkDiv, width, height);

// Class to store the lines for plotting a Statistics message.
class StatLines {
  private max: Line;
  private min: Line;
  private average: Line;
  private maxPoints: number[] = [];
  private minPoints: number[] = [];
  private averagePoints: number[] = [];
  constructor(plotter: Plot) {
    this.max = plotter.getDrawer().addLine();
    this.min = plotter.getDrawer().addLine();
    this.average = plotter.getDrawer().addLine();

    this.max.setLabel("Max");
    this.min.setLabel("Min");
    this.average.setLabel("Average");

    this.max.setColor([1, 0, 0]);
    this.min.setColor([0, 1, 0]);
    this.average.setColor([0, 0, 1]);
  }
  addPoints(parser: Parser, statsTable: Table, time: number) {
    this.maxPoints.push(time);
    this.minPoints.push(time);
    this.averagePoints.push(time);
    this.maxPoints.push(parser.readScalar(statsTable, "max") * 1000);
    this.minPoints.push(parser.readScalar(statsTable, "min") * 1000);
    this.averagePoints.push(parser.readScalar(statsTable, "average") * 1000);


    // TODO: These memory allocations absolutely kill performance.
    this.max.setPoints(new Float32Array(this.maxPoints));
    this.min.setPoints(new Float32Array(this.minPoints));
    this.average.setPoints(new Float32Array(this.averagePoints));
  }
}

function setupPlot(plotter: Plot, title: string): StatLines {
  plotter.getAxisLabels().setXLabel("Monotonic send time since start (sec)");
  plotter.getAxisLabels().setYLabel("Time (msec)");
  plotter.getAxisLabels().setTitle(title);
  return new StatLines(plotter);
}

// Sets of the two x-axes to be shared; remove this to be able to zoom/pan the
// x-axes independently.
handlerTimePlot.linkXAxis(latencyPlot);

const handlerLines = setupPlot(handlerTimePlot, "Handler Run Times");
const latencyLines = setupPlot(latencyPlot, "Handler Latencies");

const conn = new Connection();

conn.connect();

const timingReport = {
  name: '/aos',
  type: 'aos.timing.Report',
};

let reportParser = null;

conn.addConfigHandler((config: Configuration) => {
  // Locate the timing report schema so that we can read the received messages.
  let reportSchema = null;
  for (let ii = 0; ii < config.channelsLength(); ++ii) {
    if (config.channels(ii).type() === timingReport.type) {
      reportSchema = config.channels(ii).schema();
    }
  }
  if (reportSchema === null) {
    throw new Error('Couldn\'t find timing report schema in config.');
  }
  reportParser = new Parser(reportSchema);

  // Subscribe to the timing report message.
  const builder =
      new flatbuffers_builder.Builder(512) as unknown as flatbuffers.Builder;
  const channels: flatbuffers.Offset[] = [];
  for (const channel of [timingReport]) {
    const nameFb = builder.createString(channel.name);
    const typeFb = builder.createString(channel.type);
    Channel.startChannel(builder);
    Channel.addName(builder, nameFb);
    Channel.addType(builder, typeFb);
    const channelFb = Channel.endChannel(builder);
    ChannelRequest.startChannelRequest(builder);
    ChannelRequest.addChannel(builder, channelFb);
    ChannelRequest.addMethod(builder, TransferMethod.EVERYTHING_WITH_HISTORY);
    channels.push(ChannelRequest.endChannelRequest(builder));
  }

  const channelsFb = SubscriberRequest.createChannelsToTransferVector(builder, channels);
  SubscriberRequest.startSubscriberRequest(builder);
  SubscriberRequest.addChannelsToTransfer(builder, channelsFb);
  const connect = SubscriberRequest.endSubscriberRequest(builder);
  builder.finish(connect);
  conn.sendConnectMessage(builder);
});

let startTime = null;
conn.addHandler(timingReport.type, (data: Uint8Array, time: number) => {
  if (startTime === null) {
    startTime = time;
  }
  time = time - startTime;
  const table = Table.getRootTable(new ByteBuffer(data));

  const timer = reportParser.readVectorOfTables(table, "timers")[0];
  handlerLines.addPoints(
      reportParser, reportParser.readTable(timer, 'handler_time'), time);
  latencyLines.addPoints(
      reportParser, reportParser.readTable(timer, 'wakeup_latency'), time);
});

// Set up and draw the benchmarking plot
benchmarkPlot.getAxisLabels().setTitle(
    'Benchmarkping plot (1M points per line)');
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
