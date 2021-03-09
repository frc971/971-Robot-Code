// This library provides a wrapper around our WebGL plotter that makes it
// easy to plot AOS messages/channels as time series.
//
// This is works by subscribing to each channel that we want to plot, storing
// all the messages for that channel, and then periodically running through
// every message and extracting the fields to plot.
// It is also possible to insert code to make modifications to the messages
// as we read/process them, as is the case for the IMU processing code (see
// //frc971/wpilib:imu*.ts) where each message is actually a batch of several
// individual messages that need to be plotted as separate points.
//
// The basic flow for using the AosPlotter is:
// // 1) Construct the plotter
// const aosPlotter = new AosPlotter(connection);
// // 2) Add messages sources that we'll want to subscribe to.
// const source = aosPlotter.addMessageSource('/aos', 'aos.timing.Report');
// // 3) Create figures at defined positions within a given HTML element..
// const timingPlot = aosPlotter.addPlot(parentDiv, [0, 0], [width, height]);
// // 4) Add specific signals to each figure, using the message sources you
//       defined at the start.
// timingPlot.addMessageLine(source, ['pid']);
//
// The demo_plot.ts script has a basic example of using this library, with all
// the required boilerplate, as well as some extra examples about how to
// add axis labels and the such.
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
import Schema = configuration.reflection.Schema;
import Parser = reflection.Parser;
import Table = reflection.Table;
import SubscriberRequest = web_proxy.aos.web_proxy.SubscriberRequest;
import ChannelRequest = web_proxy.aos.web_proxy.ChannelRequest;
import TransferMethod = web_proxy.aos.web_proxy.TransferMethod;

export class TimestampedMessage {
  constructor(
      public readonly message: Table, public readonly time: number) {}
}

// The MessageHandler stores an array of every single message on a given channel
// and then supplies individual fields as arrays when requested. Currently this
// is very much unoptimized and re-processes the entire array of messages on
// every call to getField().
export class MessageHandler {
  protected parser: Parser;
  protected messages: TimestampedMessage[] = [];
  constructor(schema: Schema) {
    this.parser = new Parser(schema);
  }
  addMessage(data: Uint8Array, time: number): void {
    this.messages.push(
        new TimestampedMessage(Table.getRootTable(new ByteBuffer(data)), time));
  }
  // Returns a time-series of every single instance of the given field. Format
  // of the return value is [time0, value0, time1, value1,... timeN, valueN],
  // to match with the Line.setPoint() interface.
  // By convention, NaN is used to indicate that a message existed at a given
  // timestamp but the requested field was not populated.
  // If you want to retrieve a single signal from a vector, you can specify it
  // as "field_name[index]".
  getField(field: string[]): Float32Array {
    const fieldName = field[field.length - 1];
    const subMessage = field.slice(0, field.length - 1);
    const results = new Float32Array(this.messages.length * 2);
    for (let ii = 0; ii < this.messages.length; ++ii) {
      let message = this.messages[ii].message;
      for (const subMessageName of subMessage) {
        message = this.parser.readTable(message, subMessageName);
        if (message === undefined) {
          break;
        }
      }
      results[ii * 2] = this.messages[ii].time;
      const regex = /(.*)\[([0-9]*)\]/;
      let name = fieldName;
      let match = fieldName.match(regex);
      let index = undefined;
      if (match) {
        name = match[1]
        index = parseInt(match[2])
      }
      if (message === undefined) {
        results[ii * 2 + 1] = NaN;
      } else {
        if (index === undefined) {
          results[ii * 2 + 1] = this.parser.readScalar(message, name);
        } else {
          const vector =
              this.parser.readVectorOfScalars(message, name);
          if (index < vector.length) {
            results[ii * 2 + 1] = vector[index];
          } else {
            results[ii * 2 + 1] = NaN;
          }
        }
      }
    }
    return results;
  }
  numMessages(): number {
    return this.messages.length;
  }
}

class MessageLine {
  constructor(
      public readonly messages: MessageHandler, public readonly line: Line,
      public readonly field: string[]) {}
}

class AosPlot {
  private lines: MessageLine[] = [];
  constructor(
      private readonly plotter: AosPlotter, public readonly plot: Plot) {}

  // Adds a line to the figure.
  // message specifies what channel/data source to pull from, and field
  // specifies the field within that channel. field is an array specifying
  // the full path to the field within the message. For instance, to
  // plot whether the drivetrain is currently zeroed based on the drivetrain
  // status message, you would specify the ['zeroing', 'zeroed'] field to
  // get the DrivetrainStatus.zeroing().zeroed() member.
  // Currently, this interface does not provide any support for non-numeric
  // fields or for repeated fields (or sub-messages) of any sort.
  addMessageLine(message: MessageHandler, field: string[]): Line {
    const line = this.plot.getDrawer().addLine();
    line.setLabel(field.join('.'));
    this.lines.push(new MessageLine(message, line, field));
    return line;
  }

  draw(): void {
    // Only redraw lines if the number of points has changed--because getField()
    // is a relatively expensive call, we don't want to do it any more than
    // necessary.
    for (const line of this.lines) {
      if (line.messages.numMessages() * 2 != line.line.getPoints().length) {
        line.line.setPoints(line.messages.getField(line.field));
      }
    }
  }
}

export class AosPlotter {

  public static readonly TIME: string = "Monotonic Time (sec)";

  public static readonly DEFAULT_WIDTH: number = 900;
  public static readonly DEFAULT_HEIGHT: number = 400;

  public static readonly RED: number[] = [1, 0, 0];
  public static readonly GREEN: number[] = [0, 1, 0];
  public static readonly BLUE: number[] = [0, 0, 1];
  public static readonly BROWN: number[] = [0.6, 0.3, 0];
  public static readonly PINK: number[] = [1, 0.3, 0.6];
  public static readonly CYAN: number[] = [0.3, 1, 1];
  public static readonly WHITE: number[] = [1, 1, 1];


  private plots: AosPlot[] = [];
  private messages = new Set<MessageHandler>();
  private lowestHeight = 0;
  constructor(private readonly connection: Connection) {
    // Set up to redraw at some regular interval. The exact rate is unimportant.
    setInterval(() => {
      this.draw();
    }, 100);
  }

  // Sets up an AOS channel as a message source. Returns a handler that can
  // be passed to addMessageLine().
  addMessageSource(name: string, type: string): MessageHandler {
    return this.addRawMessageSource(
        name, type, new MessageHandler(this.connection.getSchema(type)));
  }

  // Same as addMessageSource, but allows you to specify a custom MessageHandler
  // that does some processing on the requested message. This allows you to
  // create post-processed versions of individual channels.
  addRawMessageSource(
      name: string, type: string,
      messageHandler: MessageHandler): MessageHandler {
    this.messages.add(messageHandler);
    // Use a "reliable" handler so that we get *all* the data when we are
    // plotting from a logfile.
    this.connection.addReliableHandler(
        name, type, (data: Uint8Array, time: number) => {
          messageHandler.addMessage(data, time);
        });
    return messageHandler;
  }
  // Add a new figure at the provided position with the provided size within
  // parentElement.
  addPlot(
      parentElement: Element, topLeft: number[]|null = null,
      size: number[] = [AosPlotter.DEFAULT_WIDTH, AosPlotter.DEFAULT_HEIGHT]):
      AosPlot {
    if (topLeft === null) {
      topLeft = [0, this.lowestHeight];
    }
    const div = document.createElement("div");
    div.style.top = topLeft[1].toString();
    div.style.left = topLeft[0].toString();
    div.style.position = 'absolute';
    parentElement.appendChild(div);
    const newPlot = new Plot(div, size[0], size[1]);
    for (let plot of this.plots.values()) {
      newPlot.linkXAxis(plot.plot);
    }
    this.plots.push(new AosPlot(this, newPlot));
    // Height goes up as you go down.
    this.lowestHeight = Math.max(topLeft[1] + size[1], this.lowestHeight);
    return this.plots[this.plots.length - 1];
  }
  private draw(): void {
    for (const plot of this.plots) {
      plot.draw();
    }
  }
}
