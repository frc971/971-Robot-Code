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
import {Channel, Configuration} from 'org_frc971/aos/configuration_generated';
import {Line, Plot, Point} from 'org_frc971/aos/network/www/plotter';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import {SubscriberRequest, ChannelRequest, TransferMethod} from 'org_frc971/aos/network/web_proxy_generated';
import {Parser, Table} from 'org_frc971/aos/network/www/reflection'
import {Schema} from 'flatbuffers_reflection/reflection_generated';
import {ByteBuffer} from 'flatbuffers';

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
  private parseFieldName(rawName: string): [string, boolean, number|null] {
    // If the fieldName includes an array index at the end, attempt to read a
    // vector.
    // The indices can either be in the form [X] for some index X, or be an
    // empty [], which is a request to return all values.
    const regex = /(.*)\[([0-9]*)\]/;
    const match = rawName.match(regex);
    if (match) {
      const name = match[1];
      const requestFullVector = match[2] === '';
      const requestedIndex = requestFullVector ? null : parseInt(match[2]);
      return [name, true, requestedIndex];
    } else {
      return [rawName, false, null];
    }
  }
  private readField<T>(
      typeIndex: number, fieldName: string,
      normalReader: (typeIndex: number, name: string) => (t: Table) => T | null,
      vectorReader: (typeIndex: number, name: string) => (t: Table) => T[] |
          null): (t: Table) => T[] | null {
    const [name, isVector, vectorIndex] = this.parseFieldName(fieldName);
    if (isVector) {
      const vectorLambda = vectorReader(typeIndex, name);
      const requestFullVector = vectorIndex === null;
      return (message: Table) => {
        const vector = vectorLambda(message);
        if (vector === null) {
          return null;
        }
        if (requestFullVector) {
          return vector;
        } else {
          return (vectorIndex < vector.length) ? [vector[vectorIndex]] : null;
        }
      };
    }
    const singleLambda = normalReader(typeIndex, fieldName);
    return (message: Table) => {
      // For single values, return as a 1-vector so that the type is
      // consistent with that used for vectors.
      const singleValue = singleLambda(message);
      return (singleValue === null) ? null : [singleValue];
    };
  }
  // Returns a time-series of every single instance of the given field. Format
  // of the return value is [time0, value0, time1, value1,... timeN, valueN],
  // to match with the Line.setPoint() interface.
  // By convention, NaN is used to indicate that a message existed at a given
  // timestamp but the requested field was not populated.
  // If you want to retrieve a single signal from a vector, you can specify it
  // as "field_name[index]".
  getField(field: string[]): Point[] {
    if (this.messages.length == 0) {
      return [];
    }
    const rootType = this.messages[0].message.typeIndex;
    const fieldName = field[field.length - 1];
    const subMessage = field.slice(0, field.length - 1);

    const lambdas = [];
    let currentType = rootType;
    for (const subMessageName of subMessage) {
      lambdas.push(this.readField(
          currentType, subMessageName,
          (typeIndex: number, name: string) =>
              this.parser.readTableLambda(typeIndex, name),
          (typeIndex: number, name: string) =>
              this.parser.readVectorOfTablesLambda(typeIndex, name)));
      const [name, isVector, vectorIndex] = this.parseFieldName(subMessageName);
      currentType =
          this.parser.getField(name, currentType).type().index();
    }
    const fieldLambda = this.readField(
        currentType, fieldName,
        (typeIndex: number, name: string) =>
            this.parser.readScalarLambda(typeIndex, name),
        (typeIndex: number, name: string) =>
            this.parser.readVectorOfScalarsLambda(typeIndex, name));

    const results = [];
    for (let ii = 0; ii < this.messages.length; ++ii) {
      let tables = [this.messages[ii].message];
      for (const lambda of lambdas) {
        let nextTables = [];
        for (const table of tables) {
          const nextTable = lambda(table);
          if (nextTable === null) {
            continue;
          }
          nextTables = nextTables.concat(nextTable);
        }
        tables = nextTables;
      }

      const values = [];
      for (const table of tables) {
        const value = fieldLambda(table);
        if (value !== null) {
          values.push(value);
        }
      }
      const time = this.messages[ii].time;
      if (tables.length === 0) {
        results.push(new Point(time, NaN));
      } else {
        for (const valueVector of values) {
          for (const value of valueVector) {
            results.push(new Point(time, (value === null) ? NaN : value));
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
  private _lastNumMessages: number = 0;
  constructor(
      public readonly messages: MessageHandler, public readonly line: Line,
      public readonly field: string[]) {}
  hasUpdate(): boolean {
    const updated = this._lastNumMessages != this.messages.numMessages();
    this._lastNumMessages = this.messages.numMessages();
    return updated;
  }
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
  addMessageLine(message: MessageHandler|null, field: string[]): Line {
    // Construct the line regardless so that we have something to return to the
    // user.
    const line = this.plot.getDrawer().addLine();
    if (message === null) {
      console.warn(
          'Not plotting field ' + field.join('.') +
          ' because of an invalid MessageHandler.');
      return line;
    }
    line.setLabel(field.join('.'));
    this.lines.push(new MessageLine(message, line, field));
    return line;
  }

  draw(): void {
    // Only redraw lines if the number of points has changed--because getField()
    // is a relatively expensive call, we don't want to do it any more than
    // necessary.
    for (const line of this.lines) {
      if (line.hasUpdate()) {
        line.line.setPoints(line.messages.getField(line.field));
      }
    }
  }
}

export class AosPlotter {
  public static readonly TIME: string = "Monotonic Time (sec)";

  public static readonly DEFAULT_WIDTH: number = 900;
  public static readonly DEFAULT_HEIGHT: number = 400;

  private plots: AosPlot[] = [];
  private messages = new Set<MessageHandler>();
  constructor(private readonly connection: Connection) {
    // Set up to redraw at some regular interval. The exact rate is unimportant.
    setInterval(() => {
      this.draw();
    }, 100);
  }

  // Sets up an AOS channel as a message source. Returns a handler that can
  // be passed to addMessageLine().
  addMessageSource(name: string, type: string): MessageHandler|null {
    let schema = null;
    try {
      schema = this.connection.getSchema(type);
    } catch (e) {
      console.error(e);
      return null;
    }
    return this.addRawMessageSource(name, type, new MessageHandler(schema));
  }

  // Same as addMessageSource, but allows you to specify a custom MessageHandler
  // that does some processing on the requested message. This allows you to
  // create post-processed versions of individual channels.
  addRawMessageSource(
      name: string, type: string,
      messageHandler: MessageHandler|null): MessageHandler {
    if (messageHandler === null) {
      return null;
    }
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
      parentElement: Element,
      size: number[] = [AosPlotter.DEFAULT_WIDTH, AosPlotter.DEFAULT_HEIGHT]):
      AosPlot {
    const div = document.createElement("div");
    div.style.position = 'relative';
    div.style.width = size[0].toString() + "px";
    div.style.height = size[1].toString() + "px";
    parentElement.appendChild(div);
    const newPlot = new Plot(div);
    for (let plot of this.plots.values()) {
      newPlot.linkXAxis(plot.plot);
    }
    this.plots.push(new AosPlot(this, newPlot));
    return this.plots[this.plots.length - 1];
  }
  private draw(): void {
    for (const plot of this.plots) {
      plot.draw();
    }
  }
}
