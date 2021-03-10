// This script provides a basic utility for de-batching the IMUValues
// message. See imu_plotter.ts for usage.
import * as configuration from 'org_frc971/aos/configuration_generated';
import * as imu from 'org_frc971/frc971/wpilib/imu_batch_generated';
import {MessageHandler, TimestampedMessage} from 'org_frc971/aos/network/www/aos_plotter';
import {Table} from 'org_frc971/aos/network/www/reflection';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';

import Schema = configuration.reflection.Schema;
import IMUValuesBatch = imu.frc971.IMUValuesBatch;
import IMUValues = imu.frc971.IMUValues;

const FILTER_WINDOW_SIZE = 100;

export class ImuMessageHandler extends MessageHandler {
  // Calculated magnitude of the measured acceleration from the IMU.
  private acceleration_magnitudes: number[] = [];
  constructor(private readonly schema: Schema) {
    super(schema);
  }
  private readScalar(table: Table, fieldName: string): number {
    return this.parser.readScalar(table, fieldName);
  }
  addMessage(data: Uint8Array, time: number): void {
    const batch = IMUValuesBatch.getRootAsIMUValuesBatch(
        new ByteBuffer(data) as unknown as flatbuffers.ByteBuffer);
    for (let ii = 0; ii < batch.readingsLength(); ++ii) {
      const message = batch.readings(ii);
      const table = Table.getNamedTable(
          message.bb as unknown as ByteBuffer, this.schema, 'frc971.IMUValues',
          message.bb_pos);
      if (this.parser.readScalar(table, "monotonic_timestamp_ns") == null) {
        console.log('Ignoring unpopulated IMU values: ');
        console.log(this.parser.toObject(table));
        continue;
      }
      const time = message.monotonicTimestampNs().toFloat64() * 1e-9;
      this.messages.push(new TimestampedMessage(table, time));
      this.acceleration_magnitudes.push(time);
      this.acceleration_magnitudes.push(Math.hypot(
          message.accelerometerX(), message.accelerometerY(),
          message.accelerometerZ()));
    }
  }

  // Computes a moving average for a given input, using a basic window centered
  // on each value.
  private movingAverageCentered(input: Float32Array): Float32Array {
    const num_measurements = input.length / 2;
    const filtered_measurements = new Float32Array(input);
    for (let ii = 0; ii < num_measurements; ++ii) {
      let sum = 0;
      let count = 0;
      for (let jj = Math.max(0, Math.ceil(ii - FILTER_WINDOW_SIZE / 2));
           jj < Math.min(num_measurements, ii + FILTER_WINDOW_SIZE / 2); ++jj) {
        sum += input[jj * 2 + 1];
        ++count;
      }
      filtered_measurements[ii * 2 + 1] = sum / count;
    }
    return new Float32Array(filtered_measurements);
  }

  getField(field: string[]): Float32Array {
    // Any requested input that ends with "_filtered" will get a moving average
    // applied to the original field.
    const filtered_suffix = "_filtered";
    if (field[0] == "acceleration_magnitude") {
      return new Float32Array(this.acceleration_magnitudes);
    } else if (field[0].endsWith(filtered_suffix)) {
      return this.movingAverageCentered(this.getField(
          [field[0].slice(0, field[0].length - filtered_suffix.length)]));
    } else {
      return super.getField(field);
    }
  }
}
