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

export class ImuMessageHandler extends MessageHandler {
  constructor(private readonly schema: Schema) {
    super(schema);
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
      this.messages.push(new TimestampedMessage(
          table, message.monotonicTimestampNs().toFloat64() * 1e-9));
    }
  }
}
