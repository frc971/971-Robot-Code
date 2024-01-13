import {ByteBuffer} from 'flatbuffers'
import {ClientStatistics} from '../../aos/network/message_bridge_client_generated'
import {ServerStatistics, State as ConnectionState} from '../../aos/network/message_bridge_server_generated'
import {Connection} from '../../aos/network/www/proxy'
import {ZeroingError} from '../../frc971/control_loops/control_loops_generated'
import {Position as DrivetrainPosition} from '../../frc971/control_loops/drivetrain/drivetrain_position_generated'
import {CANPosition as DrivetrainCANPosition} from '../../frc971/control_loops/drivetrain/drivetrain_can_position_generated'
import {Status as DrivetrainStatus} from '../../frc971/control_loops/drivetrain/drivetrain_status_generated'
import {LocalizerOutput} from '../../frc971/control_loops/drivetrain/localization/localizer_output_generated'
import {TargetMap} from '../../frc971/vision/target_map_generated'


import {FIELD_LENGTH, FIELD_WIDTH, FT_TO_M, IN_TO_M} from './constants';

// (0,0) is field center, +X is toward red DS
const FIELD_SIDE_Y = FIELD_WIDTH / 2;
const FIELD_EDGE_X = FIELD_LENGTH / 2;

const ROBOT_WIDTH = 25 * IN_TO_M;
const ROBOT_LENGTH = 32 * IN_TO_M;

export class FieldHandler {
  constructor(private readonly connection: Connection) {
  }
}
