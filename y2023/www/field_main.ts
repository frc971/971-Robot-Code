import {Connection} from 'org_frc971/aos/network/www/proxy';

import {FieldHandler} from './field_handler';

const conn = new Connection();

conn.connect();

const fieldHandler = new FieldHandler(conn);

fieldHandler.draw();

