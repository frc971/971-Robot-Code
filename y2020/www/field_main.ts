import {Connection} from 'aos/network/www/proxy';

import {FieldHandler} from './field_handler';

const conn = new Connection();

conn.connect();

const fieldHandler = new FieldHandler(conn);

fieldHandler.draw();

