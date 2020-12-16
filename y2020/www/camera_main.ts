import {Connection} from 'org_frc971/aos/network/www/proxy';

import {ImageHandler} from './image_handler';

const conn = new Connection();

conn.connect();

const iHandler = new ImageHandler(conn);
