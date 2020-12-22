import {Connection} from 'org_frc971/aos/network/www/proxy';

import {ImageHandler} from './image_handler';
import {ConfigHandler} from 'org_frc971/aos/network/www/config_handler';

const conn = new Connection();

conn.connect();

const iHandler = new ImageHandler(conn);
const configHandler = new ConfigHandler(conn);
