import {Connection} from '../../aos/network/www/proxy';

import {ImageHandler} from './image_handler';
import {ConfigHandler} from '../../aos/network/www/config_handler';

const conn = new Connection();

conn.connect();

const iHandler = new ImageHandler(conn);
const configHandler = new ConfigHandler(conn);
