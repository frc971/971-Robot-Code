import {Connection} from '../../aos/network/www/proxy';

import {StarterHandler} from './starter_handler';

const conn = new Connection();

conn.connect();

const starterHandler = new StarterHandler(conn);

starterHandler.draw();
