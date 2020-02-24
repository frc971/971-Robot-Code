import {Connection} from 'aos/network/www/proxy';

import {ImageHandler} from './image_handler';

const conn = new Connection();

conn.connect();

const iHandler = new ImageHandler();

conn.addHandler(iHandler.getId(), (data) => iHandler.handleImage(data));
conn.addHandler(
    iHandler.getResultId(), (data) => iHandler.handleImageMetadata(data));
