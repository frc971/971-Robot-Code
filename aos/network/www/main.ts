import {Connection} from './proxy';
import * as PingHandler from './ping_handler';

const conn = new Connection();

conn.connect();

PingHandler.SetupDom();

conn.addHandler(PingHandler.GetId(), PingHandler.HandlePing);
