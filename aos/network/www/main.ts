import {Connection} from './proxy';
import * as PingHandler from './ping_handler';

const conn = new Connection();

conn.connect();

PingHandler.SetupDom();

conn.addHandler('/aos', PingHandler.GetId(), PingHandler.HandlePing);
