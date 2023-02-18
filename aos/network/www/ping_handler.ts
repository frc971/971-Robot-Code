import {Ping} from '../../events/ping_generated';
import {ByteBuffer} from 'flatbuffers';

export function HandlePing(data: Uint8Array) {
  const fbBuffer = new ByteBuffer(data);
  const ping = Ping.getRootAsPing(fbBuffer);

  document.getElementById('val').innerHTML = ping.value().toString();
  document.getElementById('time').innerHTML = ping.sendTime().toString();
}

export function SetupDom() {
  const ping_div = document.createElement('div');
  document.body.appendChild(ping_div);

  const value_div = document.createElement('div');
  value_div.setAttribute('id', 'val');
  const time_div = document.createElement('div');
  time_div.setAttribute('id', 'time');

  ping_div.appendChild(value_div);
  ping_div.appendChild(time_div);
}

export function GetId() {
  return Ping.getFullyQualifiedName();
}
