import {aos} from 'org_frc971/aos/events/ping_generated';

export function HandlePing(data: Uint8Array) {
  const fbBuffer = new flatbuffers.ByteBuffer(data);
  const ping = aos.examples.Ping.getRootAsPing(fbBuffer);

  document.getElementById('val').innerHTML = ping.value().toString();
  document.getElementById('time').innerHTML = ping.sendTime().low.toString();
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
  return aos.examples.Ping.getFullyQualifiedName();
}
