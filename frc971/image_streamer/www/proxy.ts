import {Builder, ByteBuffer} from 'flatbuffers';
import {Payload, SdpType, WebSocketIce, WebSocketMessage, WebSocketSdp} from '../../../aos/network/web_proxy_generated';

// Port 9 is used to indicate an active (outgoing) TCP connection. The server
// would send a corresponding candidate with the actual TCP port it is
// listening on. Ignore NaN since it doesn't tell us anything about whether the
// port selected will have issues through the FMS firewall.
function validPort(port: number): boolean {
  return Number.isNaN(port) || port == 9 || (port >= 1180 && port <= 1190) ||
      (port >= 5800 && port <= 5810);
}

// Some browsers don't support the port property so provide our own function
// to get it.
function getIcePort(candidate: RTCIceCandidate): number {
  if (candidate.port === undefined) {
    return Number(candidate.candidate.split(' ')[5]);
  } else {
    return candidate.port;
  }
}

function isMdnsAddress(address: string): boolean {
  return address.includes('.local');
}

function getIceAddress(candidate: RTCIceCandidate): string {
  if (candidate.address === undefined) {
    return candidate.candidate.split(' ')[4];
  } else {
    return candidate.address;
  }
}

export class Connection {
  private webSocketConnection: WebSocket|null = null;
  private rtcPeerConnection: RTCPeerConnection|null = null;
  private html5VideoElement: HTMLMediaElement|null = null;
  private webSocketUrl: string;
  private statsInterval: number;

  private candidateNominatedId: string;
  private lastRtpTimestamp: number = 0;
  private lastBytesReceived: number = 0;
  private lastFramesDecoded: number = 0;


  constructor() {
    const server = location.host;
    this.webSocketUrl = `ws://${server}/ws`;

    for (let elem of document.getElementsByClassName('page_hostname')) {
      (elem as HTMLElement).innerText = location.hostname;
    }
  }

  connect(): void {
    this.html5VideoElement =
        (document.getElementById('stream') as HTMLMediaElement);

    this.webSocketConnection = new WebSocket(this.webSocketUrl);
    this.webSocketConnection.binaryType = 'arraybuffer';
    this.webSocketConnection.addEventListener(
        'message', (e) => this.onWebSocketMessage(e));
  }


  checkRemoteCandidate(candidate: RTCIceCandidate) {
    const port = getIcePort(candidate);
    if (!validPort(port)) {
      document.getElementById('bad_remote_port_port').innerText =
          port.toString();
      document.getElementById('bad_remote_port_error').style['display'] =
          'inherit';
    }
    const address = getIceAddress(candidate);
    if (isMdnsAddress(address)) {
      document.getElementById('bad_remote_address_address').innerText = address;
      document.getElementById('bad_remote_address_error').style['display'] =
          'inherit';
    }
  }

  checkLocalCandidate(candidate: RTCIceCandidate) {
    const port = getIcePort(candidate);
    if (!validPort(port)) {
      document.getElementById('bad_local_port_port').innerText =
          port.toString();
      document.getElementById('bad_local_port_error').style['display'] =
          'inherit';
    }
    const address = getIceAddress(candidate);
    if (isMdnsAddress(address)) {
      document.getElementById('bad_local_address_address').innerText = address;
      document.getElementById('bad_local_address_error').style['display'] =
          'inherit';
    }
  }

  onLocalDescription(desc: RTCSessionDescriptionInit): void {
    console.log('Local description: ' + JSON.stringify(desc));
    this.rtcPeerConnection.setLocalDescription(desc).then(() => {
      const builder = new Builder(512);
      const sdpFb = WebSocketSdp.createWebSocketSdp(
          builder, SdpType.ANSWER, builder.createString(desc.sdp));
      const message = WebSocketMessage.createWebSocketMessage(
          builder, Payload.WebSocketSdp, sdpFb);
      builder.finish(message);
      const array = builder.asUint8Array();

      this.webSocketConnection.send(array.buffer.slice(array.byteOffset));
    });
  }

  onIncomingSDP(sdp: RTCSessionDescriptionInit): void {
    console.log('Incoming SDP: ' + JSON.stringify(sdp));
    this.rtcPeerConnection.setRemoteDescription(sdp);
    this.rtcPeerConnection.createAnswer().then(
        (e) => this.onLocalDescription(e));
  }

  onIncomingICE(ice: RTCIceCandidateInit): void {
    let candidate = new RTCIceCandidate(ice);
    console.log('Incoming ICE: ' + JSON.stringify(ice));
    this.rtcPeerConnection.addIceCandidate(candidate);

    // If end of candidates, won't have a port.
    if (candidate.candidate !== '') {
      this.checkRemoteCandidate(candidate);
    }
  }

  onRequestStats(track: MediaStreamTrack): void {
    this.rtcPeerConnection.getStats(track).then((stats) => {
      // getStats returns a list of stats of various types in an implementation
      // defined order. We would like to get the protocol in use. This is found
      // in remote-candidate. However, (again, implementation defined), some
      // browsers return only remote-candidate's in use, while others return all
      // of them that attempted negotiation. To figure this out, look at the
      // currently nominated candidate-pair, then match up it's remote with a
      // remote-candidate we see later. Since the order isn't defined, store the
      // id in this in case the remote-candidate comes before candidate-pair.

      for (let dict of stats.values()) {
        if (dict.type === 'candidate-pair' && dict.nominated) {
          this.candidateNominatedId = dict.remoteCandidateId;
        }
        if (dict.type === 'remote-candidate' &&
            dict.id === this.candidateNominatedId) {
          document.getElementById('stats_protocol').innerText = dict.protocol;
        }
        if (dict.type === 'inbound-rtp') {
          const timestamp = dict.timestamp;
          const bytes_now = dict.bytesReceived;
          const frames_decoded = dict.framesDecoded;

          document.getElementById('stats_bps').innerText =
              Math.round(
                      (bytes_now - this.lastBytesReceived) * 8 /* bits */ /
                      1024 /* kbits */ / (timestamp - this.lastRtpTimestamp) *
                      1000 /* ms */)
                  .toString();

          document.getElementById('stats_fps').innerText =
              (Math.round(
                   (frames_decoded - this.lastFramesDecoded) /
                   (timestamp - this.lastRtpTimestamp) * 1000 /* ms */ * 10) /
               10).toString();


          this.lastRtpTimestamp = timestamp;
          this.lastBytesReceived = bytes_now;
          this.lastFramesDecoded = frames_decoded;
        }
      }
    });
  }

  onAddRemoteStream(event: RTCTrackEvent): void {
    const stream = event.streams[0];
    this.html5VideoElement.srcObject = stream;

    const track = stream.getTracks()[0];
    this.statsInterval =
        window.setInterval(() => this.onRequestStats(track), 1000);
  }

  onIceCandidate(event: RTCPeerConnectionIceEvent): void {
    if (event.candidate == null) {
      return;
    }

    console.log(
        'Sending ICE candidate out: ' + JSON.stringify(event.candidate));

    const builder = new Builder(512);
    const iceFb = WebSocketIce.createWebSocketIce(
        builder, builder.createString(event.candidate.candidate), null,
        event.candidate.sdpMLineIndex);
    const message = WebSocketMessage.createWebSocketMessage(
        builder, Payload.WebSocketIce, iceFb);
    builder.finish(message);
    const array = builder.asUint8Array();

    this.webSocketConnection.send(array.buffer.slice(array.byteOffset));

    // If end of candidates, won't have a port.
    if (event.candidate.candidate !== '') {
      this.checkLocalCandidate(event.candidate);
    }
  }

  // When we receive a websocket message, we need to determine what type it is
  // and handle appropriately. Either by setting the remote description or
  // adding the remote ice candidate.
  onWebSocketMessage(e: MessageEvent): void {
    const buffer = new Uint8Array(e.data)
    const fbBuffer = new ByteBuffer(buffer);
    const message = WebSocketMessage.getRootAsWebSocketMessage(fbBuffer);

    if (!this.rtcPeerConnection) {
      this.rtcPeerConnection = new RTCPeerConnection();
      this.rtcPeerConnection.ontrack = (e) => this.onAddRemoteStream(e);
      this.rtcPeerConnection.onicecandidate = (e) => this.onIceCandidate(e);
    }

    switch (message.payloadType()) {
      case Payload.WebSocketSdp:
        const sdpFb = message.payload(new WebSocketSdp());
        const sdp:
            RTCSessionDescriptionInit = {type: 'offer', sdp: sdpFb.payload()};

        this.onIncomingSDP(sdp);
        break;
      case Payload.WebSocketIce:
        const iceFb = message.payload(new WebSocketIce());
        const candidate = {} as RTCIceCandidateInit;
        candidate.candidate = iceFb.candidate();
        candidate.sdpMLineIndex = iceFb.sdpMLineIndex();
        candidate.sdpMid = iceFb.sdpMid();
        this.onIncomingICE(candidate);
        break;
      default:
        console.log('got an unknown message');
        break;
    }
  }
}
