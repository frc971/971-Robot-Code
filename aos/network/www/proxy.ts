import {aos.web_proxy} from '../web_proxy_generated';

// Analogous to the Connection class in //aos/network/web_proxy.h. Because most
// of the apis are native in JS, it is much simpler.
export class Connection {
  private webSocketConnection: WebSocket|null = null;
  private rtcPeerConnection: RTCPeerConnection|null = null;
  private dataChannel: DataChannel|null = null;
  private webSocketUrl: string;

  constructor() {
    const server = location.host;
    this.webSocketUrl = `ws://${server}/ws`;
  }

  connect(): void {
    this.webSocketConnection = new WebSocket(this.webSocketUrl);
    this.webSocketConnection.binaryType = 'arraybuffer';
    this.webSocketConnection.addEventListener(
        'open', () => this.onWebSocketOpen());
    this.webSocketConnection.addEventListener(
        'message', (e) => this.onWebSocketMessage(e));
  }

  // Handle messages on the DataChannel. Will delegate to various handlers for
  // different message types.
  onDataChannelMessage(e: MessageEvent): void {
    console.log(e);
  }

  onIceCandidate(e: RTCPeerConnectionIceEvent): void {
    console.log('Created ice candidate', e);
    if (!e.candidate) {
      return;
    }
    const candidate = e.candidate;
    const builder = new flatbuffers.Builder(512);
    const candidateString = builder.createString(candidate.candidate);
    const sdpMidString = builder.createString(candidate.sdpMid);

    const iceFb = aos.web_proxy.WebSocketIce.createWebSocketIce(
        builder, candidateString, sdpMidString, candidate.sdpMLineIndex);
    const messageFb = aos.web_proxy.WebSocketMessage.createWebSocketMessage(
        builder, aos.web_proxy.Payload.WebSocketIce, iceFb);
    builder.finish(messageFb);
    const array = builder.asUint8Array();
    this.webSocketConnection.send(array.buffer.slice(array.byteOffset));
  }

  // Called for new SDPs. Make sure to set it locally and remotely.
  onOfferCreated(description: RTCSessionDescription): void {
    console.log('Created offer', description);
    this.rtcPeerConnection.setLocalDescription(description);
    const builder = new flatbuffers.Builder(512);
    const offerString = builder.createString(description.sdp);

    const webSocketSdp = aos.web_proxy.WebSocketSdp.createWebSocketSdp(
        builder, aos.web_proxy.SdpType.OFFER, offerString);
    const message = aos.web_proxy.WebSocketMessage.createWebSocketMessage(
        builder, aos.web_proxy.Payload.WebSocketSdp, webSocketSdp);
    builder.finish(message);
    const array = builder.asUint8Array();
    this.webSocketConnection.send(array.buffer.slice(array.byteOffset));
  }

  // We now have a websocket, so start setting up the peer connection. We only
  // want a DataChannel, so create it and then create an offer to send.
  onWebSocketOpen(): void {
    this.rtcPeerConnection = new RTCPeerConnection({});
    this.dataChannel = this.rtcPeerConnection.createDataChannel('dc');
    this.dataChannel.addEventListener(
        'message', (e) => this.onDataChannelMessage(e));
    window.dc = this.dataChannel;
    this.rtcPeerConnection.addEventListener(
        'icecandidate', (e) => this.onIceCandidate(e));
    this.rtcPeerConnection.createOffer().then(
        (offer) => this.onOfferCreated(offer));
  }

  // When we receive a websocket message, we need to determine what type it is
  // and handle appropriately. Either by setting the remote description or
  // adding the remote ice candidate.
  onWebSocketMessage(e: MessageEvent): void {
    console.log('ws: ', e);
    const buffer = new Uint8Array(e.data)
    const fbBuffer = new flatbuffers.ByteBuffer(buffer);
    const message =
        aos.web_proxy.WebSocketMessage.getRootAsWebSocketMessage(fbBuffer);
    switch (message.payloadType()) {
      case aos.web_proxy.Payload.WebSocketSdp:
        console.log('got an sdp message');
        const sdpFb = message.payload(new aos.web_proxy.WebSocketSdp());
        if (sdpFb.type() !== aos.web_proxy.SdpType.ANSWER) {
          console.log('got something other than an answer back');
          break;
        }
        this.rtcPeerConnection.setRemoteDescription(new RTCSessionDescription(
            {'type': 'answer', 'sdp': sdpFb.payload()}));
        break;
      case aos.web_proxy.Payload.WebSocketIce:
        console.log('got an ice message');
        const iceFb = message.payload(new aos.web_proxy.WebSocketIce());
        const candidate = {} as RTCIceCandidateInit;
        candidate.candidate = iceFb.candidate();
        candidate.sdpMid = iceFb.sdpMid();
        candidate.sdpMLineIndex = iceFb.sdpMLineIndex();
        this.rtcPeerConnection.addIceCandidate(candidate);
        break;
      default:
        console.log('got an unknown message');
        break;
    }
  }
}
