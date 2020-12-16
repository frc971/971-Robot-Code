import * as configuration from 'org_frc971/aos/configuration_generated';
import * as web_proxy from 'org_frc971/aos/network/web_proxy_generated';
import {Builder} from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';

import Configuration = configuration.aos.Configuration;
import MessageHeader = web_proxy.aos.web_proxy.MessageHeader;
import WebSocketIce = web_proxy.aos.web_proxy.WebSocketIce;
import WebSocketMessage = web_proxy.aos.web_proxy.WebSocketMessage;
import Payload = web_proxy.aos.web_proxy.Payload;
import WebSocketSdp = web_proxy.aos.web_proxy.WebSocketSdp;
import SdpType = web_proxy.aos.web_proxy.SdpType;

// There is one handler for each DataChannel, it maintains the state of
// multi-part messages and delegates to a callback when the message is fully
// assembled.
export class Handler {
  private dataBuffer: Uint8Array|null = null;
  private receivedMessageLength: number = 0;
  constructor(
      private readonly handlerFunc:
          (data: Uint8Array, sentTime: number) => void,
      private readonly channel: RTCDataChannel) {
    channel.addEventListener('message', (e) => this.handleMessage(e));
  }

  handleMessage(e: MessageEvent): void {
    const fbBuffer = new ByteBuffer(new Uint8Array(e.data));
    const messageHeader = MessageHeader.getRootAsMessageHeader(
        fbBuffer as unknown as flatbuffers.ByteBuffer);
    const time = messageHeader.monotonicSentTime().toFloat64() * 1e-9;
    // Short circuit if only one packet
    if (messageHeader.packetCount() === 1) {
      this.handlerFunc(messageHeader.dataArray(), time);
      return;
    }

    if (messageHeader.packetIndex() === 0) {
      this.dataBuffer = new Uint8Array(messageHeader.length());
      this.receivedMessageLength = 0;
    }
    if (!messageHeader.dataLength()) {
      return;
    }
    this.dataBuffer.set(
        messageHeader.dataArray(),
        this.receivedMessageLength);
    this.receivedMessageLength += messageHeader.dataLength();

    if (messageHeader.packetIndex() === messageHeader.packetCount() - 1) {
      this.handlerFunc(this.dataBuffer, time);
    }
  }
}

// Analogous to the Connection class in //aos/network/web_proxy.h. Because most
// of the apis are native in JS, it is much simpler.
export class Connection {
  private webSocketConnection: WebSocket|null = null;
  private rtcPeerConnection: RTCPeerConnection|null = null;
  private dataChannel: RTCDataChannel|null = null;
  private webSocketUrl: string;

  private configInternal: Configuration|null = null;
  // A set of functions that accept the config to handle.
  private readonly configHandlers = new Set<(config: Configuration) => void>();

  private readonly handlerFuncs =
      new Map<string, (data: Uint8Array, sentTime: number) => void>();
  private readonly handlers = new Set<Handler>();

  constructor() {
    const server = location.host;
    this.webSocketUrl = `ws://${server}/ws`;
  }

  addConfigHandler(handler: (config: Configuration) => void): void {
    this.configHandlers.add(handler);
  }

  /**
   * Add a handler for a specific message type. Until we need to handle
   * different channel names with the same type differently, this is good
   * enough.
   */
  addHandler(id: string, handler: (data: Uint8Array, sentTime: number) => void): void {
    this.handlerFuncs.set(id, handler);
  }

  connect(): void {
    this.webSocketConnection = new WebSocket(this.webSocketUrl);
    this.webSocketConnection.binaryType = 'arraybuffer';
    this.webSocketConnection.addEventListener(
        'open', () => this.onWebSocketOpen());
    this.webSocketConnection.addEventListener(
        'message', (e) => this.onWebSocketMessage(e));
  }

  getConfig() {
    return this.configInternal;
  }

  // Handle messages on the DataChannel. Handles the Configuration message as
  // all other messages are sent on specific DataChannels.
  onConfigMessage(data: Uint8Array): void {
    const fbBuffer = new ByteBuffer(data);
    this.configInternal = Configuration.getRootAsConfiguration(
        fbBuffer as unknown as flatbuffers.ByteBuffer);
    for (const handler of Array.from(this.configHandlers)) {
      handler(this.configInternal);
    }
  }

  onDataChannel(ev: RTCDataChannelEvent): void {
    const channel = ev.channel;
    const name = channel.label;
    const channelType = name.split('/').pop();
    const handlerFunc = this.handlerFuncs.get(channelType);
    this.handlers.add(new Handler(handlerFunc, channel));
  }

  onIceCandidate(e: RTCPeerConnectionIceEvent): void {
    if (!e.candidate) {
      return;
    }
    const candidate = e.candidate;
    const builder = new Builder(512);
    const candidateString = builder.createString(candidate.candidate);
    const sdpMidString = builder.createString(candidate.sdpMid);

    const iceFb = WebSocketIce.createWebSocketIce(
        builder as unknown as flatbuffers.Builder, candidateString,
        sdpMidString, candidate.sdpMLineIndex);
    const messageFb = WebSocketMessage.createWebSocketMessage(
        builder as unknown as flatbuffers.Builder, Payload.WebSocketIce, iceFb);
    builder.finish(messageFb);
    const array = builder.asUint8Array();
    this.webSocketConnection.send(array.buffer.slice(array.byteOffset));
  }

  // Called for new SDPs. Make sure to set it locally and remotely.
  onOfferCreated(description: RTCSessionDescriptionInit): void {
    this.rtcPeerConnection.setLocalDescription(description);
    const builder = new Builder(512);
    const offerString = builder.createString(description.sdp);

    const webSocketSdp = WebSocketSdp.createWebSocketSdp(
        builder as unknown as flatbuffers.Builder, SdpType.OFFER, offerString);
    const message = WebSocketMessage.createWebSocketMessage(
        builder as unknown as flatbuffers.Builder, Payload.WebSocketSdp,
        webSocketSdp);
    builder.finish(message);
    const array = builder.asUint8Array();
    this.webSocketConnection.send(array.buffer.slice(array.byteOffset));
  }

  // We now have a websocket, so start setting up the peer connection. We only
  // want a DataChannel, so create it and then create an offer to send.
  onWebSocketOpen(): void {
    this.rtcPeerConnection = new RTCPeerConnection({});
    this.rtcPeerConnection.addEventListener(
        'datachannel', (e) => this.onDataChannel(e));
    this.dataChannel = this.rtcPeerConnection.createDataChannel('signalling');
    this.handlers.add(
        new Handler((data) => this.onConfigMessage(data), this.dataChannel));
    // TODO(james): Is this used? Can we delete it?
    // window.dc = this.dataChannel;
    this.rtcPeerConnection.addEventListener(
        'icecandidate', (e) => this.onIceCandidate(e));
    this.rtcPeerConnection.createOffer().then(
        (offer) => this.onOfferCreated(offer));
  }

  // When we receive a websocket message, we need to determine what type it is
  // and handle appropriately. Either by setting the remote description or
  // adding the remote ice candidate.
  onWebSocketMessage(e: MessageEvent): void {
    const buffer = new Uint8Array(e.data)
    const fbBuffer = new ByteBuffer(buffer);
    const message = WebSocketMessage.getRootAsWebSocketMessage(
        fbBuffer as unknown as flatbuffers.ByteBuffer);
    switch (message.payloadType()) {
      case Payload.WebSocketSdp:
        const sdpFb = message.payload(new WebSocketSdp());
        if (sdpFb.type() !== SdpType.ANSWER) {
          console.log('got something other than an answer back');
          break;
        }
        this.rtcPeerConnection.setRemoteDescription(new RTCSessionDescription(
            {'type': 'answer', 'sdp': sdpFb.payload()}));
        break;
      case Payload.WebSocketIce:
        const iceFb = message.payload(new WebSocketIce());
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

  /**
   * Subscribes to messages. Only the most recent connect message is in use. Any
   * channels not specified in the message are implicitely unsubscribed.
   * @param a Finished flatbuffer.Builder containing a Connect message to send.
   */
  sendConnectMessage(builder: any) {
    const array = builder.asUint8Array();
    this.dataChannel.send(array.buffer.slice(array.byteOffset));
  }
}
