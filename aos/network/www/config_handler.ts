import {Configuration, Channel} from 'aos/configuration_generated';
import {Connect} from 'aos/network/connect_generated';

export class ConfigHandler {
  private readonly root_div = document.getElementById('config');

  constructor(
      private readonly config: Configuration,
      private readonly dataChannel: RTCDataChannel) {}

  printConfig() {
    for (const i = 0; i < this.config.channelsLength(); i++) {
      const channel_div = document.createElement('div');
      channel_div.classList.add('channel');
      this.root_div.appendChild(channel_div);

      const input_el = document.createElement('input');
      input_el.setAttribute('data-index', i);
      input_el.setAttribute('type', 'checkbox');
      input_el.addEventListener('click', () => this.handleChange());
      channel_div.appendChild(input_el);

      const name_div = document.createElement('div');
      const name_text = document.createTextNode(this.config.channels(i).name());
      name_div.appendChild(name_text);
      const type_div = document.createElement('div');
      const type_text = document.createTextNode(this.config.channels(i).type());
      type_div.appendChild(type_text);
      const info_div = document.createElement('div');
      info_div.appendChild(name_div);
      info_div.appendChild(type_div);

      channel_div.appendChild(info_div);
    }
  }

  handleChange() {
    const toggles = this.root_div.getElementsByTagName('input');
    const builder = new flatbuffers.Builder(512);

    const channels: flatbuffers.Offset[] = [];
    for (const toggle of toggles) {
      if (!toggle.checked) {
        continue;
      }
      const index = toggle.getAttribute('data-index');
      const channel = this.config.channels(index);
      const namefb = builder.createString(channel.name());
      const typefb = builder.createString(channel.type());
      Channel.startChannel(builder);
      Channel.addName(builder, namefb);
      Channel.addType(builder, typefb);
      const channelfb = Channel.endChannel(builder);
      channels.push(channelfb);
    }

    const channelsfb =
        Connect.createChannelsToTransferVector(
            builder, channels);
    Connect.startConnect(builder);
    Connect.addChannelsToTransfer(builder, channelsfb);
    const connect = Connect.endConnect(builder);
    builder.finish(connect);
    const array = builder.asUint8Array();
    console.log('connect', array);
    this.dataChannel.send(array.buffer.slice(array.byteOffset));
  }
}
