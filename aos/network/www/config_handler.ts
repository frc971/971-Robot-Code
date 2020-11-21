import * as configuration from 'org_frc971/aos/configuration_generated';
import * as connect from 'org_frc971/aos/network/connect_generated';
import * as flatbuffers_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';

import {Connection} from './proxy';

import Configuration = configuration.aos.Configuration;
import Channel = configuration.aos.Channel;
import Connect = connect.aos.message_bridge.Connect;

export class ConfigHandler {
  private readonly root_div = document.createElement('div');
  private readonly tree_div;
  private config: Configuration|null = null

  constructor(private readonly connection: Connection) {
    this.connection.addConfigHandler((config) => this.handleConfig(config));

    document.body.appendChild(this.root_div);
    const show_button = document.createElement('button');
    show_button.addEventListener('click', () => this.toggleConfig());
    const show_text = document.createTextNode('Show/Hide Config');
    show_button.appendChild(show_text);
    this.tree_div = document.createElement('div');
    this.tree_div.hidden = true;
    this.root_div.appendChild(show_button);
    this.root_div.appendChild(this.tree_div);
  }

  handleConfig(config: Configuration) {
    this.config = config;
    this.printConfig();
  }

  printConfig() {
    for (let i = 0; i < this.config.channelsLength(); i++) {
      const channel_div = document.createElement('div');
      channel_div.classList.add('channel');
      this.tree_div.appendChild(channel_div);

      const input_el = document.createElement('input');
      input_el.setAttribute('data-index', i.toString());
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
    const builder =
        new flatbuffers_builder.Builder(512) as unknown as flatbuffers.Builder;

    const channels: flatbuffers.Offset[] = [];
    for (const toggle of toggles) {
      if (!toggle.checked) {
        continue;
      }
      const index = toggle.getAttribute('data-index');
      const channel = this.config.channels(Number(index));
      const namefb = builder.createString(channel.name());
      const typefb = builder.createString(channel.type());
      Channel.startChannel(builder);
      Channel.addName(builder, namefb);
      Channel.addType(builder, typefb);
      const channelfb = Channel.endChannel(builder);
      channels.push(channelfb);
    }

    const channelsfb =
        Connect.createChannelsToTransferVector(builder, channels);
    Connect.startConnect(builder);
    Connect.addChannelsToTransfer(builder, channelsfb);
    const connect = Connect.endConnect(builder);
    builder.finish(connect);
    this.connection.sendConnectMessage(builder);
  }

  toggleConfig() {
    this.tree_div.hidden = !this.tree_div.hidden;
  }
}
