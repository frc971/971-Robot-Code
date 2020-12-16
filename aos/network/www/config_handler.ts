import * as configuration from 'org_frc971/aos/configuration_generated';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import * as flatbuffers_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import * as web_proxy from 'org_frc971/aos/network/web_proxy_generated';


import Configuration = configuration.aos.Configuration;
import Channel = configuration.aos.Channel;
import SubscriberRequest = web_proxy.aos.web_proxy.SubscriberRequest;
import ChannelRequest = web_proxy.aos.web_proxy.ChannelRequest;
import TransferMethod = web_proxy.aos.web_proxy.TransferMethod;

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
      ChannelRequest.startChannelRequest(builder);
      ChannelRequest.addChannel(builder, channelfb);
      ChannelRequest.addMethod(builder, TransferMethod.SUBSAMPLE);
      channels.push(ChannelRequest.endChannelRequest(builder));
    }

    const channelsfb =
        SubscriberRequest.createChannelsToTransferVector(builder, channels);
    SubscriberRequest.startSubscriberRequest(builder);
    SubscriberRequest.addChannelsToTransfer(builder, channelsfb);
    const request = SubscriberRequest.endSubscriberRequest(builder);
    builder.finish(request);
    this.connection.sendConnectMessage(builder);
  }

  toggleConfig() {
    this.tree_div.hidden = !this.tree_div.hidden;
  }
}
