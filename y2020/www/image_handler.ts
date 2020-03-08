import {Channel} from 'aos/configuration_generated';
import {Connection} from 'aos/network/www/proxy';
import {Connect} from 'aos/network/connect_generated';
import {ImageMatchResult} from 'y2020/vision/sift/sift_generated'
import {CameraImage} from 'y2020/vision/vision_generated';

/*
 * All the messages that are required to show an image with metadata.
 * Messages not readable on the server node are ignored.
 */
const REQUIRED_CHANNELS = [
  {
    name: '/pi1/camera',
    type: CameraImage.getFullyQualifiedName(),
  },
  {
    name: '/pi2/camera',
    type: CameraImage.getFullyQualifiedName(),
  },
  {
    name: '/pi3/camera',
    type: CameraImage.getFullyQualifiedName(),
  },
  {
    name: '/pi4/camera',
    type: CameraImage.getFullyQualifiedName(),
  },
  {
    name: '/pi5/camera',
    type: CameraImage.getFullyQualifiedName(),
  },
  {
    name: '/pi1/camera/detailed',
    type: ImageMatchResult.getFullyQualifiedName(),
  },
  {
    name: '/pi2/camera/detailed',
    type: ImageMatchResult.getFullyQualifiedName(),
  },
  {
    name: '/pi3/camera/detailed',
    type: ImageMatchResult.getFullyQualifiedName(),
  },
  {
    name: '/pi4/camera/detailed',
    type: ImageMatchResult.getFullyQualifiedName(),
  },
  {
    name: '/pi5/camera/detailed',
    type: ImageMatchResult.getFullyQualifiedName(),
  },
];

export class ImageHandler {
  private canvas = document.createElement('canvas');
  private select = document.createElement('select');

  private imageBuffer: Uint8ClampedArray|null = null;
  private image: CameraImage|null = null;
  private imageTimestamp: flatbuffers.Long|null = null;
  private result: ImageMatchResult|null = null;
  private resultTimestamp: flatbuffers.Long|null = null;
  private width = 0;
  private height = 0;
  private selectedIndex = 0;
  private imageSkipCount = 3;

  constructor(private readonly connection: Connection) {
    document.body.appendChild(this.select);
    const defaultOption = document.createElement('option');
    defaultOption.innerText = 'Show all features';
    this.select.appendChild(defaultOption);
    this.select.addEventListener('change', (ev) => this.handleSelect(ev));
    document.body.appendChild(this.canvas);

    this.connection.addConfigHandler(() => {
      this.sendConnect();
    });
    this.connection.addHandler(ImageMatchResult.getFullyQualifiedName(), (data) => {
      this.handleImageMetadata(data);
    });
    this.connection.addHandler(CameraImage.getFullyQualifiedName(), (data) => {
      this.handleImage(data);
    });
  }

  private sendConnect(): void {
    const builder = new flatbuffers.Builder(512);
    const channels: flatbuffers.Offset[] = [];
    for (const channel of REQUIRED_CHANNELS) {
      const nameFb = builder.createString(channel.name);
      const typeFb = builder.createString(channel.type);
      Channel.startChannel(builder);
      Channel.addName(builder, nameFb);
      Channel.addType(builder, typeFb);
      const channelFb = Channel.endChannel(builder);
      channels.push(channelFb);
    }

    const channelsFb = Connect.createChannelsToTransferVector(builder, channels);
    Connect.startConnect(builder);
    Connect.addChannelsToTransfer(builder, channelsFb);
    const connect = Connect.endConnect(builder);
    builder.finish(connect);
    this.connection.sendConnectMessage(builder);
  }

  handleSelect(ev: Event) {
    this.selectedIndex = ev.target.selectedIndex;
  }

  handleImage(data: Uint8Array): void {
    console.log('got an image to process');
    if (this.imageSkipCount != 0) {
      this.imageSkipCount--;
      return;
    } else {
      this.imageSkipCount = 3;
    }

    const fbBuffer = new flatbuffers.ByteBuffer(data);
    this.image = CameraImage.getRootAsCameraImage(fbBuffer);
    this.imageTimestamp = this.image.monotonicTimestampNs();

    this.width = this.image.cols();
    this.height = this.image.rows();
    if (this.width === 0 || this.height === 0) {
      return;
    }

    this.draw();
  }

  convertImage(): void {
    this.imageBuffer = new Uint8ClampedArray(this.width * this.height * 4); // RGBA
    // Read four bytes (YUYV) from the data and transform into two pixels of
    // RGBA for canvas
    for (const j = 0; j < this.height; j++) {
      for (const i = 0; i < this.width; i += 2) {
        const y1 = this.image.data((j * this.width + i) * 2);
        const u = this.image.data((j * this.width + i) * 2 + 1);
        const y2 = this.image.data((j * this.width + i + 1) * 2);
        const v = this.image.data((j * this.width + i + 1) * 2 + 1);

        // Based on https://en.wikipedia.org/wiki/YUV#Converting_between_Y%E2%80%B2UV_and_RGB
        const c1 = y1 - 16;
        const c2 = y2 - 16;
        const d = u - 128;
        const e = v - 128;

        this.imageBuffer[(j * this.width + i) * 4 + 0] =
            (298 * c1 + 409 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 1] =
            (298 * c1 - 100 * d - 208 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 2] =
            (298 * c1 + 516 * d + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 3] = 255;
        this.imageBuffer[(j * this.width + i) * 4 + 4] =
            (298 * c2 + 409 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 5] =
            (298 * c2 - 100 * d - 208 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 6] =
            (298 * c2 + 516 * d + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 7] = 255;
      }
    }
  }

  handleImageMetadata(data: Uint8Array): void {
    console.log('got an image match result to process');
    const fbBuffer = new flatbuffers.ByteBuffer(data);
    this.result = ImageMatchResult.getRootAsImageMatchResult(fbBuffer);
    this.resultTimestamp = this.result.imageMonotonicTimestampNs();
    this.draw();
  }

  draw(): void {
    if (!this.imageTimestamp || !this.resultTimestamp ||
        this.imageTimestamp.low !== this.resultTimestamp.low ||
        this.imageTimestamp.high !== this.resultTimestamp.high) {
      console.log('image and result do not match');
      console.log(this.imageTimestamp.low, this.resultTimestamp.low);
      console.log(this.imageTimestamp.high, this.resultTimestamp.high);
      return;
    }
    this.convertImage();
    const ctx = this.canvas.getContext('2d');

    this.canvas.width = this.width;
    this.canvas.height = this.height;
    const idata = ctx.createImageData(this.width, this.height);
    idata.data.set(this.imageBuffer);
    ctx.putImageData(idata, 0, 0);
    console.log('features: ', this.result.featuresLength();
    if (this.selectedIndex === 0) {
      for (const i = 0; i < this.result.featuresLength(); i++) {
        const feature = this.result.features(i);
        this.drawFeature(feature);
      }
    } else {
      const imageMatch = this.result.imageMatches(this.selectedIndex - 1);
      for (const i = 0; i < imageMatch.matchesLength(); i++) {
        const featureIndex = imageMatch.matches(i).queryFeature();
        this.drawFeature(this.result.features(featureIndex));
      }
    }

    // Draw 'center' of target.
    // TODO(alex) adjust to new location in flatbuffer for these fields
    //ctx.strokeStyle = 'red';
    //ctx.beginPath();
    //ctx.arc(
    //    this.result.targetPointX(), this.result.targetPointY(), 20, 0,
    //    2 * Math.PI);
    //ctx.stroke();

    while (this.select.lastChild) {
      this.select.removeChild(this.select.lastChild);
    }
    const defaultOption = document.createElement('option');
    defaultOption.innerText = 'Show all features';
    defaultOption.setAttribute('value', 0);
    this.select.appendChild(defaultOption);
    for (const i = 0; i < this.result.imageMatchesLength(); i++) {
      const imageMatch = this.result.imageMatches(i);
      const option = document.createElement('option');
      option.setAttribute('value', i + 1);
      option.innerText =
          `Show image ${i} features (${imageMatch.matchesLength()})`;
      this.select.appendChild(option);
    }
    this.select.selectedIndex = this.selectedIndex;
  }

  // Based on OpenCV drawKeypoint.
  private drawFeature(feature: Feature) {
    const ctx = this.canvas.getContext('2d');
    ctx.beginPath();
    ctx.arc(feature.x(), feature.y(), feature.size(), 0, 2 * Math.PI);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(feature.x(), feature.y());
    const angle = feature.angle() * Math.PI / 180;
    ctx.lineTo(
        feature.x() + feature.size() * Math.cos(angle),
        feature.y() + feature.size() * Math.sin(angle));
    ctx.stroke();
  }
}
