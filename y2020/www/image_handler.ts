import {CameraImage} from 'y2020/vision/vision_generated';
import {ImageMatchResult} from 'y2020/vision/sift/sift_generated'

export class ImageHandler {
  private canvas = document.createElement('canvas');
  private imageBuffer: Uint8ClampedArray|null = null;
  private imageTimestamp: flatbuffers.Long|null = null;
  private result: ImageMatchResult|null = null;
  private resultTimestamp: flatbuffers.Long|null = null;
  private width = 0;
  private height = 0;
  private imageSkipCount = 3;

  constructor() {
    document.body.appendChild(this.canvas);
  }

  handleImage(data: Uint8Array): void {
    if (this.imageSkipCount != 0) {
      this.imageSkipCount--;
      return;
    } else {
      this.imageSkipCount = 3;
    }

    const fbBuffer = new flatbuffers.ByteBuffer(data);
    const image = CameraImage.getRootAsCameraImage(fbBuffer);
    this.imageTimestamp = image.monotonicTimestampNs();

    this.width = image.cols();
    this.height = image.rows();
    if (this.width === 0 || this.height === 0) {
      return;
    }
    this.imageBuffer = new Uint8ClampedArray(this.width * this.height * 4); // RGBA

    // Read four bytes (YUYV) from the data and transform into two pixels of
    // RGBA for canvas
    for (const j = 0; j < this.height; j++) {
      for (const i = 0; i < this.width; i += 2) {
        const y1 = image.data((j * this.width + i) * 2);
        const u = image.data((j * this.width + i) * 2 + 1);
        const y2 = image.data((j * this.width + i + 1) * 2);
        const v = image.data((j * this.width + i + 1) * 2 + 1);

        // Based on https://en.wikipedia.org/wiki/YUV#Converting_between_Y%E2%80%B2UV_and_RGB
        const c1 = y1 - 16;
        const c2 = y2 - 16;
        const d = u - 128;
        const e = v - 128;

        this.imageBuffer[(j * this.width + i) * 4 + 0] = (298 * c1 + 409 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 1] = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 2] = (298 * c1 + 516 * d + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 3] = 255;
        this.imageBuffer[(j * this.width + i) * 4 + 4] = (298 * c2 + 409 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 5] = (298 * c2 - 100 * d - 208 * e + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 6] = (298 * c2 + 516 * d + 128) >> 8;
        this.imageBuffer[(j * this.width + i) * 4 + 7] = 255;
      }
    }

    this.draw();
  }

  handleImageMetadata(data: Uint8Array): void {
    const fbBuffer = new flatbuffers.ByteBuffer(data);
    this.result = ImageMatchResult.getRootAsImageMatchResult(fbBuffer);
    this.resultTimestamp = this.result.imageMonotonicTimestampNs();
    this.draw();
  }

  draw(): void {
  if (!this.imageTimestamp || !this.resultTimestamp ||
        this.imageTimestamp.low !== this.resultTimestamp.low ||
        this.imageTimestamp.high !== this.resultTimestamp.high) {
      return;
    }
    const ctx = this.canvas.getContext('2d');

    this.canvas.width = this.width;
    this.canvas.height = this.height;
    const idata = ctx.createImageData(this.width, this.height);
    idata.data.set(this.imageBuffer);
    ctx.putImageData(idata, 0, 0);
    for (const i = 0; i < this.result.featuresLength(); i++) {
      const feature = this.result.features(i);
      // Based on OpenCV drawKeypoint.
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

  getId(): string {
    return CameraImage.getFullyQualifiedName();
  }

  getResultId(): string {
    return ImageMatchResult.getFullyQualifiedName();
  }
}
