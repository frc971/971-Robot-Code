import {CameraImage} from 'y2020/vision/vision_generated';

export class ImageHandler {
  private canvas = document.createElement('canvas');

  constructor() {
    document.body.appendChild(this.canvas);
  }

  handleImage(data: Uint8Array) {
    const fbBuffer = new flatbuffers.ByteBuffer(data);
    const image = CameraImage.getRootAsCameraImage(fbBuffer);

    const width = image.cols();
    const height = image.rows();
    if (width === 0 || height === 0) {
      return;
    }
    const imageBuffer = new Uint8ClampedArray(width * height * 4); // RGBA

    // Read four bytes (YUYV) from the data and transform into two pixels of
    // RGBA for canvas
    for (const j = 0; j < height; j++) {
      for (const i = 0; i < width; i += 2) {
        const y1 = image.data((j * width + i) * 2);
        const u = image.data((j * width + i) * 2 + 1);
        const y2 = image.data((j * width + i + 1) * 2);
        const v = image.data((j * width + i + 1) * 2 + 1);

        // Based on https://en.wikipedia.org/wiki/YUV#Converting_between_Y%E2%80%B2UV_and_RGB
        const c1 = y1 - 16;
        const c2 = y2 - 16;
        const d = u - 128;
        const e = v - 128;

        imageBuffer[(j * width + i) * 4 + 0] = (298 * c1 + 409 * e + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 1] =
            (298 * c1 - 100 * d - 208 * e + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 2] = (298 * c1 + 516 * d + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 3] = 255;
        imageBuffer[(j * width + i) * 4 + 4] = (298 * c2 + 409 * e + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 5] =
            (298 * c2 - 100 * d - 208 * e + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 6] = (298 * c2 + 516 * d + 128) >> 8;
        imageBuffer[(j * width + i) * 4 + 7] = 255;
      }
    }

    const ctx = this.canvas.getContext('2d');

    this.canvas.width = width;
    this.canvas.height = height;
    const idata = ctx.createImageData(width, height);
    idata.data.set(imageBuffer);
    ctx.putImageData(idata, 0, 0);
  }

  getId() {
    return CameraImage.getFullyQualifiedName();
  }
}
