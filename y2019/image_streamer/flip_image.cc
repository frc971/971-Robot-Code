#include "flip_image.h"

#define cimg_display 0
#define cimg_use_jpeg
#define cimg_plugin "plugins/jpeg_buffer.h"
#include "third_party/cimg/CImg.h"

void flip_image(const char *input, const int input_size, JOCTET *buffer,
                unsigned int *buffer_size, bool flip) {
  ::cimg_library::CImg<unsigned char> image;
  image.load_jpeg_buffer((JOCTET *)(input), input_size);
  if (flip) {
    image.rotate(90);
  } else {
    image.rotate(270);
  }

  image.save_jpeg_buffer(buffer, *buffer_size, 80);
}
