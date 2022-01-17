#include "flip_image.h"

#ifdef __clang__
// CImg has undefined behavior that Clang warns about. Just suppress the
// warnings, somebody should evaluate these more carefully if this code is used
// again.
#pragma clang diagnostic ignored "-Wvarargs"
#pragma clang diagnostic ignored "-Wnull-pointer-arithmetic"
#pragma clang diagnostic ignored "-Wchar-subscripts"
#pragma clang diagnostic ignored "-Wtautological-unsigned-char-zero-compare"
#endif

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
