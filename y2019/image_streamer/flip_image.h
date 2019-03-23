#ifndef Y2019_IMAGE_STREAMER_FLIP_IMAGE_H_
#define Y2019_IMAGE_STREAMER_FLIP_IMAGE_H_

#include <stddef.h>
#include <stdio.h>
#include "third_party/libjpeg/jerror.h"
#include "third_party/libjpeg/jpeglib.h"

void flip_image(const char *input, const int input_size, JOCTET *buffer,
                unsigned int *buffer_size, bool flip_image);

#endif  // Y2019_IMAGE_STREAMER_FLIP_IMAGE_H
