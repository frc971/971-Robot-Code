#ifndef _AOS_VISION_IMAGE_JPEGROUTINES_H_
#define _AOS_VISION_IMAGE_JPEGROUTINES_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

// Returns true if successful false if an error was encountered.
// Will decompress data into out. Out must be of the right size
// as determined below.
bool ProcessJpeg(DataRef data, PixelRef *out);

// Gets the format for the particular jpeg.
ImageFormat GetFmt(DataRef data);

// Decodes jpeg from data. Will resize if necessary.
// (Should not be necessary in most normal cases).
//
// Consider this the canonical way to decode jpegs if no other
// choice is given.
inline bool DecodeJpeg(DataRef data, ImageValue *value) {
  auto fmt = GetFmt(data);
  if (!value->fmt().Equals(fmt)) {
    *value = ImageValue(fmt);
  }
  return ProcessJpeg(data, value->data());
}

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_IMAGE_JPEGROUTINES_H_
