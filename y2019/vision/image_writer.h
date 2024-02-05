#ifndef Y2019_VISION_IMAGE_WRITER_H_
#define Y2019_VISION_IMAGE_WRITER_H_

#include <string>

#include "aos/vision/image/image_types.h"

namespace y2019::vision {

class ImageWriter {
 public:
  ImageWriter();

  void WriteImage(::aos::vision::DataRef data);

 private:
  void SetDirPath();

  ::std::string file_prefix_ = std::string("debug_viewer_jpeg_");
  ::std::string dir_path_;

  unsigned int image_count_ = 0;
};

}  // namespace y2019::vision

#endif  // Y2019_VISION_IMAGE_WRITER_H_
