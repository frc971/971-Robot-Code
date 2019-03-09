#ifndef _Y2019_VISION_IMAGE_WRITER_H_
#define _Y2019_VISION_IMAGE_WRITER_H_

#include <string>

#include "aos/logging/logging.h"
#include "aos/vision/image/image_types.h"

namespace y2019 {
namespace vision {

class ImageWriter {
  public:
   ImageWriter() {
     LOG(INFO, "Initializing image writer\n");
     SetDirPath();
   }

   // This is destructive to data.
   void ProcessImage(::aos::vision::DataRef data, size_t num_targets);
  private:
   void SetDirPath();

   void WriteImage(::aos::vision::DataRef data);

   std::string file_prefix_ = std::string("debug_viewer_jpeg_");
   std::string dir_path_;

   size_t previous_num_targets_ = 0;
   char previous_image_[640 * 480 * 2];

   unsigned int image_count_ = 0;
   unsigned int debounce_count_ = 0;
};

}  // namespace vision
}  // namespace y2017

#endif  // _Y2019_VISION_IMAGE_WRITER_H_
