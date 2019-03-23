#include <fstream>
#include <sys/stat.h>

#include "y2019/vision/image_writer.h"

namespace y2019 {
namespace vision {

void ImageWriter::WriteImage(::aos::vision::DataRef data) {
  LOG(INFO, "Writing image %d", image_count_);
  std::ofstream ofs(
      dir_path_ + file_prefix_ + std::to_string(image_count_) + ".yuyv",
      std::ofstream::out);
  ofs << data;
  ofs.close();
  ++image_count_;
}

void ImageWriter::ProcessImage(::aos::vision::DataRef data,
                                size_t num_targets) {
  ++debounce_count_;
  if (debounce_count_ < 10) {
    return;
  }
  // Write the image if there are fewer targets in this frame than the last.
  if (num_targets < previous_num_targets_) {
    WriteImage(previous_image_);
    WriteImage(data);
    debounce_count_ = 0;
  }
  //data.swap(previous_image_);
  data.copy(previous_image_, sizeof(previous_image_));
}

void ImageWriter::SetDirPath() {
  std::string base_path = "/jevois/data/run_";
  for (int i = 0;; ++i) {
    struct stat st;
    std::string option = base_path + std::to_string(i);
    if (stat(option.c_str(), &st) != 0) {
      file_prefix_ = option + "/";
      LOG(INFO, "Writing to %s\n", file_prefix_.c_str());
      mkdir(file_prefix_.c_str(), 0777);
      break;
    }
  }
}

}  // namespace vision
}  // namespace y2019
