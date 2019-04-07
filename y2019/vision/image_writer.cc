#include <fstream>
#include <sys/stat.h>

#include "glog/logging.h"
#include "y2019/vision/image_writer.h"

namespace y2019 {
namespace vision {

ImageWriter::ImageWriter() {
  LOG(INFO) <<  "Initializing image writer";
  SetDirPath();
}

void ImageWriter::WriteImage(::aos::vision::DataRef data) {
  LOG(INFO) << "Writing image " << image_count_;
  std::ofstream ofs(
      dir_path_ + file_prefix_ + std::to_string(image_count_) + ".yuyv",
      std::ofstream::out);
  ofs << data;
  ofs.close();
  ++image_count_;
}

void ImageWriter::SetDirPath() {
  ::std::string base_path = "/jevois/data/run_";
  for (int i = 0;; ++i) {
    struct stat st;
    std::string option = base_path + std::to_string(i);
    if (stat(option.c_str(), &st) != 0) {
      file_prefix_ = option + "/";
      LOG(INFO) << "Writing to " << file_prefix_.c_str();
      mkdir(file_prefix_.c_str(), 0777);
      break;
    }
  }
}

}  // namespace vision
}  // namespace y2019
