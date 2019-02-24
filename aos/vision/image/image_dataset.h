#ifndef _AOS_VISION_IMAGE_IMAGE_DATASET_H_
#define _AOS_VISION_IMAGE_IMAGE_DATASET_H_

#include <string>
#include <vector>

namespace aos {
namespace vision {

struct DatasetFrame {
  // TODO: These should be V4L formats ideally.
  bool is_jpeg = true;
  std::string data;
};

std::vector<DatasetFrame> LoadDataset(const std::string &jpeg_list_filename);

DatasetFrame LoadFile(const std::string &jpeg_filename);

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_IMAGE_IMAGE_DATASET_H_
