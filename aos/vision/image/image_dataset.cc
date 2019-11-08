#include "aos/vision/image/image_dataset.h"

#include <fstream>

#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

namespace {
std::string GetFileContents(const std::string &filename) {
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in) {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return (contents);
  }
  fprintf(stderr, "Could not read file: %s\n", filename.c_str());
  exit(-1);
}

std::vector<std::string> Split(DataRef inp, char delim) {
  size_t i = 0;
  std::vector<size_t> pos;
  while (i < inp.size()) {
    i = inp.find(delim, i);
    if (i == std::string::npos) break;
    // fprintf(stderr, "k=%d, i=%d\n", k, (int)i);
    pos.emplace_back(i);
    i = i + 1;
  }
  std::vector<std::string> res;
  res.reserve(pos.size() + 1);
  i = 0;
  for (auto p : pos) {
    res.emplace_back(inp.substr(i, p - i));
    i = p + 1;
  }
  res.emplace_back(inp.substr(i));
  return res;
}
}  // namespace

DatasetFrame LoadFile(const std::string &jpeg_filename) {
  bool is_jpeg = true;
  size_t l = jpeg_filename.size();
  if (l > 4 && jpeg_filename[l - 1] == 'v') {
    is_jpeg = false;
  }
  return DatasetFrame{is_jpeg, GetFileContents(jpeg_filename)};
}

std::vector<DatasetFrame> LoadDataset(const std::string &jpeg_list_filename) {
  std::vector<DatasetFrame> images;
  auto contents = GetFileContents(jpeg_list_filename);

  std::string basename;
  auto it = jpeg_list_filename.find_last_of('/');
  if (it != std::string::npos) {
    basename = jpeg_list_filename.substr(0, it + 1);
  }

  for (const auto &jpeg_filename : Split(contents, '\n')) {
    [&]() {
      if (jpeg_filename.empty()) return;
      for (std::size_t i = 0; i < jpeg_filename.size(); ++i) {
        if (jpeg_filename[i] == '#') return;
        if (jpeg_filename[i] != ' ') break;
      }
      if (jpeg_filename[0] == '/') {
        images.emplace_back(LoadFile(jpeg_filename));
      } else {
        images.emplace_back(LoadFile(basename + jpeg_filename));
      }
    }();
  }

  return images;
}

}  // namespace vision
}  // namespace aos
