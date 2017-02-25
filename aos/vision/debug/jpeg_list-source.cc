#include "aos/vision/debug/debug_framework.h"

#include <gdk/gdk.h>
#include <fstream>
#include <string>

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
    res.emplace_back(inp.substr(i, p - i).to_string());
    i = p + 1;
  }
  res.emplace_back(inp.substr(i).to_string());
  return res;
}
}  // namespace

class JpegListImageSource : public ImageSource {
 public:
  void Init(const std::string &jpeg_list_filename,
            DebugFrameworkInterface *interface) override {
    interface_ = interface;
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
          images_.emplace_back(GetFileContents(jpeg_filename));
        } else {
          images_.emplace_back(GetFileContents(basename + jpeg_filename));
        }
      }();
    }
    fprintf(stderr, "loaded %lu items\n", images_.size());
    if (!images_.empty()) {
      interface_->NewJpeg(images_[idx_]);
      interface_->InstallKeyPress([this](uint32_t keyval) {
        if (keyval == GDK_KEY_Left && idx_ > 0) {
          --idx_;
        } else if (keyval == GDK_KEY_Right && idx_ < images_.size()) {
          idx_ = (idx_ + 1) % images_.size();
        } else {
          return;
        }
        interface_->NewJpeg(images_[idx_]);
      });
    }
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is the name of a file with each jpeg filename on a new line.
    This viewer source will load each jpeg individually and cycle through them
    with the arrow keys.
)"[1];
  }

 private:
  DebugFrameworkInterface *interface_ = nullptr;
  std::vector<std::string> images_;
  size_t idx_ = 0;
};

REGISTER_IMAGE_SOURCE("jpeg_list", JpegListImageSource);

}  // namespace vision
}  // namespace aos
