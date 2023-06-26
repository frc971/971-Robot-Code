#include "aos/events/logging/file_operations.h"

#include "absl/strings/match.h"
#include "glog/logging.h"

namespace aos::logger::internal {

bool IsValidFilename(std::string_view filename) {
  return absl::EndsWith(filename, ".bfbs") ||
         absl::EndsWith(filename, ".bfbs.xz") ||
         absl::EndsWith(filename, ".bfbs.sz");
}

void LocalFileOperations::FindLogs(std::vector<File> *files) {
  auto MaybeAddFile = [&files](std::string_view filename, size_t size) {
    if (!IsValidFilename(filename)) {
      VLOG(1) << "Ignoring " << filename << " with invalid extension.";
    } else {
      VLOG(1) << "Found log " << filename;
      files->emplace_back(File{
          .name = std::string(filename),
          .size = size,
      });
    }
  };
  if (std::filesystem::is_directory(filename_)) {
    VLOG(1) << "Searching in " << filename_;
    for (const auto &file :
         std::filesystem::recursive_directory_iterator(filename_)) {
      if (!file.is_regular_file()) {
        VLOG(1) << file << " is not file.";
        continue;
      }
      MaybeAddFile(file.path().string(), file.file_size());
    }
  } else {
    MaybeAddFile(filename_, std::filesystem::file_size(filename_));
  }
}

}  // namespace aos::logger::internal
