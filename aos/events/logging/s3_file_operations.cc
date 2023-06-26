#include "aos/events/logging/s3_file_operations.h"

#include "aos/events/logging/s3_fetcher.h"

namespace aos::logger::internal {

std::vector<FileOperations::File> Convert(
    std::vector<std::pair<std::string, size_t>> &&input) {
  std::vector<FileOperations::File> result;
  result.reserve(input.size());
  for (std::pair<std::string, size_t> &i : input) {
    result.emplace_back(FileOperations::File{
        .name = std::move(i.first),
        .size = i.second,
    });
  }
  return result;
}

S3FileOperations::S3FileOperations(std::string_view url)
    : object_urls_(Convert(ListS3Objects(url))) {}

void S3FileOperations::FindLogs(std::vector<File> *files) {
  // We already have a recursive listing, so just grab all the objects from
  // there.
  for (const File &object_url : object_urls_) {
    if (IsValidFilename(object_url.name)) {
      files->push_back(object_url);
    }
  }
}

}  // namespace aos::logger::internal
