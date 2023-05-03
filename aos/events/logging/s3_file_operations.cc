#include "aos/events/logging/s3_file_operations.h"

#include "aos/events/logging/s3_fetcher.h"

namespace aos::logger::internal {

S3FileOperations::S3FileOperations(std::string_view url)
    : object_urls_(ListS3Objects(url)) {}

void S3FileOperations::FindLogs(std::vector<std::string> *files) {
  // We already have a recursive listing, so just grab all the objects from
  // there.
  for (const std::string &object_url : object_urls_) {
    if (IsValidFilename(object_url)) {
      files->push_back(object_url);
    }
  }
}

}  // namespace aos::logger::internal