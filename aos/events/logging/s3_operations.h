#ifndef AOS_EVENTS_LOGGING_S3_OPERATIONS_H_
#define AOS_EVENTS_LOGGING_S3_OPERATIONS_H_

#include "aos/events/logging/file_operations.h"

namespace aos::logger::internal {

class S3FileOperations final : public FileOperations {
 public:
  explicit S3FileOperations(std::string_view url);

  bool Exists() final { return !object_urls_.empty(); }

  void FindLogs(std::vector<std::string> *files) final;

 private:
  const std::vector<std::string> object_urls_;
};

}  // namespace aos::logger::internal

#endif  // AOS_EVENTS_LOGGING_S3_OPERATIONS_H_
