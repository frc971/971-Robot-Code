#ifndef AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_
#define AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_

#include <filesystem>
#include <string>
#include <vector>

namespace aos::logger::internal {

// Predicate to include or exclude file to be considered as a log file.
bool IsValidFilename(std::string_view filename);

// Abstraction that supports listing of the logs on file system and S3. It is
// associated with either a single file or directory that contains log files.
class FileOperations {
 public:
  virtual ~FileOperations() = default;

  virtual bool Exists() = 0;
  virtual void FindLogs(std::vector<std::string> *files) = 0;
};

// Implements FileOperations with standard POSIX filesystem APIs. These work on
// files local to the machine they're running on.
class LocalFileOperations final : public FileOperations {
 public:
  explicit LocalFileOperations(std::string_view filename)
      : filename_(filename) {}

  bool Exists() override { return std::filesystem::exists(filename_); }

  void FindLogs(std::vector<std::string> *files) override;

 private:
  std::string filename_;
};

}  // namespace aos::logger::internal

#endif  // AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_
