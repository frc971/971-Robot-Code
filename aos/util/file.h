#ifndef AOS_UTIL_FILE_H_
#define AOS_UTIL_FILE_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <memory>
#include <string>
#include <string_view>

#include "absl/strings/numbers.h"
#include "absl/types/span.h"
#include "aos/scoped/scoped_fd.h"
#include "glog/logging.h"

namespace aos {
namespace util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
::std::string ReadFileToStringOrDie(const std::string_view filename);

// Creates filename if it doesn't exist and sets the contents to contents.
void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents,
                            mode_t permissions = S_IRWXU);

// Returns true if it succeeds or false if the filesystem is full.
bool MkdirPIfSpace(std::string_view path, mode_t mode);

inline void MkdirP(std::string_view path, mode_t mode) {
  CHECK(MkdirPIfSpace(path, mode));
}

bool PathExists(std::string_view path);

// Recursively removes everything in the provided path.  Ignores any errors it
// runs across.
void UnlinkRecursive(std::string_view path);

enum class FileOptions { kReadable, kWriteable };

// Maps file from disk into memory
std::shared_ptr<absl::Span<uint8_t>> MMapFile(
    const std::string &path, FileOptions options = FileOptions::kReadable);

// Wrapper to handle reading the contents of a file into a buffer. Meant for
// situations where the malloc'ing of ReadFileToStringOrDie is inappropriate,
// but where you still want to read a file.
template <int kBufferSize = 1024>
class FileReader {
 public:
  FileReader(std::string_view filename)
      : file_(open(::std::string(filename).c_str(), O_RDONLY)) {
    PCHECK(file_.get() != -1) << ": opening " << filename;
    memset(buffer_, 0, kBufferSize);
  }
  // Reads the entire contents of the file into the internal buffer and returns
  // a string_view of it.
  // Note: The result may not be null-terminated.
  std::string_view ReadContents() {
    PCHECK(0 == lseek(file_.get(), 0, SEEK_SET));
    const ssize_t result = read(file_.get(), buffer_, sizeof(buffer_));
    PCHECK(result >= 0);
    return {buffer_, static_cast<size_t>(result)};
  }
  // Calls ReadContents() and attempts to convert the result into an integer, or
  // dies trying.
  int ReadInt() {
    int result;
    std::string_view contents = ReadContents();
    CHECK(absl::SimpleAtoi(contents, &result))
        << "Failed to parse \"" << contents << "\" as int.";
    return result;
  }

 private:
  aos::ScopedFD file_;
  char buffer_[kBufferSize];
};

// Simple interface to allow opening a file for writing and then writing it
// without any malloc's.
// TODO(james): It may make sense to add a ReadBytes() interface here that can
// take a memory buffer to fill, to avoid the templating required by the
// self-managed buffer of FileReader<>.
class FileWriter {
 public:
  // The result of an individual call to WriteBytes().
  // Because WriteBytes() may repeatedly call write() when partial writes occur,
  // it is possible for a non-zero number of bytes to have been written while
  // still having an error because a late call to write() failed.
  struct WriteResult {
    // Total number of bytes successfully written to the file.
    size_t bytes_written;
    // If the write was successful (return_code > 0), equal to bytes_written.
    // Otherwise, equal to the return value of the final call to write.
    int return_code;
  };

  FileWriter(std::string_view filename, mode_t permissions = S_IRWXU);

  WriteResult WriteBytes(absl::Span<const uint8_t> bytes);
  WriteResult WriteBytes(std::string_view bytes);
  void WriteBytesOrDie(absl::Span<const uint8_t> bytes);
  void WriteBytesOrDie(std::string_view bytes);
  int fd() const { return file_.get(); }

 private:
  aos::ScopedFD file_;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_UTIL_FILE_H_
