#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <chrono>

#include "aos/init.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace chrono = std::chrono;

DEFINE_string(file, "/media/sda1/foo", "File to write to.");

DEFINE_uint32(write_size, 4096, "Size of hunk to write");
DEFINE_bool(sync, false, "If true, sync the file after each written block.");
DEFINE_bool(writev, false, "If true, use writev.");
DEFINE_bool(direct, false, "If true, O_DIRECT.");
DEFINE_uint32(chunks, 1, "Chunks to write using writev.");
DEFINE_uint32(chunk_size, 512, "Chunk size to write using writev.");

int main(int argc, char ** argv) {
  aos::InitGoogle(&argc, &argv);

  std::vector<uint8_t> data;

  // We want uncompressible data.  The easiest way to do this is to grab a good
  // sized block from /dev/random, and then reuse it.
  {
    int random_fd = open("/dev/random", O_RDONLY | O_CLOEXEC);
    PCHECK(random_fd != -1) << ": Failed to open /dev/random";
    data.resize(FLAGS_write_size);
    size_t written = 0;
    while (written < data.size()) {
      const size_t result =
          read(random_fd, data.data() + written, data.size() - written);
      PCHECK(result > 0);
      written += result;
    }

    PCHECK(close(random_fd) == 0);
  }

  std::vector<struct iovec> iovec;
  iovec.resize(FLAGS_chunks);
  CHECK_LE(FLAGS_chunks * FLAGS_chunk_size, FLAGS_write_size);

  for (size_t i = 0; i < FLAGS_chunks; ++i) {
    iovec[i].iov_base = &data[i * FLAGS_chunk_size];
    iovec[i].iov_len = FLAGS_chunk_size;
  }
  iovec[FLAGS_chunks - 1].iov_base = &data[(FLAGS_chunks - 1) * FLAGS_chunk_size];
  iovec[FLAGS_chunks - 1].iov_len = data.size() - (FLAGS_chunks - 1) * FLAGS_chunk_size;

  int fd = open(
      FLAGS_file.c_str(),
      O_RDWR | O_CLOEXEC | O_CREAT | (FLAGS_direct ? O_DIRECT : 0),
      0774);
  PCHECK(fd != -1);

  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  aos::monotonic_clock::time_point last_time = start_time;
  size_t last_written_data = 0;
  size_t written_data = 0;

  while (true) {
    if (FLAGS_writev) {
      PCHECK(writev(fd, iovec.data(), iovec.size()) ==
             static_cast<ssize_t>(data.size()))
          << ": Failed after "
          << chrono::duration<double>(aos::monotonic_clock::now() - start_time)
                 .count();
    } else {
      PCHECK(write(fd, data.data(), data.size()) ==
             static_cast<ssize_t>(data.size()))
          << ": Failed after "
          << chrono::duration<double>(aos::monotonic_clock::now() - start_time)
                 .count();
    }

    // Trigger a flush if asked.
    if (FLAGS_sync) {
      const aos::monotonic_clock::time_point monotonic_now =
          aos::monotonic_clock::now();
      sync_file_range(fd, written_data, data.size(), SYNC_FILE_RANGE_WRITE);

      // Now, blocking flush the previous page so we don't get too far ahead.
      // This is Linus' recommendation.
      if (written_data > 0) {
        sync_file_range(fd, written_data - data.size(), data.size(),
                        SYNC_FILE_RANGE_WAIT_BEFORE | SYNC_FILE_RANGE_WRITE |
                            SYNC_FILE_RANGE_WAIT_AFTER);
        posix_fadvise(fd, written_data - data.size(), data.size(),
                      POSIX_FADV_DONTNEED);
      }
      VLOG(1) << "Took "
              << chrono::duration<double>(aos::monotonic_clock::now() -
                                          monotonic_now)
                     .count();
    }

    written_data += data.size();

    const aos::monotonic_clock::time_point monotonic_now =
        aos::monotonic_clock::now();
    // Print out MB/s once it has been at least 1 second since last time.
    if (monotonic_now > last_time + chrono::seconds(1)) {
      LOG(INFO)
          << ((written_data - last_written_data) /
              chrono::duration<double>(monotonic_now - last_time).count() /
              1024. / 1024.)
          << " MB/s";
      last_time = monotonic_now;
      last_written_data = written_data;
    }
  }

  return 0;
}
