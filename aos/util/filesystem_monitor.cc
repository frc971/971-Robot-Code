#include <stddef.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <chrono>
#include <istream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_split.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/util/filesystem_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

namespace aos::util {
namespace {
std::optional<std::string> ReadShortFile(std::string_view file_name) {
  // Open as input and seek to end immediately.
  std::ifstream file(std::string(file_name), std::ios_base::in);
  if (!file.good()) {
    VLOG(1) << "Can't read " << file_name;
    return std::nullopt;
  }
  const size_t kMaxLineLength = 4096;
  char buffer[kMaxLineLength];
  file.read(buffer, kMaxLineLength);
  if (!file.eof()) {
    return std::nullopt;
  }
  return std::string(buffer, file.gcount());
}
}  // namespace

// Periodically sends out the Filesystems message with filesystem utilization
// info.
class FilesystemMonitor {
 public:
  FilesystemMonitor(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        sender_(event_loop_->MakeSender<FilesystemStatus>("/aos")) {
    periodic_timer_ =
        event_loop_->AddTimer([this]() { PublishFilesystemStatus(); });
    event_loop_->OnRun([this]() {
      periodic_timer_->Schedule(event_loop_->monotonic_now(),
                                std::chrono::seconds(5));
    });
  }

 private:
  void PublishFilesystemStatus() {
    aos::Sender<FilesystemStatus>::Builder builder = sender_.MakeBuilder();

    std::optional<std::string> contents = ReadShortFile("/proc/self/mountinfo");

    CHECK(contents.has_value());

    std::vector<flatbuffers::Offset<Filesystem>> filesystems;

    // Iterate through /proc/self/mounts to find all the filesystems.
    for (std::string_view line :
         absl::StrSplit(std::string_view(contents->c_str(), contents->size()),
                        '\n', absl::SkipWhitespace())) {
      // See https://www.kernel.org/doc/Documentation/filesystems/proc.txt for
      // the format.
      std::vector<std::string_view> elements =
          absl::StrSplit(line, ' ', absl::SkipWhitespace());

      // First thing after - is the filesystem type.
      size_t i = 6;
      while (elements[i] != "-") {
        ++i;
        CHECK_LT(i + 1, elements.size());
      }

      // Mount point is the 4th element.
      std::string mount_point(elements[4]);
      std::string_view type = elements[i + 1];

      // Ignore filesystems without reasonable types.
      if (type != "ext2" && type != "xfs" && type != "vfat" && type != "ext3" &&
          type != "ext4" && type != "tmpfs" && type != "devtmpfs") {
        continue;
      }
      VLOG(1) << mount_point << ", type " << type;

      struct statvfs info;

      PCHECK(statvfs(mount_point.c_str(), &info) == 0);

      VLOG(1) << "overall size: " << info.f_frsize * info.f_blocks << ", free "
              << info.f_bavail * info.f_bsize << ", inodes " << info.f_files
              << ", free " << info.f_ffree;

      flatbuffers::Offset<flatbuffers::String> path_offset =
          builder.fbb()->CreateString(mount_point);
      flatbuffers::Offset<flatbuffers::String> type_offset =
          builder.fbb()->CreateString(type);
      Filesystem::Builder filesystem_builder =
          builder.MakeBuilder<Filesystem>();
      filesystem_builder.add_path(path_offset);
      filesystem_builder.add_type(type_offset);
      filesystem_builder.add_overall_space(info.f_frsize * info.f_blocks);
      filesystem_builder.add_free_space(info.f_bavail * info.f_bsize);
      filesystem_builder.add_overall_inodes(info.f_files);
      filesystem_builder.add_free_inodes(info.f_ffree);

      filesystems.emplace_back(filesystem_builder.Finish());
    }

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Filesystem>>>
        filesystems_offset = builder.fbb()->CreateVector(filesystems);

    FilesystemStatus::Builder filesystem_status_builder =
        builder.MakeBuilder<FilesystemStatus>();

    filesystem_status_builder.add_filesystems(filesystems_offset);

    (void)builder.Send(filesystem_status_builder.Finish());
  }

  aos::EventLoop *event_loop_;

  aos::Sender<FilesystemStatus> sender_;

  aos::TimerHandler *periodic_timer_;
};

}  // namespace aos::util

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop shm_event_loop(&config.message());

  aos::util::FilesystemMonitor filesystem_monitor(&shm_event_loop);

  shm_event_loop.Run();

  return 0;
}
