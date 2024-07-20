// See :logfile_utils_out_of_space_test for usage and details.

#include <array>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/logfile_utils.h"
#include "aos/init.h"

ABSL_DECLARE_FLAG(int32_t, flush_size);
ABSL_FLAG(std::string, tmpfs, "", "tmpfs with the desired size");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  absl::SetFlag(&FLAGS_flush_size, 1);
  CHECK(!absl::GetFlag(FLAGS_tmpfs).empty())
      << ": Must specify a tmpfs location";

  std::array<uint8_t, 10240> data;
  data.fill(0);

  // Don't use odirect
  aos::logger::FileBackend file_backend("/", false);
  aos::logger::DetachedBufferWriter writer(
      file_backend.RequestFile(absl::GetFlag(FLAGS_tmpfs) + "/file"),
      std::make_unique<aos::logger::DummyEncoder>(data.size()));
  for (int i = 0; i < 8; ++i) {
    aos::logger::DataEncoder::SpanCopier coppier(data);
    writer.CopyMessage(&coppier, aos::monotonic_clock::now());
    CHECK(!writer.ran_out_of_space()) << ": " << i;
  }
  {
    aos::logger::DataEncoder::SpanCopier coppier(data);
    writer.CopyMessage(&coppier, aos::monotonic_clock::now());
  }
  CHECK(writer.ran_out_of_space());
  writer.acknowledge_out_of_space();
}
