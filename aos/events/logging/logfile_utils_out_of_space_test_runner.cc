// See :logfile_utils_out_of_space_test for usage and details.

#include <array>

#include "aos/events/logging/logfile_utils.h"
#include "aos/init.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DECLARE_int32(flush_size);
DEFINE_string(tmpfs, "", "tmpfs with the desired size");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  FLAGS_flush_size = 1;
  CHECK(!FLAGS_tmpfs.empty()) << ": Must specify a tmpfs location";

  std::array<uint8_t, 10240> data;
  data.fill(0);

  aos::logger::FileBackend file_backend("/");
  aos::logger::DetachedBufferWriter writer(
      file_backend.RequestFile(FLAGS_tmpfs + "/file"),
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
