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

  aos::logger::DetachedBufferWriter writer(
      FLAGS_tmpfs + "/file", std::make_unique<aos::logger::DummyEncoder>());
  std::array<uint8_t, 10240> data;
  data.fill(0);
  for (int i = 0; i < 8; ++i) {
    writer.QueueSpan(data);
    CHECK(!writer.ran_out_of_space()) << ": " << i;
  }
  writer.QueueSpan(data);
  CHECK(writer.ran_out_of_space());
  writer.acknowledge_out_of_space();
}
