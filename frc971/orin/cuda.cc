#include "frc971/orin/cuda.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(
    sync, false,
    "If true, force synchronization after each step to isolate errors better.");

namespace frc971::apriltag {

size_t overall_memory = 0;

void CheckAndSynchronize(std::string_view message) {
  CHECK_CUDA(cudaDeviceSynchronize()) << message;
  CHECK_CUDA(cudaGetLastError()) << message;
}

void MaybeCheckAndSynchronize() {
  if (FLAGS_sync) CheckAndSynchronize();
}

void MaybeCheckAndSynchronize(std::string_view message) {
  if (FLAGS_sync) CheckAndSynchronize(message);
}

}  // namespace frc971::apriltag
