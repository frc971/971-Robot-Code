#include "frc971/orin/cuda.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(
    sync, false,
    "If true, force synchronization after each step to isolate errors better.");

namespace frc971 {
namespace apriltag {

void CheckAndSynchronize() {
  CHECK_CUDA(cudaDeviceSynchronize());
  CHECK_CUDA(cudaGetLastError());
}

void MaybeCheckAndSynchronize() {
  if (FLAGS_sync) CheckAndSynchronize();
}

}  // namespace apriltag
}  // namespace frc971
