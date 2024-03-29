#include "frc971/orin/cuda.h"

#include "absl/flags/flag.h"
#include "absl/log/check.h"

ABSL_FLAG(
    bool, sync, false,
    "If true, force synchronization after each step to isolate errors better.");

namespace frc971::apriltag {

size_t overall_memory = 0;

void CudaStream::Wait(CudaEvent *event) {
  CHECK_CUDA(cudaStreamWaitEvent(stream_, event->get(), 0));
}

void CheckAndSynchronize(std::string_view message) {
  CHECK_CUDA(cudaDeviceSynchronize()) << message;
  CHECK_CUDA(cudaGetLastError()) << message;
}

void MaybeCheckAndSynchronize() {
  if (absl::GetFlag(FLAGS_sync)) CheckAndSynchronize();
}

void MaybeCheckAndSynchronize(std::string_view message) {
  if (absl::GetFlag(FLAGS_sync)) CheckAndSynchronize(message);
}

}  // namespace frc971::apriltag
