#ifndef FRC971_ORIN_CUDA_H_
#define FRC971_ORIN_CUDA_H_

#include <chrono>
#include <span>

#include "glog/logging.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

// CHECKs that a cuda method returned success.
// TODO(austin): This will not handle if and else statements quite right, fix if
// we care.
#define CHECK_CUDA(condition)                                             \
  if (auto c = condition)                                                 \
  LOG(FATAL) << "Check failed: " #condition " (" << cudaGetErrorString(c) \
             << ") "

namespace frc971::apriltag {

// Class to manage the lifetime of a Cuda stream.  This is used to provide
// relative ordering between kernels on the same stream.
class CudaStream {
 public:
  CudaStream() { CHECK_CUDA(cudaStreamCreate(&stream_)); }

  CudaStream(const CudaStream &) = delete;
  CudaStream &operator=(const CudaStream &) = delete;

  virtual ~CudaStream() { CHECK_CUDA(cudaStreamDestroy(stream_)); }

  // Returns the stream.
  cudaStream_t get() { return stream_; }

 private:
  cudaStream_t stream_;
};

// Class to manage the lifetime of a Cuda Event.  Cuda events are used for
// timing events on a stream.
class CudaEvent {
 public:
  CudaEvent() { CHECK_CUDA(cudaEventCreate(&event_)); }

  CudaEvent(const CudaEvent &) = delete;
  CudaEvent &operator=(const CudaEvent &) = delete;

  virtual ~CudaEvent() { CHECK_CUDA(cudaEventDestroy(event_)); }

  // Queues up an event to be timestamped on the stream when it is executed.
  void Record(CudaStream *stream) {
    CHECK_CUDA(cudaEventRecord(event_, stream->get()));
  }

  // Returns the time elapsed between start and this event if it has been
  // triggered.
  std::chrono::nanoseconds ElapsedTime(const CudaEvent &start) {
    float ms;
    CHECK_CUDA(cudaEventElapsedTime(&ms, start.event_, event_));
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<float, std::milli>(ms));
  }

  // Waits until the event has been triggered.
  void Synchronize() { CHECK_CUDA(cudaEventSynchronize(event_)); }

 private:
  cudaEvent_t event_;
};

// Class to manage the lifetime of page locked host memory for fast copies back
// to host memory.
template <typename T>
class HostMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T.
  HostMemory(size_t size) {
    T *memory;
    CHECK_CUDA(cudaMallocHost((void **)(&memory), size * sizeof(T)));
    span_ = std::span<T>(memory, size);
  }
  HostMemory(const HostMemory &) = delete;
  HostMemory &operator=(const HostMemory &) = delete;

  virtual ~HostMemory() { CHECK_CUDA(cudaFreeHost(span_.data())); }

  // Returns a pointer to the memory.
  T *get() { return span_.data(); }
  const T *get() const { return span_.data(); }

  // Returns the number of objects the memory can hold.
  size_t size() const { return span_.size(); }

  // Copies data from other (host memory) to this's memory.
  void MemcpyFrom(const T *other) {
    memcpy(span_.data(), other, sizeof(T) * size());
  }
  // Copies data to other (host memory) from this's memory.
  void MemcpyTo(const T *other) {
    memcpy(other, span_.data(), sizeof(T) * size());
  }

 private:
  std::span<T> span_;
};

// Class to manage the lifetime of device memory.
template <typename T>
class GpuMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T in
  // device memory.
  GpuMemory(size_t size) : size_(size) {
    CHECK_CUDA(cudaMalloc((void **)(&memory_), size * sizeof(T)));
  }
  GpuMemory(const GpuMemory &) = delete;
  GpuMemory &operator=(const GpuMemory &) = delete;

  virtual ~GpuMemory() { CHECK_CUDA(cudaFree(memory_)); }

  // Returns the device pointer to the memory.
  T *get() { return memory_; }
  const T *get() const { return memory_; }

  // Returns the number of objects this memory can hold.
  size_t size() const { return size_; }

  // Copies data from host memory to this memory asynchronously on the provided
  // stream.
  void MemcpyAsyncFrom(const T *host_memory, CudaStream *stream) {
    CHECK_CUDA(cudaMemcpyAsync(memory_, host_memory, sizeof(T) * size_,
                               cudaMemcpyHostToDevice, stream->get()));
  }
  void MemcpyAsyncFrom(const HostMemory<T> *host_memory, CudaStream *stream) {
    MemcpyAsyncFrom(host_memory->get(), stream);
  }

  // Copies data to host memory from this memory asynchronously on the provided
  // stream.
  void MemcpyAsyncTo(T *host_memory, size_t size, CudaStream *stream) const {
    CHECK_CUDA(cudaMemcpyAsync(reinterpret_cast<void *>(host_memory),
                               reinterpret_cast<void *>(memory_),
                               sizeof(T) * size, cudaMemcpyDeviceToHost,
                               stream->get()));
  }
  void MemcpyAsyncTo(T *host_memory, CudaStream *stream) const {
    MemcpyAsyncTo(host_memory, size_, stream);
  }
  void MemcpyAsyncTo(HostMemory<T> *host_memory, CudaStream *stream) const {
    MemcpyAsyncTo(host_memory->get(), stream);
  }

  // Copies data from host_memory to this memory blocking.
  void MemcpyFrom(const T *host_memory) {
    CHECK_CUDA(cudaMemcpy(reinterpret_cast<void *>(memory_),
                          reinterpret_cast<const void *>(host_memory),
                          sizeof(T) * size_, cudaMemcpyHostToDevice));
  }
  void MemcpyFrom(const HostMemory<T> *host_memory) {
    MemcpyFrom(host_memory->get());
  }

  // Copies data to host_memory from this memory.  Only copies size objects.
  void MemcpyTo(T *host_memory, size_t size) const {
    CHECK_CUDA(cudaMemcpy(reinterpret_cast<void *>(host_memory), memory_,
                          sizeof(T) * size, cudaMemcpyDeviceToHost));
  }
  // Copies data to host_memory from this memory.
  void MemcpyTo(T *host_memory) const { MemcpyTo(host_memory, size_); }
  void MemcpyTo(HostMemory<T> *host_memory) const {
    MemcpyTo(host_memory->get());
  }

  // Sets the memory asynchronously to contain data of type 'val' on the provide
  // stream.
  void MemsetAsync(const uint8_t val, CudaStream *stream) const {
    CHECK_CUDA(cudaMemsetAsync(memory_, val, sizeof(T) * size_, stream->get()));
  }

  // Allocates a vector on the host, copies size objects into it, and returns
  // it.
  std::vector<T> Copy(size_t s) const {
    CHECK_LE(s, size_);
    std::vector<T> result(s);
    MemcpyTo(result.data(), s);
    return result;
  }

  // Copies all the objects in this memory to a vector on the host and returns
  // it.
  std::vector<T> Copy() const { return Copy(size_); }

 private:
  T *memory_;
  const size_t size_;
};

// Synchronizes and CHECKs for success the last CUDA operation.
void CheckAndSynchronize(std::string_view message = "");

// Synchronizes and CHECKS iff --sync is passed on the command line.  Makes it
// so we can leave debugging in the code.
void MaybeCheckAndSynchronize();
void MaybeCheckAndSynchronize(std::string_view message);

}  // namespace frc971::apriltag

#endif  // FRC971_ORIN_CUDA_H_
