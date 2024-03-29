#include "frc971/orin/line_fit_filter.h"

#include <cub/block/block_reduce.cuh>
#include <cub/warp/warp_merge_sort.cuh>
#include <iomanip>

#include "frc971/orin/cuda.h"

// #define DEBUG_BLOB_NUMBER 401

namespace frc971::apriltag {

static_assert(sizeof(LineFitPoint) == 40, "Size of LineFitPoint changed");
static_assert(sizeof(int4) == 16, "Size of int4 changed");

constexpr size_t kPointsPerBlock = 256;
// Number of errors to calculate before and after for the err filter and peak
// finder.
constexpr size_t kBeforeBuffer = 4;
constexpr size_t kAfterBuffer = 4;
constexpr size_t kErrorsBuffer = kBeforeBuffer + kAfterBuffer;

__device__ double FitLineError(size_t N, int64_t Mx, int64_t My, int64_t Mxx,
                               int64_t Myy, int64_t Mxy, int64_t W) {
  int64_t Cxx = Mxx * W - Mx * Mx;
  int64_t Cxy = Mxy * W - Mx * My;
  int64_t Cyy = Myy * W - My * My;

  // Pose it as an eigenvalue problem.
  //
  // TODO(austin): Are floats good enough?  Hard to tell what the "right answer"
  // actually is...
  float eig_small = ((Cxx + Cyy) - std::hypotf((Cxx - Cyy), 2 * Cxy)) /
                    static_cast<float>(W * W * 8.0);
  return N * eig_small;
}

struct TempStorage {
  // Block of points that we pre-load.
  LineFitPoint tmp_storage[kPointsPerBlock];

  // The errors loaded.  The first kAfterBuffer errors are the indices (could be
  // duplicated) immediately before the first blob.  The last kAfterBuffer
  // errors are the indices immediately following the last blob.
  double tmp_errs[kPointsPerBlock - 2 * kAfterBuffer];

  // The errors which are from the start of the first blob before the beginning
  // of the buffer.
  double tmp_errs_before[kAfterBuffer];
  // The errors which are from the end of the last blob after the end of the
  // buffer.
  double tmp_errs_after[kAfterBuffer];

  // The filtered errors in the primary block.
  double tmp_filtered_errs[kPointsPerBlock - 4 * kAfterBuffer + 2];

  // The filtered error corresponding to the first error in tmp_errs_before.
  double tmp_filtered_err_before;
  // The filtered error corresponding to the last error in tmp_errs_after.
  double tmp_filtered_err_after;

  static_assert(sizeof(tmp_storage) == sizeof(LineFitPoint) * kPointsPerBlock,
                "Storage changed size.");
};

template <size_t kThreads>
class ErrorCalculator {
 public:
  __host__ __device__ ErrorCalculator(
      const LineFitPoint *line_fit_points_device, size_t points,
      const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
      TempStorage *storage)
      : line_fit_points_device_(line_fit_points_device),
        points_(points),
        selected_extents_device_(selected_extents_device),
        storage_(storage) {}

  __host__ __device__ void Load() {
    constexpr size_t kLoadSize = sizeof(int4);
    static_assert(kLoadSize == 16, "kload");

    // Global index in 128 bit loads.
    const uint32_t global_index =
        global_block_index_cache_start_ * sizeof(LineFitPoint) / kLoadSize +
        threadIdx.x;

    for (uint32_t i = 0; i < kPointsPerBlock * sizeof(LineFitPoint) / kLoadSize;
         i += kThreads) {
      // Load things in 128 bit increments strided across our threads in such a
      // way our loads will coalesce nicely.
      //
      // Maybe overload a little bit past the end, but we over-allocate
      // things on the host too.  Otherwise the end of the message can be
      // corrupted.  For arrays which are multiples of kPointsPerBlock long,
      // this will have no issues.
      if (i + global_index <= points_ * sizeof(LineFitPoint) / kLoadSize) {
        *(reinterpret_cast<int4 *>(storage_->tmp_storage) + i + threadIdx.x) =
            *(reinterpret_cast<const int4 *>(line_fit_points_device_) +
              global_index + i);
      }
    }
  }

  // Loads the blob extents for the provided buffer index. This index is from
  // the front of the range we are responsible for computing.
  __host__ __device__ void LoadExtents(size_t index) {
    const size_t current_blob_index =
        storage_->tmp_storage[index + (blockIdx.x == 0 ? 0 : kErrorsBuffer)]
            .blob_index;
    if (current_blob_index != blob_index_) {
      starting_offset_ =
          selected_extents_device_[current_blob_index].value.starting_offset;
      count_ = selected_extents_device_[current_blob_index].value.count;
      blob_index_ = current_blob_index;
      ksz_ = std::min<int>(20, count_ / 12);
    }
  }

  // Loads the error from our shared memory.  This follows the same model as
  // BlobIndex and CalculateError below.  Call LoadExtents first to get the
  // right index, and proceed from there.
  __host__ __device__ double LoadError(ssize_t index) const {
    // Index into the current blob.  Should be between 0 and count_.
    const uint32_t positive_blob_index = (index + count_) % count_;

    // This one is a normal one, it's global index is inside our range or
    // contiguously just before our range.  Just do it.
    //
    // (note: kBeforeBuffer should be - on the right side of the >=, but due to
    // unsigned arithmatic, it needs to be on the left.)
    if (positive_blob_index + starting_offset_ + kBeforeBuffer >=
        global_block_index_start()) {
      // It is either inside our block, or just after our block.  That is all
      // contiguous in memory and easy to calculate, so just return it.
      if (positive_blob_index + starting_offset_ <
          global_block_index_end() + kAfterBuffer) {
        return storage_->tmp_errs[positive_blob_index + starting_offset_ +
                                  kBeforeBuffer - global_block_index_start()];
      }

      // OK, this is past the back, but at the back end of the blob at the back
      // end. These bytes are packed onto the back 4 objects of the buffer.
      const size_t buffer_end = starting_offset_ + count_;
      const size_t distance_from_end =
          buffer_end - (positive_blob_index + starting_offset_);
      return storage_->tmp_errs_after[4 - distance_from_end];
    } else {
      // We must be inside the before bits.
      return storage_->tmp_errs_before[positive_blob_index];
    }
  }

  // Loads the filtered error from the shared memory.  This follows the same
  // model as BlobIndex and CalculateError below.  Call LoadExtents first to get
  // the right index, and proceed from there.
  __host__ __device__ double LoadFilteredError(ssize_t index) const {
    const uint32_t positive_blob_index =
        (index + global_block_index_start_ + count_ - starting_offset_) %
        count_;
    // This one is a normal one, it's global index is inside our range or
    // contiguously just before our range.  Just do it.
    if (positive_blob_index + starting_offset_ + 1 >=
        global_block_index_start()) {
      // It is either inside our block, or just after our block.  That is all
      // contiguous in memory and easy to calculate, so just return it.
      if (positive_blob_index + starting_offset_ <
          global_block_index_end() + 1) {
        return storage_
            ->tmp_filtered_errs[positive_blob_index + starting_offset_ + 1 -
                                global_block_index_start()];
      }

      // OK, this is past the back, but at the back end of the blob at the back
      // end.  Return the on error we've got there.
      return storage_->tmp_filtered_err_after;
    } else {
      // We must be inside the before bits.
      return storage_->tmp_filtered_err_before;
    }
  }

  // Gets the line fit point from either our local cache, or from main memory if
  // it is too far away.
  __host__ __device__ LineFitPoint GetPoint(size_t global_index,
                                            bool print = false) const {
    if (global_index >= global_block_index_cache_end_ ||
        global_index < global_block_index_cache_start_) {
      if (print) {
#ifdef DEBUG_BLOB_NUMBER
        printf(
            "Block %d Thread %d   Loading global %d, relative %d from global\n",
            blockIdx.x, threadIdx.x, (int)global_index,
            (int)(global_index - starting_offset_));
#endif
      }
      return line_fit_points_device_[global_index];
    } else {
      if (print) {
#ifdef DEBUG_BLOB_NUMBER
        printf(
            "Block %d Thread %d   Loading global %d, relative %d from cache\n",
            blockIdx.x, threadIdx.x, (int)global_index,
            (int)(global_index - starting_offset_));
#endif
      }
      return storage_
          ->tmp_storage[global_index - global_block_index_cache_start_];
    }
  }

  // Returns the blob index for the provided buffer index.  This is relative to
  // the start of the range we are responsible for.
  __forceinline__ __host__ __device__ size_t
  BlobIndex(size_t buffer_index) const {
    return global_block_index_start_ + buffer_index - starting_offset_;
  }

  // Calculates the line fit error centered on the provided blob index in the
  // current extent.
  __device__ double CalculateError(ssize_t blob_index,
                                            bool print = false) const {
    // Index into the blob list for the current key.
    const size_t i0 = (blob_index + 2 * count_ - ksz_) % count_;
    const size_t i1 = (blob_index + count_ + ksz_) % count_;

    const size_t global_i0 = i0 + starting_offset_;
    const size_t global_i1 = i1 + starting_offset_;

    int32_t Mx, My, W;
    int64_t Mxx, Myy, Mxy;
    int N;  // how many points are included in the set?

    if (i0 < i1) {
      N = i1 - i0 + 1;

      LineFitPoint lf1 = GetPoint(global_i1);

      Mx = lf1.Mx;
      My = lf1.My;
      Mxx = lf1.Mxx;
      Mxy = lf1.Mxy;
      Myy = lf1.Myy;
      W = lf1.W;

      if (i0 > 0) {
        LineFitPoint lf0 = GetPoint(global_i0 - 1);

        Mx -= lf0.Mx;
        My -= lf0.My;
        Mxx -= lf0.Mxx;
        Mxy -= lf0.Mxy;
        Myy -= lf0.Myy;
        W -= lf0.W;
      }
    } else {
      // i0 > i1, e.g. [15, 2]. Wrap around.
      LineFitPoint lf0 = GetPoint(global_i0 - 1, print);
      LineFitPoint lfsz = GetPoint(starting_offset_ + count_ - 1, print);

      Mx = lfsz.Mx - lf0.Mx;
      My = lfsz.My - lf0.My;
      Mxx = lfsz.Mxx - lf0.Mxx;
      Mxy = lfsz.Mxy - lf0.Mxy;
      Myy = lfsz.Myy - lf0.Myy;
      W = lfsz.W - lf0.W;

      LineFitPoint lf1 = GetPoint(global_i1, print);

      Mx += lf1.Mx;
      My += lf1.My;
      Mxx += lf1.Mxx;
      Mxy += lf1.Mxy;
      Myy += lf1.Myy;
      W += lf1.W;

      N = count_ - i0 + i1 + 1;
    }

    // And now fit it.
    return FitLineError(N, Mx, My, Mxx, Myy, Mxy, W);
  }

  // Returns the starting global index of the region we are responsible for
  // computing.
  __host__ __device__ __forceinline__ uint32_t
  global_block_index_start() const {
    return global_block_index_start_;
  }
  // Returns the ending global index of the region we are responsible for
  // computing.
  __host__ __device__ __forceinline__ uint32_t global_block_index_end() const {
    return global_block_index_end_;
  }
  // Returns the size of the region we are responsible for computing.
  __host__ __device__ __forceinline__ uint32_t global_block_index_size() const {
    return global_block_index_end_ - global_block_index_start_;
  }

  // Returns the starting index in global indices of the current blob ID.
  __host__ __device__ __forceinline__ uint32_t
  global_current_blob_starting_index() const {
    return starting_offset_;
  }

  // Returns the number of points in the current blob ID.
  __host__ __device__ __forceinline__ uint32_t
  global_current_blob_count() const {
    return count_;
  }

  // Returns the current blob ID.
  __host__ __device__ __forceinline__ uint32_t blob_index() const {
    return blob_index_;
  }

 private:
  const LineFitPoint *line_fit_points_device_;
  size_t points_;
  const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device_;

  TempStorage *storage_;

  // Start and end location of the LineFitPoint cache in tmp_storage_.
  const uint32_t global_block_index_cache_start_ =
      blockIdx.x == 0
          ? 0
          : blockIdx.x * (kPointsPerBlock - 2 * kErrorsBuffer) - kErrorsBuffer;
  const uint32_t global_block_index_cache_end_ = std::min<uint32_t>(
      global_block_index_cache_start_ + kPointsPerBlock, points_);

  const uint32_t global_block_index_start_ =
      blockIdx.x * (kPointsPerBlock - 2 * kErrorsBuffer);
  const uint32_t global_block_index_end_ = std::min<uint32_t>(
      global_block_index_start_ + kPointsPerBlock - 2 * kErrorsBuffer, points_);

  size_t blob_index_ = 0xffffffff;
  size_t starting_offset_ = 0;
  size_t count_ = 0;
  size_t ksz_ = 0;
};

template <size_t kThreads>
__global__ void DoFitLines(
    const LineFitPoint *line_fit_points_device, size_t points,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    double *errs_device, double *filtered_errs_device, Peak *peaks_device) {
  __shared__ TempStorage storage;

  ErrorCalculator<kThreads> calculator(line_fit_points_device, points,
                                       selected_extents_device, &storage);

  calculator.Load();

  // TODO(austin): Whichever warp loads the first ksz blobs should load the
  // prior ksz blobs.  Figure out if things are too slow first.  __syncwarp()
  // and make the first warp do it?

  __syncthreads();

  // We need to compute a couple of errs_device past each end in shared memory.
  // This needs to be (FilterCoefficients().size() - 1) + 2.  The first term is
  // the number of extra errors we need for the filter.  The second is the extra
  // number of errors needed to implement the peak finder (1 on each side).

  // OK, the way this all works now, error calculation work [0, kErrorsBuffer)
  // is done in the first kErrorsBuffer threads, and written to
  // tmp_errs [0, kErrorsBuffer), even if this wraps back into the buffer.

  // Now that everything is loaded, have each thread process its points by
  // strides.

  // TODO(austin): We need 4 reserved ranges.  Proposal is:
  //   [0, kErrorsBuffer) -> beginning points of the extent of blob 0, ie
  //     starting at starting_offset_ globally.
  //
  //   [kErrorsBuffer, 2 * kErrorsBuffer) -> 4 points immediately before the
  //   starting point.
  //
  // And then symetrically off the back end too.  This will mean we process 16
  // less points per thread.  Whatever.

  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    // OK, now we need to do the actual line fit.
    if (i < kErrorsBuffer) {
      // Start by extending the errors before the blob in our buffer space.  All
      // of this will be done in blob0's address space.
      //
      // In the first block, this will load the 8th element.  But, since blobs
      // are 24 or longer, this will be the 0th blob.  For following blocks,
      // this is actually the right index.
      calculator.LoadExtents(0);

      // First 4 threads do the first 4 elements in the first blob.
      if (i < kBeforeBuffer) {
        storage.tmp_errs_before[i] = calculator.CalculateError(i);
      } else {
        // Then, compute the 4 errors right before the buffer.
        storage.tmp_errs[i - kBeforeBuffer] = calculator.CalculateError(
            calculator.BlobIndex(i - kBeforeBuffer) - kBeforeBuffer);
      }
    } else if (i - kErrorsBuffer + calculator.global_block_index_start() <
               calculator.global_block_index_end() + kBeforeBuffer) {
      // Now we are solidly in the middle of the block.  Make sure anything
      // computed after the end of the buffer uses the extents of the end.
      calculator.LoadExtents(std::min<uint32_t>(
          calculator.global_block_index_size() - 1, i - kErrorsBuffer));
      size_t target_index = calculator.BlobIndex(i - kErrorsBuffer);

#ifdef DEBUG_BLOB_NUMBER
      const bool print =
          (calculator.blob_index() == DEBUG_BLOB_NUMBER &&
           ((484 - 4 <= target_index && target_index <= 484 + 4)));
#endif

      storage.tmp_errs[i - kBeforeBuffer] =
          calculator.CalculateError(target_index);
#ifdef DEBUG_BLOB_NUMBER
      if (print) {
        printf("Block %d Thread %d (idx %d)   Err: %f\n", blockIdx.x,
               threadIdx.x, (int)target_index,
               calculator.CalculateError(target_index, true));
      }
#endif
    } else {
      // Past the end of the normal calcs, do the end buffering.
      if (i < kPointsPerBlock - kBeforeBuffer) {
        continue;
      }

      // We are just supposed to continue with the extents of the last index
      // and keep going.
      calculator.LoadExtents(calculator.global_block_index_size() - 1);

      // Wrap before the beginning now of the last blob.
      storage.tmp_errs_after[i - (kPointsPerBlock - kBeforeBuffer)] =
          calculator.CalculateError(calculator.global_current_blob_count() -
                                    kBeforeBuffer +
                                    (i - (kPointsPerBlock - kBeforeBuffer)));
    }
  }

  __syncthreads();

#ifdef DEBUG_BLOB_NUMBER
  /*
  if (threadIdx.x == 0) {
    for (int i = (int)calculator.global_block_index_cache_start_;
         i < (int)calculator.global_block_index_cache_end_; ++ i) {
      auto x = calculator.GetPoint(i);
      if (x.blob_index == DEBUG_BLOB_NUMBER) {
        printf("Block %d Thread %d   Loading global %d, relative %d, Mx: %f\n",
               blockIdx.x, threadIdx.x, i,
               (int)(i - calculator.selected_extents_device_[x.blob_index]
                             .value.starting_offset),
               x.Mx / 2.0);
      }
    }
  }

  __syncthreads();
  */
#endif

  // We now have all the errors loaded! Box filter them.
  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    // The peak finder needs 1 more filtered error in each direction.
    ssize_t target_index;
    double *destination;
    if (i < kErrorsBuffer) {
      if (i < kErrorsBuffer - 2) {
        continue;
      } else {
        calculator.LoadExtents(0);
        if (i < kErrorsBuffer - 1) {
          // Target index is now 0.
          target_index = 0;
          destination = &storage.tmp_filtered_err_before;
        } else {
          target_index = calculator.BlobIndex(0) - 1;
          destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
        }
      }
    } else if (i - kErrorsBuffer + calculator.global_block_index_start() >=
               calculator.global_block_index_end()) {
      if (i - kErrorsBuffer + calculator.global_block_index_start() >=
          calculator.global_block_index_end() + 2) {
        // Past 1 past the end for the peak finder.
        break;
      }
      calculator.LoadExtents(calculator.global_block_index_end() -
                             calculator.global_block_index_start() - 1);

      if (i - kErrorsBuffer + calculator.global_block_index_start() >=
          calculator.global_block_index_end() + 1) {
        destination = &storage.tmp_filtered_err_after;
        target_index = calculator.global_current_blob_count() - 1;
      } else {
        destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
        target_index = calculator.BlobIndex(i - kErrorsBuffer);
      }
    } else {
      calculator.LoadExtents(i - kErrorsBuffer);
      target_index = calculator.BlobIndex(i - kErrorsBuffer);
      destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
    }

    double accumulated_error = 0.0;

#ifdef DEBUG_BLOB_NUMBER
    const bool print = (calculator.blob_index() == DEBUG_BLOB_NUMBER &&
                        ((484 - 4 <= target_index && target_index <= 484 + 4)));
#endif

#pragma unroll
    for (size_t j = 0; j < FilterCoefficients().size(); ++j) {
      const double e = calculator.LoadError(target_index + j -
                                            FilterCoefficients().size() / 2);
      accumulated_error += e * FilterCoefficients()[j];
#ifdef DEBUG_BLOB_NUMBER
      if (print) {
        printf("Block %d Thread %d (idx %d)    + %f * %f (%f) -> %f\n",
               blockIdx.x, threadIdx.x, (int)target_index, e,
               FilterCoefficients()[j], e * FilterCoefficients()[j],
               accumulated_error);
      }
#endif
    }
    *destination = accumulated_error;

#ifdef DEBUG_BLOB_NUMBER
    if (print) {
      printf(
          "Block %d Thread %d (idx %d)  Blob %d of size %d, index %d "
          "filtered_error = "
          "%f, error = %f\n",
          blockIdx.x, threadIdx.x, (int)target_index,
          (int)calculator.blob_index(),
          (int)calculator.global_current_blob_count(), (int)target_index,
          accumulated_error, calculator.LoadError(target_index));
    }
#endif
  }

  __syncthreads();
  // OK, errors are now saved to shared memory.  Compute peaks and push out to
  // disk.

  // TODO(austin): Don't save out the unfiltered errors if we aren't
  // testing/debugging...
  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    if (i < kErrorsBuffer) {
      continue;
    }

    if (i - kErrorsBuffer + calculator.global_block_index_start() >=
        calculator.global_block_index_end()) {
      break;
    }

    errs_device[i - kErrorsBuffer + calculator.global_block_index_start()] =
        storage.tmp_errs[i - kBeforeBuffer];
  }

  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    if (i < kErrorsBuffer) {
      continue;
    }

    if (i - kErrorsBuffer + calculator.global_block_index_start() >=
        calculator.global_block_index_end()) {
      break;
    }

    calculator.LoadExtents(i - kErrorsBuffer);

    const double before_error =
        calculator.LoadFilteredError(i - kErrorsBuffer - 1);
    const double my_error = calculator.LoadFilteredError(i - kErrorsBuffer);
    const double after_error =
        calculator.LoadFilteredError(i - kErrorsBuffer + 1);

    // This gets us ready to sort.
    filtered_errs_device[i - kErrorsBuffer +
                         calculator.global_block_index_start()] = my_error;

    const bool is_peak = my_error > before_error && my_error > after_error;

    Peak peak;
    peak.error = -my_error;
    peak.blob_index = is_peak ? calculator.blob_index() : Peak::kNoPeak();
    peak.filtered_point_index =
        i - kErrorsBuffer + calculator.global_block_index_start();

    peaks_device[peak.filtered_point_index] = peak;
  }
}

void FitLines(
    const LineFitPoint *line_fit_points_device, size_t points,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    size_t num_extents, double *errs_device, double *filtered_errs_device,
    Peak *peaks_device, CudaStream *stream) {
  constexpr size_t kThreads = 128;
  const size_t kBlocks = (points + kPointsPerBlock - 2 * kErrorsBuffer - 1) /
                         (kPointsPerBlock - 2 * kErrorsBuffer);

  VLOG(1) << "Spawning with " << kThreads << " threads, and " << kBlocks
          << " blocks for " << num_extents << " blob_ids and " << points
          << " points";
  DoFitLines<kThreads><<<kBlocks, kThreads, 0, stream->get()>>>(
      line_fit_points_device, points, selected_extents_device, errs_device,
      filtered_errs_device, peaks_device);
}

namespace {

// Returns n choose k as a constexpr.
constexpr uint Binomial(uint n, uint k) {
  if (n < k) {
    return 0;
  }

  uint m = std::min(k, n - k);
  uint result = 1;

  for (uint i = 0; i < m; ++i) {
    result *= (n - i);
    result /= (i + 1);
  }

  return result;
}

constexpr int kNMaxima = 10;

// Returns the sum of binomials for CumulativeSumsM0.
template <size_t nmaxima, size_t N>
constexpr uint SumBinomial(uint i, uint start = 0) {
  size_t result = 0;
  for (uint m0 = start; m0 <= i; ++m0) {
    result += Binomial(nmaxima - m0 - 1, N);
  }
  return result;
}

// Returns the cumulative number of indices consumed by incrementing each m0
// index.
constexpr std::array<uint, 7> CumulativeSumsM0() {
  constexpr size_t nmaxima = kNMaxima;
  return {
      SumBinomial<nmaxima, 3>(0, 0), SumBinomial<nmaxima, 3>(1, 0),
      SumBinomial<nmaxima, 3>(2, 0), SumBinomial<nmaxima, 3>(3, 0),
      SumBinomial<nmaxima, 3>(4, 0), SumBinomial<nmaxima, 3>(5, 0),
      SumBinomial<nmaxima, 3>(6, 0),
  };
}

// Returns the number of indices consumed by each m1 index.  [0] is how many
// you'd consume if m0 and m1 were both as small as they could be.  To handle m0
// > 0, increment your index.
constexpr std::array<uint, 7> BinomialM1() {
  constexpr size_t nmaxima = kNMaxima;
  return {
      Binomial(nmaxima - 2, 2), Binomial(nmaxima - 3, 2),
      Binomial(nmaxima - 4, 2), Binomial(nmaxima - 5, 2),
      Binomial(nmaxima - 6, 2), Binomial(nmaxima - 7, 2),
      Binomial(nmaxima - 8, 2),
  };
}

// Returns the m0 for the provided global index.
__host__ __device__ const cuda::std::pair<uint, uint> FindM0(uint i) {
  uint m0 = 0;
  uint last = 0;
  while (true) {
    uint next = CumulativeSumsM0()[m0];
    if (i < next) {
      return cuda::std::make_pair(m0, i - last);
    }
    last = next;
    ++m0;
  }
}

// Returns the m1 for the provided m0 and global index remainder.
__host__ __device__ const cuda::std::pair<uint, uint> FindM1(uint m0, uint i) {
  uint m1 = m0;
  while (true) {
    uint next = BinomialM1()[m1];
    if (i < next || m1 == kNMaxima - 4) {
      return cuda::std::make_pair(m1, i);
    }
    i -= next;
    ++m1;
  }
}

// Returns the m2 for the provided m1 and global index remainder.
__host__ __device__ const cuda::std::pair<uint, uint> FindM2(uint m1, uint i) {
  uint m2 = m1;
  while (true) {
    uint next = kNMaxima - m2 - 1 - 1;
    if (i < next || m2 == kNMaxima - 3) {
      return cuda::std::make_pair(m2, i);
    }
    i -= next;
    ++m2;
  }
}

}  // namespace

__host__ __device__ std::tuple<uint, uint, uint, uint> Unrank(uint i) {
  uint m0 = 0;
  uint m1;
  uint m2;
  uint m3;

  auto result0 = FindM0(i);
  m0 = result0.first;
  i = result0.second;

  auto result1 = FindM1(m0, result0.second);
  m1 = result1.first + 1;

  i = result1.second;
  auto result2 = FindM2(m1, result1.second);
  m2 = result2.first + 1;

  m3 = m2 + result2.second + 1;
  return std::make_tuple(m0, m1, m2, m3);
}

// TODO(austin): Convert this into a constant lookup.
__device__ __host__ cuda::std::pair<uint, uint> GetM0M1(uint tid) {
  constexpr int nmaxima = kNMaxima;
  uint count = 0;
  for (uint m0 = 0; m0 < nmaxima - 3; m0++) {
    for (uint m1 = m0 + 1; m1 < nmaxima - 2; m1++) {
      if (count == tid) {
        return cuda::std::make_pair(m0, m1);
      }
      ++count;
    }
  }
  return cuda::std::make_pair(0, 0);
}

__device__ __forceinline__ LineFitMoments
ReadMoments(const LineFitPoint *line_fit_points_device, size_t blob_point_count,
            size_t index0, size_t index1) {
  LineFitMoments result;

  if (index0 < index1) {
    result.N = index1 - index0 + 1;

    LineFitPoint lf1 = line_fit_points_device[index1];

    result.Mx = lf1.Mx;
    result.My = lf1.My;
    result.Mxx = lf1.Mxx;
    result.Mxy = lf1.Mxy;
    result.Myy = lf1.Myy;
    result.W = lf1.W;

    if (index0 > 0) {
      LineFitPoint lf0 = line_fit_points_device[index0 - 1];

      result.Mx -= lf0.Mx;
      result.My -= lf0.My;
      result.Mxx -= lf0.Mxx;
      result.Mxy -= lf0.Mxy;
      result.Myy -= lf0.Myy;
      result.W -= lf0.W;
    }
  } else {
    // index0 > index1, e.g. [15, 2]. Wrap around.
    LineFitPoint lf0 = line_fit_points_device[index0 - 1];
    LineFitPoint lfsz = line_fit_points_device[blob_point_count - 1];

    result.Mx = lfsz.Mx - lf0.Mx;
    result.My = lfsz.My - lf0.My;
    result.Mxx = lfsz.Mxx - lf0.Mxx;
    result.Mxy = lfsz.Mxy - lf0.Mxy;
    result.Myy = lfsz.Myy - lf0.Myy;
    result.W = lfsz.W - lf0.W;

    LineFitPoint lf1 = line_fit_points_device[index1];

    result.Mx += lf1.Mx;
    result.My += lf1.My;
    result.Mxx += lf1.Mxx;
    result.Mxy += lf1.Mxy;
    result.Myy += lf1.Myy;
    result.W += lf1.W;

    result.N = blob_point_count - index0 + index1 + 1;
  }
  return result;
}

__device__ void FitLine(LineFitMoments moments, double *lineparam01,
                        double *lineparam23, double *err, double *mse,
                        bool print = false) {
  if (print) {
    printf(
        "Block %d Thread %d   Mx: %.4f, My: %.4f, Mxx: %.4f, Mxy: %.4f, Myy: "
        "%.4f, W: %d, N: %d\n",
        blockIdx.x, threadIdx.x, moments.Mx / 2.0, moments.My / 2.0,
        moments.Mxx / 4.0, moments.Mxy / 4.0, moments.Myy / 4.0, moments.W,
        moments.N);
  }
  int64_t Cxx = moments.Mxx * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.Mx);
  int64_t Cxy = moments.Mxy * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.My);
  int64_t Cyy = moments.Myy * moments.W - static_cast<int64_t>(moments.My) *
                                              static_cast<int64_t>(moments.My);

  // Pose it as an eigenvalue problem.
  //
  // TODO(austin): Are floats good enough?  Hard to tell what the "right answer"
  // actually is...
  const float hypot_cached = std::hypotf((Cxx - Cyy), 2 * Cxy);
  const float eight_w_squared = static_cast<float>(
      static_cast<int64_t>(moments.W) * static_cast<int64_t>(moments.W) * 8.0);
  const float eig_small = (Cxx + Cyy - hypot_cached) / eight_w_squared;
  if (print) {
    printf("Block %d Thread %d   eig_small: (%ld + %ld - %f) / %f -> %f\n",
           blockIdx.x, threadIdx.x, Cxx, Cyy, hypot_cached, eight_w_squared,
           eig_small);
  }

  if (lineparam01) {
    lineparam01[0] =
        static_cast<float>(moments.Mx) / static_cast<float>(moments.W * 2);
    lineparam01[1] =
        static_cast<float>(moments.My) / static_cast<float>(moments.W * 2);
  }
  if (lineparam23) {
    // These don't match the originals at all, but the math should come out
    // right.  n{xy}{12} end up being multiplied by 8 W^2, and we compare the
    // square but use hypot on nx, ny directly.  (and let the W^2 term come out
    // as common to both the hypot and nxy terms.)
    const float nx1 = static_cast<float>(Cxx - Cyy) - hypot_cached;
    const float ny1 = 2 * Cxy;
    const float M1 = nx1 * nx1 + ny1 * ny1;
    const float nx2 = 2 * Cxy;
    const float ny2 = static_cast<float>(Cyy - Cxx) - hypot_cached;
    const float M2 = nx2 * nx2 + ny2 * ny2;

    float nx, ny;
    if (M1 > M2) {
      nx = nx1;
      ny = ny1;
    } else {
      nx = nx2;
      ny = ny2;
    }

    float length = std::hypotf(nx, ny);
    lineparam23[0] = nx / length;
    lineparam23[1] = ny / length;
  }

  // sum of squared errors
  *err = moments.N * eig_small;

  if (print) {
    printf("Block %d Thread %d   err: %f\n", blockIdx.x, threadIdx.x,
           moments.N * eig_small);
  }

  // mean squared error
  *mse = eig_small;
}

__device__ __forceinline__ void FitLine(
    const LineFitPoint *line_fit_points_device, size_t blob_point_count,
    size_t index0, size_t index1, double *lineparam01, double *lineparam23,
    double *err, double *mse, bool print = false) {
  LineFitMoments moments =
      ReadMoments(line_fit_points_device, blob_point_count, index0, index1);
  FitLine(moments, lineparam01, lineparam23, err, mse, print);
}

struct QuadFitStorage {
  double errorm0m1[kNMaxima - 3][kNMaxima - 3];
  double lineparams23m0m1[kNMaxima - 3][kNMaxima - 3][2];
  uint16_t point_indices[16];
};

class QuadFitCalculator {
 public:
  __host__ __device__ QuadFitCalculator(
      const Peak *peaks_device, const PeakExtents *peak_extents,
      const LineFitPoint *line_fit_points_device,
      const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
      float max_line_fit_mse, double max_dot, QuadFitStorage *storage)
      : peaks_device_(peaks_device),
        extents_(peak_extents[blockIdx.x]),
        selected_extent_(selected_extents_device[extents_.blob_index].value),
        line_fit_points_device_(line_fit_points_device +
                                selected_extent_.starting_offset),
        max_line_fit_mse_(max_line_fit_mse),
        max_dot_(max_dot),
        storage_(storage) {}

  __host__ __device__ int RawIndexFromMaxima(int m) const {
    return peaks_device_[m + extents_.starting_offset].filtered_point_index -
           selected_extent_.starting_offset;
  }

  __host__ __device__ uint32_t IndexFromMaxima(int m) const {
    return storage_->point_indices[m];
  }

  __device__ __forceinline__ uint32_t sz() const {
    return selected_extent_.count;
  }

  __device__ __forceinline__ void FitLine(size_t index0, size_t index1,
                                          double *lineparam01,
                                          double *lineparam23, double *err,
                                          double *mse,
                                          bool print = false) const {
    if (print) {
      printf("Block %d Thread %d   Fitting line %d, %d\n", blockIdx.x,
             threadIdx.x, (int)index0, (int)index1);
    }
    ::frc971::apriltag::FitLine(line_fit_points_device_, selected_extent_.count,
                                index0, index1, lineparam01, lineparam23, err,
                                mse, print);
  }

  __device__ __forceinline__ LineFitMoments ReadMoments(size_t index0,
                                                        size_t index1) {
    return ::frc971::apriltag::ReadMoments(
        line_fit_points_device_, selected_extent_.count, index0, index1);
  }

  __device__ void ComputeM0M1Fit() {
    if (extents_.count < 4) {
      return;
    }

    cuda::std::pair<uint, uint> m0m1 = GetM0M1(threadIdx.x);
    const uint m0 = m0m1.first;
    const uint m1 = m0m1.second;
    if (m0 >= m1) {
      return;
    }

    // First 35 threads compute the first 35 m0, m1 pairs.
    // That lets us cache them in shared memory.  Then, all the threads
    // calculate the full depth of the tree.
    //
    // If we don't have the full kNMaxima maxima, skip any indices which are
    // past the end.
    if (m1 < kNMaxima && m1 < extents_.count) {
      const uint i0 = IndexFromMaxima(m0);
      const uint i1 = IndexFromMaxima(m1);

      double err01;
      double mse01;

      FitLine(i0, i1, nullptr, storage_->lineparams23m0m1[m0][m1 - 1], &err01,
              &mse01);

      if (mse01 > max_line_fit_mse_) {
        err01 = std::numeric_limits<double>::max();
      }

      storage_->errorm0m1[m0][m1 - 1] = err01;
    } else {
      storage_->errorm0m1[m0][m1 - 1] = std::numeric_limits<double>::max();
    }
  }

  __host__ __device__ int blob_index() const { return extents_.blob_index; }

  __device__ double FitLines(uint m0, uint m1, uint m2,
                                      uint m3) const {
    const bool print =
#ifdef DEBUG_BLOB_NUMBER
        (blob_index() == DEBUG_BLOB_NUMBER &&
         (  //(m0 == 5 && m1 == 6 && m2 == 7 && m3 == 8) ||
             (m0 == 0 && m1 == 4 && m2 == 8 && m3 == 9)));
#else
        false;
#endif
    double err = std::numeric_limits<double>::max();

    if (extents_.count < 4) {
      return err;
    }

    if (m3 >= kNMaxima || m3 >= extents_.count) {
      return err;
    }

    const double errm0m1 = storage_->errorm0m1[m0][m1 - 1];
    if (errm0m1 == std::numeric_limits<double>::max()) {
      return err;
    }

    const double *paramsm0m123 = storage_->lineparams23m0m1[m0][m1 - 1];

    double errm1m2;
    double msem1m2;
    double paramsm1m223[2];

    const int i1 = IndexFromMaxima(m1);
    const int i2 = IndexFromMaxima(m2);
    FitLine(i1, i2, nullptr, paramsm1m223, &errm1m2, &msem1m2, print);
    if (msem1m2 > max_line_fit_mse_) {
      return std::numeric_limits<double>::max();
    }

    // return m0 + m1 * 10 + m2 * 100 + m3 * 1000 + errm0m1 + errm1m2;

    const double dot =
        paramsm0m123[0] * paramsm1m223[0] + paramsm0m123[1] * paramsm1m223[1];
    if (fabs(dot) > max_dot_) {
      return std::numeric_limits<double>::max();
    }

    const int i3 = IndexFromMaxima(m3);

    double errm2m3;
    double msem2m3;

    FitLine(i2, i3, nullptr, nullptr, &errm2m3, &msem2m3, print);
    if (msem2m3 > max_line_fit_mse_) {
      return std::numeric_limits<double>::max();
    }

    const int i0 = IndexFromMaxima(m0);
    double errm3m0;
    double msem3m0;
    FitLine(i3, i0, nullptr, nullptr, &errm3m0, &msem3m0, print);
    if (msem3m0 > max_line_fit_mse_) {
      return std::numeric_limits<double>::max();
    }

    if (print) {
      printf(
          "Block %d Thread %d  %d %d %d %d: errm0m1 %f errm1m2 %f errm2m3 %f "
          "errm3m0 %f\n",
          blockIdx.x, threadIdx.x, i0, i1, i2, i3, errm0m1, errm1m2, errm2m3,
          errm3m0);
    }

    return errm0m1 + errm1m2 + errm2m3 + errm3m0;
  }

  __host__ __device__ size_t peaks_count() const { return extents_.count; }

 private:
  const Peak *peaks_device_;
  const PeakExtents extents_;
  const MinMaxExtents selected_extent_;

  const LineFitPoint *line_fit_points_device_;
  const float max_line_fit_mse_;
  const double max_dot_;
  QuadFitStorage *storage_;
};

struct QuadError {
  uint8_t m0;
  uint8_t m1;
  uint8_t m2;
  uint8_t m3;
  double error;
};

struct MinQuadError {
  __host__ __device__ QuadError operator()(const QuadError &a,
                                           const QuadError &b) {
    if (a.error <= b.error) {
      return a;
    } else {
      return b;
    }
  }
};

struct CustomLess {
  __device__ bool operator()(const uint16_t lhs, const uint16_t rhs) {
    return lhs < rhs;
  }
};

__global__ void DoFitQuads(
    const Peak *peaks_device, const PeakExtents *peak_extents,
    const LineFitPoint *line_fit_points_device,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    float max_line_fit_mse, double cos_critical_rad,
    FitQuad *fit_quads_device) {
  __shared__ QuadFitStorage storage;
  // Specialize BlockReduce for a 1D block of 128 threads of type int
  typedef cub::BlockReduce<QuadError, MaxRankedIndex()> BlockReduce;
  // Allocate shared memory for BlockReduce
  __shared__ typename BlockReduce::TempStorage temp_storage_reduce;

  QuadFitCalculator calculator(peaks_device, peak_extents,
                               line_fit_points_device, selected_extents_device,
                               max_line_fit_mse, cos_critical_rad, &storage);

  // Step 1, unsort the maxima back by point index.
  constexpr size_t kItemsPerThread = 1;
  constexpr size_t kItems = 16;
  using WarpMergeSortT = cub::WarpMergeSort<uint16_t, kItemsPerThread, kItems>;

  if (threadIdx.x < kItems) {
    __shared__ typename WarpMergeSortT::TempStorage temp_storage_merge;
    uint16_t index[1];
    if (threadIdx.x >= calculator.peaks_count() || threadIdx.x >= kNMaxima) {
      index[0] = std::numeric_limits<uint16_t>::max();
    } else {
      index[0] = calculator.RawIndexFromMaxima(threadIdx.x);
    }
    WarpMergeSortT(temp_storage_merge).Sort(index, CustomLess());
    storage.point_indices[threadIdx.x] = index[0];
  }

  __syncthreads();

#ifdef DEBUG_BLOB_NUMBER
  if (threadIdx.x == 0) {
    if (calculator.blob_index() == DEBUG_BLOB_NUMBER) {
      for (size_t i = 0; i < std::min(calculator.peaks_count(), kItems); ++i) {
        printf("  %d: %d, now %d\n", (int)i, calculator.RawIndexFromMaxima(i),
               storage.point_indices[i]);
      }
    }
  }
#endif

  // TODO(austin): Where do we handle blobs with less than 4 extents?
  calculator.ComputeM0M1Fit();

  __syncthreads();

  const std::tuple<uint, uint, uint, uint> ms = Unrank(threadIdx.x);
  QuadError quad_error;
  quad_error.m0 = std::get<0>(ms);
  quad_error.m1 = std::get<1>(ms);
  quad_error.m2 = std::get<2>(ms);
  quad_error.m3 = std::get<3>(ms);
  quad_error.error = calculator.FitLines(quad_error.m0, quad_error.m1,
                                         quad_error.m2, quad_error.m3);

#ifdef DEBUG_BLOB_NUMBER
  if (calculator.blob_index() == DEBUG_BLOB_NUMBER) {
    if (quad_error.error < 0.0) {
      printf("blob index %d maxima: %d %d %d %d, error negative of: %f\n",
             (int)calculator.blob_index(), (int)quad_error.m0,
             (int)quad_error.m1, (int)quad_error.m2, (int)quad_error.m3,
             quad_error.error);
    }
  }
#endif

  // Each thread obtains an input item
  // Compute the block-wide max for thread0
  QuadError min_error =
      BlockReduce(temp_storage_reduce).Reduce(quad_error, MinQuadError());

  if (threadIdx.x == 0) {
    const bool valid = min_error.error < max_line_fit_mse * calculator.sz();
    fit_quads_device[blockIdx.x].valid = valid;
    fit_quads_device[blockIdx.x].blob_index = calculator.blob_index();
    uint32_t i0 = calculator.IndexFromMaxima(min_error.m0);
    uint32_t i1 = calculator.IndexFromMaxima(min_error.m1);
    uint32_t i2 = calculator.IndexFromMaxima(min_error.m2);
    uint32_t i3 = calculator.IndexFromMaxima(min_error.m3);
#ifdef DEBUG_BLOB_NUMBER
    if (calculator.blob_index() == DEBUG_BLOB_NUMBER) {
      printf("blob index %d maxima: %d\n", (int)calculator.blob_index(),
             (int)calculator.peaks_count());
      for (size_t i = 0; i < std::min(kItems, calculator.peaks_count()); ++i) {
        printf("  %d: %d\n", (int)i, calculator.IndexFromMaxima(i));
      }
      printf("Best fit is %d(%d) %d(%d) %d(%d) %d(%d) -> %f, valid? %d\n", i0,
             (int)min_error.m0, i1, (int)min_error.m1, i2, (int)min_error.m2,
             i3, (int)min_error.m3, min_error.error, valid);
    }
#endif
    fit_quads_device[blockIdx.x].indices[0] = i0;
    fit_quads_device[blockIdx.x].indices[1] = i1;
    fit_quads_device[blockIdx.x].indices[2] = i2;
    fit_quads_device[blockIdx.x].indices[3] = i3;
    fit_quads_device[blockIdx.x].moments[0] = calculator.ReadMoments(i0, i1);
    fit_quads_device[blockIdx.x].moments[1] = calculator.ReadMoments(i1, i2);
    fit_quads_device[blockIdx.x].moments[2] = calculator.ReadMoments(i2, i3);
    fit_quads_device[blockIdx.x].moments[3] = calculator.ReadMoments(i3, i0);
  }
}

void FitQuads(
    const Peak *peaks_device, size_t /*peaks*/, const PeakExtents *peak_extents,
    size_t num_extents, const LineFitPoint *line_fit_points_device, int nmaxima,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    float max_line_fit_mse, double cos_critical_rad, FitQuad *fit_quads_device,
    CudaStream *stream) {
  constexpr size_t kThreads = MaxRankedIndex();
  const size_t kBlocks = num_extents;
  VLOG(1) << "Spawning with " << kThreads << " threads, and " << kBlocks
          << " blocks for " << num_extents << " blob_ids";
  CHECK_EQ(nmaxima, kNMaxima)
      << ": Kernel is compiled and optimized for a fixed nmaxima, please "
         "recompile if you want to change it.";
  DoFitQuads<<<kBlocks, kThreads, 0, stream->get()>>>(
      peaks_device, peak_extents, line_fit_points_device,
      selected_extents_device, max_line_fit_mse, cos_critical_rad,
      fit_quads_device);
}

std::ostream &operator<<(std::ostream &os,
                         const frc971::apriltag::LineFitMoments &moments) {
  os << "{Mx:" << std::setprecision(20) << moments.Mx / 2.
     << ", My:" << std::setprecision(20) << moments.My / 2.
     << ", Mxx:" << std::setprecision(20) << moments.Mxx / 4.
     << ", Mxy:" << std::setprecision(20) << moments.Mxy / 4.
     << ", Myy:" << std::setprecision(20) << moments.Myy / 4.
     << ", W:" << std::setprecision(20) << moments.W << ", N:" << moments.N
     << "}";
  return os;
}

}  // namespace frc971::apriltag
