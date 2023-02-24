#ifndef AOS_UTIL_ERROR_COUNTER_H_
#define AOS_UTIL_ERROR_COUNTER_H_
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

namespace aos::util {
// Class to manage simple error counters for flatbuffer status message.
// This presumes that you have a flatbuffer enum type Error which has
// enum values that are continuous and start at zero. These are then
// counted by a Count flatbuffer table that is of the format:
// table Count {
//   error:Error (id: 0);
//   count:uint (id: 1);
// }
// And which is stored as a vector in the resulting status message,
// where the index within the vector corresponds with the underlying
// value of the enum.
template <typename Error, typename Count>
class ErrorCounter {
 public:
  static constexpr size_t kNumErrors =
      static_cast<int>(Error::MAX) - static_cast<int>(Error::MIN) + 1;
  static_assert(0 == static_cast<int>(Error::MIN),
                "Expected Error enum values to start at zero.");
  // TODO(james): Is there any good way to check that the values are contiguous?
  // There's no Error::COUNT, and the method I previously used (checking the
  // size of the return type of EnumValues*()) requires the user to pass that
  // method as a template argument.
  ErrorCounter() = default;
  static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Count>>>
  Initialize(flatbuffers::FlatBufferBuilder *fbb) {
    std::array<flatbuffers::Offset<Count>, kNumErrors> count_offsets;
    for (size_t ii = 0; ii < kNumErrors; ++ii) {
      typename Count::Builder builder(*fbb);
      builder.add_error(static_cast<Error>(ii));
      builder.add_count(0);
      count_offsets[ii] = builder.Finish();
    }
    const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Count>>>
        offset = fbb->CreateVector(count_offsets.data(), count_offsets.size());
    return offset;
  }

  void set_mutable_vector(
      flatbuffers::Vector<flatbuffers::Offset<Count>> *vector) {
    vector_ = vector;
  }

  void InvalidateBuffer() { vector_ = nullptr; }

  void IncrementError(Error error) {
    CHECK_NOTNULL(vector_);
    DCHECK_LT(static_cast<size_t>(error), vector_->size());
    Count *counter = vector_->GetMutableObject(static_cast<size_t>(error));
    counter->mutate_count(counter->count() + 1);
  }

  // Sets all the error counts to zero.
  void ResetCounts() {
    CHECK_NOTNULL(vector_);
    DCHECK_EQ(vector_->size(), kNumErrors) << this << " vector " << vector_;
    for (size_t ii = 0; ii < kNumErrors; ++ii) {
      vector_->GetMutableObject(ii)->mutate_count(0);
    }
  }

 private:
  flatbuffers::Vector<flatbuffers::Offset<Count>> *vector_ = nullptr;
};

// The ArrayErrorCounter serves the same purpose as the ErrorCounter class,
// except that:
// (a) It owns its own memory, rather than modifying a flatbuffer in-place.
// (b) Because of this, the user has greater flexibility in choosing when to
//     reset the error counters.
template <typename Error, typename Count>
class ArrayErrorCounter {
 public:
  static constexpr size_t kNumErrors = ErrorCounter<Error, Count>::kNumErrors;
  ArrayErrorCounter() { ResetCounts(); }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Count>>>
  PopulateCounts(flatbuffers::FlatBufferBuilder *fbb) const {
    const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Count>>>
        offset = ErrorCounter<Error, Count>::Initialize(fbb);
    flatbuffers::Vector<flatbuffers::Offset<Count>> *vector =
        flatbuffers::GetMutableTemporaryPointer(*fbb, offset);
    for (size_t ii = 0; ii < kNumErrors; ++ii) {
      vector->GetMutableObject(ii)->mutate_count(error_counts_.at(ii));
    }
    return offset;
  }

  void IncrementError(Error error) {
    DCHECK_LT(static_cast<size_t>(error), error_counts_.size());
    error_counts_.at(static_cast<size_t>(error))++;
  }

  // Sets all the error counts to zero.
  void ResetCounts() { error_counts_.fill(0); }

 private:
  std::array<size_t, kNumErrors> error_counts_;
};
}  // namespace aos::util
#endif  // AOS_UTIL_ERROR_COUNTER_H_
