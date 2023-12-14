#ifndef AOS_FLATBUFFERS_STATIC_VECTOR_H_
#define AOS_FLATBUFFERS_STATIC_VECTOR_H_
#include <span>

#include "flatbuffers/base.h"
#include "glog/logging.h"

#include "aos/containers/inlined_vector.h"
#include "aos/containers/sized_array.h"
#include "aos/flatbuffers/base.h"

namespace aos::fbs {

namespace internal {
// Helper class for managing how we specialize the Vector object for different
// contained types.
// Users of the Vector class should never need to care about this.
// Template arguments:
//   T: The type that the vector stores.
//   kInline: Whether the type in question is stored inline or not.
//   Enable: Used for SFINAE around struct values; can be ignored.
// The struct provides the following types:
//   Type: The type of the data that will be stored inline in the vector.
//   ObjectType: The type of the actual data (only used for non-inline objects).
//   FlatbufferType: The type used by flatbuffers::Vector to store this type.
//   ConstFlatbufferType: The type used by a const flatbuffers::Vector to store
//     this type.
//   kDataAlign: Alignment required by the stored type.
//   kDataSize: Nominal size required by each non-inline data member. This is
//     what will be initially allocated; once created, individual members may
//     grow to accommodate dynamically lengthed vectors.
template <typename T, bool kInline, class Enable = void>
struct InlineWrapper;
}  // namespace internal

// This Vector class provides a mutable, resizeable, flatbuffer vector.
//
// Upon creation, the Vector will start with enough space allocated for
// kStaticLength elements, and must be provided with a memory buffer that
// is large enough to serialize all the kStaticLength members (kStaticLength may
// be zero).
//
// Once created, the Vector may be grown using calls to reserve().
// This will result in the Vector attempting to allocate memory via its
// parent object; such calls may fail if there is no space available in the
// allocator.
//
// Note that if you are using the Vector class in a realtime context (and thus
// must avoid dynamic memory allocations) you must only be using a Vector of
// inline data (i.e., scalars, enums, or structs). Flatbuffer tables and strings
// require overhead to manage and so require some form of dynamic memory
// allocation. If we discover a strong use-case for such things, then we may
// provide some interface that allows managing said metadata on the stack or
// in another realtime-safe manner.
//
// Template arguments:
//   T: Type contained by the vector; either a scalar/struct/enum type or a
//      static flatbuffer type of some sort (a String or an implementation of
//      aos::fbs::Table).
//   kStaticLength: Number of elements to statically allocate memory for.
//      May be zero.
//   kInline: Whether the type T will be stored inline in the vector.
//   kForceAlign: Alignment to force for the start of the vector (e.g., for
//     byte arrays it may be desirable to have the entire array aligned).
//   kNullTerminate: Whether to reserve an extra byte past the end of
//     the inline data for null termination. Not included in kStaticLength,
//     so if e.g. you want to store the string "abc" then kStaticLength can
//     be 3 and kNullTerminate can be true and the vector data will take
//     up 4 bytes of memory.
//
// Vector buffer memory layout:
// * Requirements:
//   * Minimum alignment of 4 bytes (for element count).
//   * The start of the vector data must be aligned to either
//     alignof(InlineType) or a user-specified number.
//   * The element count for the vector must immediately precede the vector
//     data (and so may itself not be aligned to alignof(InlineType)).
//   * For non-inlined types, the individual types must be aligned to
//     their own alignment.
// * In order to accommodate this, the vector buffer as a whole must
//   generally be aligned to the greatest of the above alignments. There
//   are two reasonable ways one could do this:
//   * Require that the 4th byte of the buffer provided by aligned to
//     the maximum alignment of its contents.
//   * Require that the buffer itself by aligned, and provide padding
//     ourselves. The Vector would then have to expose its own offset
//     because it would not start at the start of the buffer.
//   The former requires that the wrapping code understand the internals
//   of how vectors work; the latter generates extra padding and adds
//   extra logic around handling non-zero offsets.
//   To maintain general simplicity, we will use the second condition and eat
//   the cost of the potential extra few bytes of padding.
// * The layout of the buffer will thus be:
//   [padding; element_count; inline_data; padding; offset_data]
//   The first padding will be of size max(0, kAlign - 4).
//   The element_count is of size 4.
//   The inline_data is of size sizeof(InlineType) * kStaticLength.
//   The second padding is of size
//       (kAlign - ((sizeof(InlineType) * kStaticLength) % kAlign)).
//   The remaining data is only present if kInline is false.
//   The offset data is of size T::kSize * kStaticLength. T::kSize % T::kAlign
//   must be zero.
//   Note that no padding is required on the end because T::kAlign will always
//   end up being equal to the alignment (this can only be violated if
//   kForceAlign is used, but we do not allow that).
//   The Vector class leaves any padding uninitialized. Until and unless we
//   determine that it is a performance issue, it is the responsibility of the
//   parent of this object to zero-initialize the memory.
template <typename T, size_t kStaticLength, bool kInline,
          size_t kForceAlign = 0, bool kNullTerminate = false>
class Vector : public ResizeableObject {
 public:
  static_assert(kInline || !kNullTerminate,
                "It does not make sense to null-terminate vectors of objects.");
  // Type stored inline in the serialized vector (offsets for tables/strings; T
  // otherwise).
  using InlineType = typename internal::InlineWrapper<T, kInline>::Type;
  // OUt-of-line type for out-of-line T.
  using ObjectType = typename internal::InlineWrapper<T, kInline>::ObjectType;
  // Type used as the template parameter to flatbuffers::Vector<>.
  using FlatbufferType =
      typename internal::InlineWrapper<T, kInline>::FlatbufferType;
  using ConstFlatbufferType =
      typename internal::InlineWrapper<T, kInline>::ConstFlatbufferType;
  // flatbuffers::Vector type that corresponds to this Vector.
  typedef flatbuffers::Vector<FlatbufferType> Flatbuffer;
  typedef const flatbuffers::Vector<ConstFlatbufferType> ConstFlatbuffer;
  // Alignment of the inline data.
  static constexpr size_t kInlineAlign =
      std::max(kForceAlign, alignof(InlineType));
  // Type used for serializing the length of the vector.
  typedef uint32_t LengthType;
  // Overall alignment of this type, and required alignment of the buffer that
  // must be provided to the Vector.
  static constexpr size_t kAlign =
      std::max({alignof(LengthType), kInlineAlign,
                internal::InlineWrapper<T, kInline>::kDataAlign});
  // Padding inserted prior to the length element of the vector (to manage
  // alignment of the data properly; see class comment)
  static constexpr size_t kPadding1 =
      std::max<size_t>(0, kAlign - sizeof(LengthType));
  // Size of the vector length field.
  static constexpr size_t kLengthSize = sizeof(LengthType);
  // Size of all the inline vector data, including null termination (prior to
  // any dynamic increases in size).
  static constexpr size_t kInlineSize =
      sizeof(InlineType) * (kStaticLength + (kNullTerminate ? 1 : 0));
  // Per-element size of any out-of-line data.
  static constexpr size_t kDataElementSize =
      internal::InlineWrapper<T, kInline>::kDataSize;
  // Padding between the inline data and any out-of-line data, to manage
  // mismatches in alignment between the two.
  static constexpr size_t kPadding2 = kAlign - (kInlineSize % kAlign);
  // Total statically allocated space for any out-of-line data ("offset data")
  // (prior to any dynamic increases in size).
  static constexpr size_t kOffsetOffsetDataSize =
      kInline ? 0 : (kStaticLength * kDataElementSize);
  // Total nominal size of the Vector.
  static constexpr size_t kSize =
      kPadding1 + kLengthSize + kInlineSize + kPadding2 + kOffsetOffsetDataSize;
  // Offset from the start of the provided buffer to where the actual start of
  // the vector is.
  static constexpr size_t kOffset = kPadding1;
  // Constructors; the provided buffer must be aligned to kAlign and be kSize in
  // length. parent must be non-null.
  Vector(std::span<uint8_t> buffer, ResizeableObject *parent)
      : ResizeableObject(buffer, parent) {
    CHECK_EQ(0u, reinterpret_cast<size_t>(buffer.data()) % kAlign);
    CHECK_EQ(kSize, buffer.size());
    SetLength(0u);
    if (!kInline) {
      // Initialize the offsets for any sub-tables. These are used to track
      // where each table will get serialized in memory as memory gets
      // resized/moved around.
      for (size_t index = 0; index < kStaticLength; ++index) {
        object_absolute_offsets_.emplace_back(kPadding1 + kLengthSize +
                                              kInlineSize + kPadding2 +
                                              index * kDataElementSize);
      }
    }
  }
  Vector(const Vector &) = delete;
  Vector &operator=(const Vector &) = delete;
  virtual ~Vector() {}
  // Current allocated length of this vector.
  // Does not include null termination.
  size_t capacity() const { return allocated_length_; }
  // Current length of the vector.
  // Does not include null termination.
  size_t size() const { return length_; }

  // Appends an element to the Vector. Used when kInline is false. Returns
  // nullptr if the append failed due to insufficient capacity. If you need to
  // increase the capacity() of the vector, call reserve().
  [[nodiscard]] T *emplace_back();
  // Appends an element to the Vector. Used when kInline is true. Returns false
  // if there is insufficient capacity for a new element.
  [[nodiscard]] bool emplace_back(T element) {
    static_assert(kInline);
    return AddInlineElement(element);
  }

  // Adjusts the allocated size of the vector (does not affect the actual
  // current length as returned by size()). Returns true on success, and false
  // if the allocation failed for some reason.
  // Note that reductions in size will not currently result in the allocated
  // size actually changing.
  [[nodiscard]] bool reserve(size_t new_length) {
    if (new_length > allocated_length_) {
      const size_t new_elements = new_length - allocated_length_;
      // First, we must add space for our new inline elements.
      if (!InsertBytes(
              inline_data() + allocated_length_ + (kNullTerminate ? 1 : 0),
              new_elements * sizeof(InlineType), SetZero::kYes)) {
        return false;
      }
      if (!kInline) {
        // For non-inline objects, create the space required for all the new
        // object data.
        const size_t insertion_point = buffer_.size();
        if (!InsertBytes(buffer_.data() + insertion_point,
                         new_elements * kDataElementSize, SetZero::kYes)) {
          return false;
        }
        for (size_t index = 0; index < new_elements; ++index) {
          // Note that the already-allocated data may be arbitrarily-sized, so
          // we cannot use the same static calculation that we do in the
          // constructor.
          object_absolute_offsets_.emplace_back(insertion_point +
                                                index * kDataElementSize);
        }
        objects_.reserve(new_length);
      }
      allocated_length_ = new_length;
    }
    return true;
  }

  // Accessors for using the Vector as a flatbuffers::Vector.
  // Note that these pointers will be unstable if any memory allocations occur
  // that cause memory to get shifted around.
  ConstFlatbuffer *AsFlatbufferVector() const {
    return reinterpret_cast<const Flatbuffer *>(vector_buffer().data());
  }

  // Copies the contents of the provided vector into this; returns false on
  // failure (e.g., if the provided vector is too long for the amount of space
  // we can allocate through reserve()).
  // This is a deep copy, and will call FromFlatbuffer on any constituent
  // objects.
  [[nodiscard]] bool FromFlatbuffer(ConstFlatbuffer *vector);

  // Returns the element at the provided index. index must be less than size().
  const T &at(size_t index) const {
    CHECK_LT(index, length_);
    return unsafe_at(index);
  }

  // Same as at(), except that bounds checks are only performed in non-optimized
  // builds.
  // TODO(james): The GetInlineElement() call itself does some bounds-checking;
  // consider down-grading that.
  const T &unsafe_at(size_t index) const {
    DCHECK_LT(index, length_);
    if (kInline) {
      // This reinterpret_cast is extremely wrong if T != InlineType (this is
      // fine because we only do this if kInline is true).
      // TODO(james): Get the templating improved so that we can get away with
      // specializing at() instead of using if statements. Resolving this will
      // also allow deduplicating the Resize() calls.
      // This specialization is difficult because you cannot partially
      // specialize a templated class method (online things seem to suggest e.g.
      // using a struct as the template parameter rather than having separate
      // parameters).
      return reinterpret_cast<const T &>(GetInlineElement(index));
    } else {
      return objects_[index].t;
    }
  }

  // Returns a mutable pointer to the element at the provided index. index must
  // be less than size().
  T &at(size_t index) {
    CHECK_LT(index, length_);
    return unsafe_at(index);
  }

  // Same as at(), except that bounds checks are only performed in non-optimized
  // builds.
  // TODO(james): The GetInlineElement() call itself does some bounds-checking;
  // consider down-grading that.
  T &unsafe_at(size_t index) {
    DCHECK_LT(index, length_);
    if (kInline) {
      // This reinterpret_cast is extremely wrong if T != InlineType (this is
      // fine because we only do this if kInline is true).
      // TODO(james): Get the templating improved so that we can get away with
      // specializing at() instead of using if statements. Resolving this will
      // also allow deduplicating the Resize() calls.
      // This specialization is difficult because you cannot partially
      // specialize a templated class method (online things seem to suggest e.g.
      // using a struct as the template parameter rather than having separate
      // parameters).
      return reinterpret_cast<T &>(GetInlineElement(index));
    } else {
      return objects_[index].t;
    }
  }

  const T &operator[](size_t index) const { return at(index); }
  T &operator[](size_t index) { return at(index); }

  // Resizes the vector to the requested size.
  // size must be less than or equal to the current capacity() of the vector.
  // Does not allocate additional memory (call reserve() to allocate additional
  // memory).
  // Zero-initializes all inline element; initializes all subtable/string
  // elements to extant but empty objects.
  void resize(size_t size);

  // Resizes an inline vector to the requested size.
  // When changing the size of the vector, the removed/inserted elements will be
  // set to zero if requested. Otherwise, they will be left uninitialized.
  void resize_inline(size_t size, SetZero set_zero) {
    CHECK_LE(size, allocated_length_);
    static_assert(
        kInline,
        "Vector::resize_inline() only works for inline vector types (scalars, "
        "enums, structs).");
    if (size == length_) {
      return;
    }
    if (set_zero == SetZero::kYes) {
      memset(
          reinterpret_cast<void *>(inline_data() + std::min(size, length_)), 0,
          std::abs(static_cast<ssize_t>(length_) - static_cast<ssize_t>(size)) *
              sizeof(InlineType));
    }
    length_ = size;
    SetLength(length_);
  }
  // Resizes a vector of offsets to the requested size.
  // If the size is increased, the new elements will be initialized
  // to empty but extant objects for non-inlined types (so, zero-length
  // vectors/strings; objects that exist but have no fields populated).
  // Note that this is always equivalent to resize().
  void resize_not_inline(size_t size) {
    CHECK_LE(size, allocated_length_);
    static_assert(!kInline,
                  "Vector::resize_not_inline() only works for offset vector "
                  "types (objects, strings).");
    if (size == length_) {
      return;
    } else if (length_ > size) {
      // TODO: Remove any excess allocated memory.
      length_ = size;
      SetLength(length_);
      return;
    } else {
      while (length_ < size) {
        CHECK_NOTNULL(emplace_back());
      }
    }
  }

  // Accessors directly to the inline data of a vector.
  const T *data() const {
    static_assert(kInline,
                  "If you have a use-case for directly accessing the "
                  "flatbuffer data pointer for vectors of "
                  "objects/strings, please start a discussion.");
    return inline_data();
  }

  T *data() {
    static_assert(kInline,
                  "If you have a use-case for directly accessing the "
                  "flatbuffer data pointer for vectors of "
                  "objects/strings, please start a discussion.");
    return inline_data();
  }

  std::string SerializationDebugString() const {
    std::stringstream str;
    str << "Raw Size: " << kSize << " alignment: " << kAlign
        << " allocated length: " << allocated_length_ << " inline alignment "
        << kInlineAlign << " kPadding1 " << kPadding1 << "\n";
    str << "Observed length " << GetLength() << " (expected " << length_
        << ")\n";
    str << "Inline Size " << kInlineSize << " Inline bytes/value:\n";
    // TODO(james): Get pretty-printing for structs so we can provide better
    // debug.
    internal::DebugBytes(
        internal::GetSubSpan(vector_buffer(), kLengthSize,
                             sizeof(InlineType) * allocated_length_),
        str);
    str << "kPadding2 " << kPadding2 << " offset data size "
        << kOffsetOffsetDataSize << "\n";
    return str.str();
  }

 protected:
  friend struct internal::TableMover<
      Vector<T, kStaticLength, kInline, kForceAlign, kNullTerminate>>;
  // protected so that the String class can access the move constructor.
  Vector(Vector &&) = default;

 private:
  // See kAlign and kOffset.
  size_t Alignment() const final { return kAlign; }
  size_t AbsoluteOffsetOffset() const override { return kOffset; }
  // Returns a buffer that starts at the start of the vector itself (past any
  // padding).
  std::span<uint8_t> vector_buffer() {
    return internal::GetSubSpan(buffer(), kPadding1);
  }
  std::span<const uint8_t> vector_buffer() const {
    return internal::GetSubSpan(buffer(), kPadding1);
  }

  bool AddInlineElement(InlineType e) {
    if (length_ == allocated_length_) {
      return false;
    }
    SetInlineElement(length_, e);
    ++length_;
    SetLength(length_);
    return true;
  }

  void SetInlineElement(size_t index, InlineType value) {
    CHECK_LT(index, allocated_length_);
    inline_data()[index] = value;
  }

  InlineType &GetInlineElement(size_t index) {
    CHECK_LT(index, allocated_length_);
    return inline_data()[index];
  }

  const InlineType &GetInlineElement(size_t index) const {
    CHECK_LT(index, allocated_length_);
    return inline_data()[index];
  }

  // Returns a pointer to the start of the inline data itself.
  InlineType *inline_data() {
    return reinterpret_cast<InlineType *>(vector_buffer().data() + kLengthSize);
  }
  const InlineType *inline_data() const {
    return reinterpret_cast<const InlineType *>(vector_buffer().data() +
                                                kLengthSize);
  }

  // Updates the length of the vector to match the provided length. Does not set
  // the length_ member.
  void SetLength(LengthType length) {
    *reinterpret_cast<LengthType *>(vector_buffer().data()) = length;
    if (kNullTerminate) {
      memset(reinterpret_cast<void *>(inline_data() + length), 0,
             sizeof(InlineType));
    }
  }
  LengthType GetLength() const {
    return *reinterpret_cast<const LengthType *>(vector_buffer().data());
  }

  // Overrides to allow ResizeableObject to manage memory adjustments.
  size_t NumberOfSubObjects() const final {
    return kInline ? 0 : allocated_length_;
  }
  using ResizeableObject::SubObject;
  SubObject GetSubObject(size_t index) final {
    return SubObject{
        reinterpret_cast<uoffset_t *>(&GetInlineElement(index)),
        // In order to let this compile regardless of whether type T is an
        // object type or not, we just use a reinterpret_cast.
        (index < length_)
            ? reinterpret_cast<ResizeableObject *>(&objects_[index].t)
            : nullptr,
        &object_absolute_offsets_[index]};
  }
  // Implementation that handles copying from a flatbuffers::Vector of an inline
  // data type.
  [[nodiscard]] bool FromInlineFlatbuffer(ConstFlatbuffer *vector) {
    if (!reserve(CHECK_NOTNULL(vector)->size())) {
      return false;
    }

    // We will be overwriting the whole vector very shortly; there is no need to
    // clear the buffer to zero.
    resize_inline(vector->size(), SetZero::kNo);

    memcpy(inline_data(), vector->Data(), size() * sizeof(InlineType));
    return true;
  }

  // Implementation that handles copying from a flatbuffers::Vector of a
  // not-inline data type.
  [[nodiscard]] bool FromNotInlineFlatbuffer(const Flatbuffer *vector) {
    if (!reserve(vector->size())) {
      return false;
    }
    // "Clear" the vector.
    resize_not_inline(0);

    for (const typename T::Flatbuffer *entry : *vector) {
      if (!CHECK_NOTNULL(emplace_back())->FromFlatbuffer(entry)) {
        return false;
      }
    }
    return true;
  }

  // In order to allow for easy partial template specialization, we use a
  // non-member class to call FromInline/FromNotInlineFlatbuffer and
  // resize_inline/resize_not_inline. There are not actually any great ways to
  // do this with just our own class member functions, so instead we make these
  // methods members of a friend of the Vector class; we then partially
  // specialize the entire InlineWrapper class and use it to isolate anything
  // that needs to have a common user interface while still having separate
  // actual logic.
  template <typename T_, bool kInline_, class Enable_>
  friend struct internal::InlineWrapper;

  // Note: The objects here really want to be owned by this object (as opposed
  // to e.g. returning a stack-allocated object from the emplace_back() methods
  // that the user then owns). There are two main challenges with have the user
  // own the object on question:
  // 1. We can't have >1 reference floating around, or else one object's state
  //    can become out of date. This forces us to do ref-counting and could
  //    make certain types of code obnoxious to write.
  // 2. Once the user-created object goes out of scope, we lose all of its
  //    internal state. In _theory_ it should be possible to reconstruct most
  //    of the relevant state by examining the contents of the buffer, but
  //    doing so would be cumbersome.
  aos::InlinedVector<internal::TableMover<ObjectType>,
                     kInline ? 0 : kStaticLength>
      objects_;
  aos::InlinedVector<size_t, kInline ? 0 : kStaticLength>
      object_absolute_offsets_;
  // Current actual length of the vector.
  size_t length_ = 0;
  // Current length that we have allocated space available for.
  size_t allocated_length_ = kStaticLength;
};

template <typename T, size_t kStaticLength, bool kInline, size_t kForceAlign,
          bool kNullTerminate>
T *Vector<T, kStaticLength, kInline, kForceAlign,
          kNullTerminate>::emplace_back() {
  static_assert(!kInline);
  if (length_ >= allocated_length_) {
    return nullptr;
  }
  const size_t object_start = object_absolute_offsets_[length_];
  std::span<uint8_t> object_buffer =
      internal::GetSubSpan(buffer(), object_start, T::kSize);
  objects_.emplace_back(object_buffer, this);
  const uoffset_t offset =
      object_start - (reinterpret_cast<size_t>(&GetInlineElement(length_)) -
                      reinterpret_cast<size_t>(buffer().data()));
  CHECK(AddInlineElement(offset));
  return &objects_[objects_.size() - 1].t;
}

// The String class is a special version of the Vector that is always
// null-terminated, always contains 1-byte character elements, and which has a
// few extra methods for convenient string access.
template <size_t kStaticLength>
class String : public Vector<char, kStaticLength, true, 0, true> {
 public:
  typedef Vector<char, kStaticLength, true, 0, true> VectorType;
  typedef flatbuffers::String Flatbuffer;
  String(std::span<uint8_t> buffer, ResizeableObject *parent)
      : VectorType(buffer, parent) {}
  virtual ~String() {}
  void SetString(std::string_view string) {
    CHECK_LT(string.size(), VectorType::capacity());
    VectorType::resize_inline(string.size(), SetZero::kNo);
    memcpy(VectorType::data(), string.data(), string.size());
  }
  std::string_view string_view() const {
    return std::string_view(VectorType::data(), VectorType::size());
  }
  std::string str() const {
    return std::string(VectorType::data(), VectorType::size());
  }
  const char *c_str() const { return VectorType::data(); }

 private:
  friend struct internal::TableMover<String<kStaticLength>>;
  String(String &&) = default;
};

namespace internal {
// Specialization for all non-inline vector types. All of these types will just
// use offsets for their inline data and have appropriate member types/constants
// for the remaining fields.
template <typename T>
struct InlineWrapper<T, false, void> {
  typedef uoffset_t Type;
  typedef T ObjectType;
  typedef flatbuffers::Offset<typename T::Flatbuffer> FlatbufferType;
  typedef flatbuffers::Offset<typename T::Flatbuffer> ConstFlatbufferType;
  static_assert((T::kSize % T::kAlign) == 0);
  static constexpr size_t kDataAlign = T::kAlign;
  static constexpr size_t kDataSize = T::kSize;
  template <typename StaticVector>
  static bool FromFlatbuffer(
      StaticVector *to, const typename StaticVector::ConstFlatbuffer *from) {
    return to->FromNotInlineFlatbuffer(from);
  }
  template <typename StaticVector>
  static void ResizeVector(StaticVector *target, size_t size) {
    target->resize_not_inline(size);
  }
};
// Specialization for "normal" scalar inline data (ints, floats, doubles,
// enums).
template <typename T>
struct InlineWrapper<T, true,
                     typename std::enable_if_t<!std::is_class<T>::value>> {
  typedef T Type;
  typedef T ObjectType;
  typedef T FlatbufferType;
  typedef T ConstFlatbufferType;
  static constexpr size_t kDataAlign = alignof(T);
  static constexpr size_t kDataSize = sizeof(T);
  template <typename StaticVector>
  static bool FromFlatbuffer(
      StaticVector *to, const typename StaticVector::ConstFlatbuffer *from) {
    return to->FromInlineFlatbuffer(from);
  }
  template <typename StaticVector>
  static void ResizeVector(StaticVector *target, size_t size) {
    target->resize_inline(size, SetZero::kYes);
  }
};
// Specialization for booleans, given that flatbuffers uses uint8_t's for bools.
template <>
struct InlineWrapper<bool, true, void> {
  typedef uint8_t Type;
  typedef uint8_t ObjectType;
  typedef uint8_t FlatbufferType;
  typedef uint8_t ConstFlatbufferType;
  static constexpr size_t kDataAlign = 1u;
  static constexpr size_t kDataSize = 1u;
  template <typename StaticVector>
  static bool FromFlatbuffer(
      StaticVector *to, const typename StaticVector::ConstFlatbuffer *from) {
    return to->FromInlineFlatbuffer(from);
  }
  template <typename StaticVector>
  static void ResizeVector(StaticVector *target, size_t size) {
    target->resize_inline(size, SetZero::kYes);
  }
};
// Specialization for flatbuffer structs.
// The flatbuffers codegen uses struct pointers rather than references or the
// such, so it needs to be treated special.
template <typename T>
struct InlineWrapper<T, true,
                     typename std::enable_if_t<std::is_class<T>::value>> {
  typedef T Type;
  typedef T ObjectType;
  typedef T *FlatbufferType;
  typedef const T *ConstFlatbufferType;
  static constexpr size_t kDataAlign = alignof(T);
  static constexpr size_t kDataSize = sizeof(T);
  template <typename StaticVector>
  static bool FromFlatbuffer(
      StaticVector *to, const typename StaticVector::ConstFlatbuffer *from) {
    return to->FromInlineFlatbuffer(from);
  }
  template <typename StaticVector>
  static void ResizeVector(StaticVector *target, size_t size) {
    target->resize_inline(size, SetZero::kYes);
  }
};
}  // namespace internal
   //
template <typename T, size_t kStaticLength, bool kInline, size_t kForceAlign,
          bool kNullTerminate>
bool Vector<T, kStaticLength, kInline, kForceAlign,
            kNullTerminate>::FromFlatbuffer(ConstFlatbuffer *vector) {
  return internal::InlineWrapper<T, kInline>::FromFlatbuffer(this, vector);
}

template <typename T, size_t kStaticLength, bool kInline, size_t kForceAlign,
          bool kNullTerminate>
void Vector<T, kStaticLength, kInline, kForceAlign, kNullTerminate>::resize(
    size_t size) {
  internal::InlineWrapper<T, kInline>::ResizeVector(this, size);
}

}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_STATIC_VECTOR_H_
