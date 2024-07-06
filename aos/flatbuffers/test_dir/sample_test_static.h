#pragma once
// This is a generated file. Do not modify.
#include <optional>

#include "aos/flatbuffers/static_table.h"
#include "aos/flatbuffers/static_vector.h"
#include "aos/flatbuffers/test_dir/include_generated.h"
#include "aos/flatbuffers/test_dir/include_static.h"
#include "aos/flatbuffers/test_generated.h"
#include "aos/flatbuffers/test_static.h"

namespace aos::fbs::testing {
class MinimallyAlignedTableStatic : public ::aos::fbs::Table {
 public:
  // The underlying "raw" flatbuffer type for this type.
  typedef aos::fbs::testing::MinimallyAlignedTable Flatbuffer;
  typedef flatbuffers::unique_ptr<Flatbuffer::NativeTableType>
      FlatbufferObjectType;
  // Returns this object as a flatbuffer type. This reference may not be valid
  // following mutations to the underlying flatbuffer, due to how memory may get
  // may get moved around.
  const Flatbuffer &AsFlatbuffer() const {
    return *GetFlatbuffer<Flatbuffer>();
  }

  // Space taken up by the inline portion of the flatbuffer table data, in
  // bytes.
  static constexpr size_t kInlineDataSize = 5;
  // Space taken up by the vtable for this object, in bytes.
  static constexpr size_t kVtableSize =
      sizeof(::flatbuffers::voffset_t) * (2 + 1);
  // Offset from the start of the internal memory buffer to the start of the
  // vtable.
  static constexpr size_t kVtableStart = ::aos::fbs::AlignOffset(
      kInlineDataSize, alignof(::flatbuffers::voffset_t));
  // Required alignment of this object. The buffer that this object gets
  // constructed into must be aligned to this value.
  static constexpr size_t kAlign = std::max<size_t>({kMinAlign, 1});

  // Offset into this object to measure the alignment at.
  static constexpr size_t kAlignOffset = sizeof(::flatbuffers::soffset_t);
  static_assert(
      1 <= kAlign,
      "Flatbuffer schema minalign should not exceed our required alignment.");
  // Offset from the start of the memory buffer to the start of any out-of-line
  // data (subtables, vectors, strings).
  static constexpr size_t kOffsetDataStart = (kVtableStart + kVtableSize);
  // Various overrides to support the Table parent class.
  size_t FixedVtableOffset() const final { return kVtableStart; }
  size_t VtableSize() const final { return kVtableSize; }
  size_t InlineTableSize() const final { return kInlineDataSize; }
  size_t OffsetDataStart() const final { return kOffsetDataStart; }
  size_t Alignment() const final { return kAlign; }
  // Exposes the name of the flatbuffer type to allow interchangeable use
  // of the Flatbuffer and FlatbufferStatic types in various AOS methods.
  static const char *GetFullyQualifiedName() {
    return Flatbuffer::GetFullyQualifiedName();
  }

  // Constructors for creating a flatbuffer object.
  // Users should typically use the Builder class to create these objects,
  // in order to allow it to populate the root table offset.

  // The buffer provided to these constructors should be aligned to kAlign
  // and kSize in length.
  // The parent/allocator may not be nullptr.
  MinimallyAlignedTableStatic(std::span<uint8_t> buffer,
                              ::aos::fbs::ResizeableObject *parent)
      : Table(buffer, parent) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  MinimallyAlignedTableStatic(std::span<uint8_t> buffer,
                              ::aos::fbs::Allocator *allocator)
      : Table(buffer, allocator) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  MinimallyAlignedTableStatic(
      std::span<uint8_t> buffer,
      ::std::unique_ptr<::aos::fbs::Allocator> allocator)
      : Table(buffer, ::std::move(allocator)) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }

  virtual ~MinimallyAlignedTableStatic() {}

  // Sets the field field, causing it to be populated if it is not already.
  // This will populate the field even if the specified value is the default.
  void set_field(const uint8_t &value) {
    SetField<uint8_t>(kInlineAbsoluteOffset_field, 4, value);
  }

  // Returns the value of field if set; nullopt otherwise.
  std::optional<uint8_t> field() const {
    return has_field()
               ? std::make_optional(Get<uint8_t>(kInlineAbsoluteOffset_field))
               : std::nullopt;
  }
  // Returns a pointer to modify the field field.
  // The pointer may be invalidated by mutations/movements of the underlying
  // buffer. Returns nullptr if the field is not set.
  uint8_t *mutable_field() {
    return has_field() ? MutableGet<uint8_t>(kInlineAbsoluteOffset_field)
                       : nullptr;
  }

  // Clears the field field. This will cause has_field() to return false.
  void clear_field() { ClearField(kInlineAbsoluteOffset_field, 1, 4); }

  // Returns true if the field field is set and can be accessed.
  bool has_field() const { return AsFlatbuffer().has_field(); }

  // Clears every field of the table, removing any existing state.
  void Clear() { clear_field(); }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // This is a deep copy, and will call FromFlatbuffer on any constituent
  // objects.
  [[nodiscard]] bool FromFlatbuffer([[maybe_unused]] const Flatbuffer &other) {
    Clear();

    if (other.has_field()) {
      set_field(other.field());
    }

    return true;
  }
  // Equivalent to FromFlatbuffer(const Flatbuffer&); this overload is provided
  // to ease implementation of the aos::fbs::Vector internals.
  [[nodiscard]] bool FromFlatbuffer(const Flatbuffer *other) {
    CHECK(other != nullptr);
    return FromFlatbuffer(*other);
  }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // Because the Flatbuffer Object API does not provide any concept of an
  // optionally populated scalar field, all scalar fields will be populated
  // after a call to FromFlatbufferObject().
  // This is a deep copy, and will call FromFlatbufferObject on
  // any constituent objects.
  [[nodiscard]] bool FromFlatbuffer(
      [[maybe_unused]] const Flatbuffer::NativeTableType &other) {
    Clear();

    set_field(other.field);

    return true;
  }
  [[nodiscard]] bool FromFlatbuffer(
      const flatbuffers::unique_ptr<Flatbuffer::NativeTableType> &other) {
    return FromFlatbuffer(*other);
  }

 private:
  // We need to provide a MoveConstructor to allow this table to be
  // used inside of vectors, but we do not want it readily available to
  // users. See TableMover for more details.
  MinimallyAlignedTableStatic(MinimallyAlignedTableStatic &&) = default;
  friend struct ::aos::fbs::internal::TableMover<MinimallyAlignedTableStatic>;

  // Offset from the start of the buffer to the inline data for the field field.
  static constexpr size_t kInlineAbsoluteOffset_field = 4;

  // This object has no non-inline subobjects, so we don't have to do anything
  // special.
  size_t NumberOfSubObjects() const final { return 0; }
  using ::aos::fbs::ResizeableObject::SubObject;
  SubObject GetSubObject(size_t) final { LOG(FATAL) << "No subobjects."; }

 public:
  // Nominal size of this object, in bytes. The object may grow beyond this
  // size, but will always start at this size and so the initial buffer must
  // match this size.
  static constexpr size_t kSize =
      ::aos::fbs::AlignOffset((kVtableStart + kVtableSize) - kAlignOffset,
                              kAlign, kAlignOffset) +
      kAlignOffset;
  // Always statically allocate memory for tables (set for consistency with
  // static_vector.h).
  static constexpr size_t kPreallocatedSize = kSize;
  // Size required for a buffer that includes a root table offset at the start.
  static constexpr size_t kRootSize =
      ::aos::fbs::AlignOffset(kSize + sizeof(::flatbuffers::uoffset_t), kAlign);
};
}  // namespace aos::fbs::testing

namespace aos::fbs::testing {
class SubTableStatic : public ::aos::fbs::Table {
 public:
  // The underlying "raw" flatbuffer type for this type.
  typedef aos::fbs::testing::SubTable Flatbuffer;
  typedef flatbuffers::unique_ptr<Flatbuffer::NativeTableType>
      FlatbufferObjectType;
  // Returns this object as a flatbuffer type. This reference may not be valid
  // following mutations to the underlying flatbuffer, due to how memory may get
  // may get moved around.
  const Flatbuffer &AsFlatbuffer() const {
    return *GetFlatbuffer<Flatbuffer>();
  }

  // Space taken up by the inline portion of the flatbuffer table data, in
  // bytes.
  static constexpr size_t kInlineDataSize = 10;
  // Space taken up by the vtable for this object, in bytes.
  static constexpr size_t kVtableSize =
      sizeof(::flatbuffers::voffset_t) * (2 + 3);
  // Offset from the start of the internal memory buffer to the start of the
  // vtable.
  static constexpr size_t kVtableStart = ::aos::fbs::AlignOffset(
      kInlineDataSize, alignof(::flatbuffers::voffset_t));
  // Required alignment of this object. The buffer that this object gets
  // constructed into must be aligned to this value.
  static constexpr size_t kAlign = std::max<size_t>({kMinAlign, 4, 2});

  // Offset into this object to measure the alignment at.
  static constexpr size_t kAlignOffset = sizeof(::flatbuffers::soffset_t);
  static_assert(
      1 <= kAlign,
      "Flatbuffer schema minalign should not exceed our required alignment.");
  // Offset from the start of the memory buffer to the start of any out-of-line
  // data (subtables, vectors, strings).
  static constexpr size_t kOffsetDataStart = (kVtableStart + kVtableSize);
  // Various overrides to support the Table parent class.
  size_t FixedVtableOffset() const final { return kVtableStart; }
  size_t VtableSize() const final { return kVtableSize; }
  size_t InlineTableSize() const final { return kInlineDataSize; }
  size_t OffsetDataStart() const final { return kOffsetDataStart; }
  size_t Alignment() const final { return kAlign; }
  // Exposes the name of the flatbuffer type to allow interchangeable use
  // of the Flatbuffer and FlatbufferStatic types in various AOS methods.
  static const char *GetFullyQualifiedName() {
    return Flatbuffer::GetFullyQualifiedName();
  }

  // Constructors for creating a flatbuffer object.
  // Users should typically use the Builder class to create these objects,
  // in order to allow it to populate the root table offset.

  // The buffer provided to these constructors should be aligned to kAlign
  // and kSize in length.
  // The parent/allocator may not be nullptr.
  SubTableStatic(std::span<uint8_t> buffer,
                 ::aos::fbs::ResizeableObject *parent)
      : Table(buffer, parent) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  SubTableStatic(std::span<uint8_t> buffer, ::aos::fbs::Allocator *allocator)
      : Table(buffer, allocator) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  SubTableStatic(std::span<uint8_t> buffer,
                 ::std::unique_ptr<::aos::fbs::Allocator> allocator)
      : Table(buffer, ::std::move(allocator)) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }

  virtual ~SubTableStatic() {}

  // Sets the baz field, causing it to be populated if it is not already.
  // This will populate the field even if the specified value is the default.
  void set_baz(const float &value) {
    SetField<float>(kInlineAbsoluteOffset_baz, 8, value);
  }

  // Returns the value of baz if set; nullopt otherwise.
  std::optional<float> baz() const {
    return has_baz() ? std::make_optional(Get<float>(kInlineAbsoluteOffset_baz))
                     : std::nullopt;
  }
  // Returns a pointer to modify the baz field.
  // The pointer may be invalidated by mutations/movements of the underlying
  // buffer. Returns nullptr if the field is not set.
  float *mutable_baz() {
    return has_baz() ? MutableGet<float>(kInlineAbsoluteOffset_baz) : nullptr;
  }

  // Clears the baz field. This will cause has_baz() to return false.
  void clear_baz() { ClearField(kInlineAbsoluteOffset_baz, 4, 8); }

  // Returns true if the baz field is set and can be accessed.
  bool has_baz() const { return AsFlatbuffer().has_baz(); }

  // Sets the foo field, causing it to be populated if it is not already.
  // This will populate the field even if the specified value is the default.
  void set_foo(const int16_t &value) {
    SetField<int16_t>(kInlineAbsoluteOffset_foo, 4, value);
  }

  // Returns the value of foo if set; nullopt otherwise.
  std::optional<int16_t> foo() const {
    return has_foo()
               ? std::make_optional(Get<int16_t>(kInlineAbsoluteOffset_foo))
               : std::nullopt;
  }
  // Returns a pointer to modify the foo field.
  // The pointer may be invalidated by mutations/movements of the underlying
  // buffer. Returns nullptr if the field is not set.
  int16_t *mutable_foo() {
    return has_foo() ? MutableGet<int16_t>(kInlineAbsoluteOffset_foo) : nullptr;
  }

  // Clears the foo field. This will cause has_foo() to return false.
  void clear_foo() { ClearField(kInlineAbsoluteOffset_foo, 2, 4); }

  // Returns true if the foo field is set and can be accessed.
  bool has_foo() const { return AsFlatbuffer().has_foo(); }

  // Clears every field of the table, removing any existing state.
  void Clear() {
    clear_baz();
    clear_foo();
  }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // This is a deep copy, and will call FromFlatbuffer on any constituent
  // objects.
  [[nodiscard]] bool FromFlatbuffer([[maybe_unused]] const Flatbuffer &other) {
    Clear();

    if (other.has_baz()) {
      set_baz(other.baz());
    }

    if (other.has_foo()) {
      set_foo(other.foo());
    }

    return true;
  }
  // Equivalent to FromFlatbuffer(const Flatbuffer&); this overload is provided
  // to ease implementation of the aos::fbs::Vector internals.
  [[nodiscard]] bool FromFlatbuffer(const Flatbuffer *other) {
    CHECK(other != nullptr);
    return FromFlatbuffer(*other);
  }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // Because the Flatbuffer Object API does not provide any concept of an
  // optionally populated scalar field, all scalar fields will be populated
  // after a call to FromFlatbufferObject().
  // This is a deep copy, and will call FromFlatbufferObject on
  // any constituent objects.
  [[nodiscard]] bool FromFlatbuffer(
      [[maybe_unused]] const Flatbuffer::NativeTableType &other) {
    Clear();

    set_baz(other.baz);

    set_foo(other.foo);

    return true;
  }
  [[nodiscard]] bool FromFlatbuffer(
      const flatbuffers::unique_ptr<Flatbuffer::NativeTableType> &other) {
    return FromFlatbuffer(*other);
  }

 private:
  // We need to provide a MoveConstructor to allow this table to be
  // used inside of vectors, but we do not want it readily available to
  // users. See TableMover for more details.
  SubTableStatic(SubTableStatic &&) = default;
  friend struct ::aos::fbs::internal::TableMover<SubTableStatic>;

  // Offset from the start of the buffer to the inline data for the baz field.
  static constexpr size_t kInlineAbsoluteOffset_baz = 4;

  // Offset from the start of the buffer to the inline data for the foo field.
  static constexpr size_t kInlineAbsoluteOffset_foo = 8;

  // This object has no non-inline subobjects, so we don't have to do anything
  // special.
  size_t NumberOfSubObjects() const final { return 0; }
  using ::aos::fbs::ResizeableObject::SubObject;
  SubObject GetSubObject(size_t) final { LOG(FATAL) << "No subobjects."; }

 public:
  // Nominal size of this object, in bytes. The object may grow beyond this
  // size, but will always start at this size and so the initial buffer must
  // match this size.
  static constexpr size_t kSize =
      ::aos::fbs::AlignOffset((kVtableStart + kVtableSize) - kAlignOffset,
                              kAlign, kAlignOffset) +
      kAlignOffset;
  // Always statically allocate memory for tables (set for consistency with
  // static_vector.h).
  static constexpr size_t kPreallocatedSize = kSize;
  // Size required for a buffer that includes a root table offset at the start.
  static constexpr size_t kRootSize =
      ::aos::fbs::AlignOffset(kSize + sizeof(::flatbuffers::uoffset_t), kAlign);
};
}  // namespace aos::fbs::testing

namespace aos::fbs::testing {
class TestTableStatic : public ::aos::fbs::Table {
 public:
  // The underlying "raw" flatbuffer type for this type.
  typedef aos::fbs::testing::TestTable Flatbuffer;
  typedef flatbuffers::unique_ptr<Flatbuffer::NativeTableType>
      FlatbufferObjectType;
  // Returns this object as a flatbuffer type. This reference may not be valid
  // following mutations to the underlying flatbuffer, due to how memory may get
  // may get moved around.
  const Flatbuffer &AsFlatbuffer() const {
    return *GetFlatbuffer<Flatbuffer>();
  }

  // Space taken up by the inline portion of the flatbuffer table data, in
  // bytes.
  static constexpr size_t kInlineDataSize = 68;
  // Space taken up by the vtable for this object, in bytes.
  static constexpr size_t kVtableSize =
      sizeof(::flatbuffers::voffset_t) * (2 + 13);
  // Offset from the start of the internal memory buffer to the start of the
  // vtable.
  static constexpr size_t kVtableStart = ::aos::fbs::AlignOffset(
      kInlineDataSize, alignof(::flatbuffers::voffset_t));
  // Required alignment of this object. The buffer that this object gets
  // constructed into must be aligned to this value.
  static constexpr size_t kAlign = std::max<size_t>(
      {kMinAlign, 8,
       ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>::kAlign,
       ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>::kAlign,
       ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false,
                          0>::kAlign,
       ::aos::fbs::Vector<int32_t, 3, true, 64>::kAlign,
       ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>::kAlign,
       ::aos::fbs::Vector<int32_t, 3, true, 0>::kAlign,
       ::aos::fbs::String<0>::kAlign,
       ::aos::fbs::Vector<uint8_t, 0, true, 0>::kAlign,
       aos::fbs::testing::included::IncludedTableStatic::kAlign,
       aos::fbs::testing::SubTableStatic::kAlign,
       ::aos::fbs::String<20>::kAlign, 4});

  // Offset into this object to measure the alignment at.
  static constexpr size_t kAlignOffset = sizeof(::flatbuffers::soffset_t);
  static_assert(
      1 <= kAlign,
      "Flatbuffer schema minalign should not exceed our required alignment.");
  // Offset from the start of the memory buffer to the start of any out-of-line
  // data (subtables, vectors, strings).
  static constexpr size_t kOffsetDataStart = (kVtableStart + kVtableSize);
  // Various overrides to support the Table parent class.
  size_t FixedVtableOffset() const final { return kVtableStart; }
  size_t VtableSize() const final { return kVtableSize; }
  size_t InlineTableSize() const final { return kInlineDataSize; }
  size_t OffsetDataStart() const final { return kOffsetDataStart; }
  size_t Alignment() const final { return kAlign; }
  // Exposes the name of the flatbuffer type to allow interchangeable use
  // of the Flatbuffer and FlatbufferStatic types in various AOS methods.
  static const char *GetFullyQualifiedName() {
    return Flatbuffer::GetFullyQualifiedName();
  }

  // Constructors for creating a flatbuffer object.
  // Users should typically use the Builder class to create these objects,
  // in order to allow it to populate the root table offset.

  // The buffer provided to these constructors should be aligned to kAlign
  // and kSize in length.
  // The parent/allocator may not be nullptr.
  TestTableStatic(std::span<uint8_t> buffer,
                  ::aos::fbs::ResizeableObject *parent)
      : Table(buffer, parent) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  TestTableStatic(std::span<uint8_t> buffer, ::aos::fbs::Allocator *allocator)
      : Table(buffer, allocator) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }
  TestTableStatic(std::span<uint8_t> buffer,
                  ::std::unique_ptr<::aos::fbs::Allocator> allocator)
      : Table(buffer, ::std::move(allocator)) {
    CHECK_EQ(buffer.size(), kSize);
    CHECK_EQ(0u,
             reinterpret_cast<size_t>(buffer.data() + kAlignOffset) % kAlign);
    PopulateVtable();
  }

  virtual ~TestTableStatic() {}

  // Sets the substruct field, causing it to be populated if it is not already.
  // This will populate the field even if the specified value is the default.
  void set_substruct(const aos::fbs::testing::SubStruct &value) {
    SetField<aos::fbs::testing::SubStruct>(kInlineAbsoluteOffset_substruct, 12,
                                           value);
  }

  // Returns the value of substruct if set; nullopt otherwise.
  std::optional<aos::fbs::testing::SubStruct> substruct() const {
    return has_substruct()
               ? std::make_optional(Get<aos::fbs::testing::SubStruct>(
                     kInlineAbsoluteOffset_substruct))
               : std::nullopt;
  }
  // Returns a pointer to modify the substruct field.
  // The pointer may be invalidated by mutations/movements of the underlying
  // buffer. Returns nullptr if the field is not set.
  aos::fbs::testing::SubStruct *mutable_substruct() {
    return has_substruct() ? MutableGet<aos::fbs::testing::SubStruct>(
                                 kInlineAbsoluteOffset_substruct)
                           : nullptr;
  }

  // Clears the substruct field. This will cause has_substruct() to return
  // false.
  void clear_substruct() {
    ClearField(kInlineAbsoluteOffset_substruct, 16, 12);
  }

  // Returns true if the substruct field is set and can be accessed.
  bool has_substruct() const { return AsFlatbuffer().has_substruct(); }

  // Creates an empty object for the vector_of_structs field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0> *
  add_vector_of_structs() {
    CHECK(!vector_of_structs_.has_value());
    constexpr size_t kVtableIndex = 18;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true,
                                     0>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_vector_of_structs;
      std::optional<std::span<uint8_t>> inserted_bytes = InsertBytes(
          buffer().data() + object_absolute_offset,
          ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>::kSize,
          ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_vector_of_structs = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_structs_.emplace(
          BufferForObject(
              object_absolute_offset_vector_of_structs,
              ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>::
                  RoundedLength(inserted_bytes.value().size())),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_structs_.emplace(
          BufferForObject(object_absolute_offset_vector_of_structs,
                          ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3,
                                             true, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_vector_of_structs, kVtableIndex,
        object_absolute_offset_vector_of_structs -
            kInlineAbsoluteOffset_vector_of_structs);
    return &vector_of_structs_.value().t;
  }

  // Returns a pointer to the vector_of_structs field, if set. nullptr
  // otherwise.
  const ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0> *
  vector_of_structs() const {
    return vector_of_structs_.has_value() ? &vector_of_structs_.value().t
                                          : nullptr;
  }
  ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0> *
  mutable_vector_of_structs() {
    return vector_of_structs_.has_value() ? &vector_of_structs_.value().t
                                          : nullptr;
  }

  // Clears the vector_of_structs field. This will cause has_vector_of_structs()
  // to return false.
  void clear_vector_of_structs() {
    vector_of_structs_.reset();
    ClearField(kInlineAbsoluteOffset_vector_of_structs, 4, 18);
  }

  // Returns true if the vector_of_structs field is set and can be accessed.
  bool has_vector_of_structs() const {
    return AsFlatbuffer().has_vector_of_structs();
  }

  // Creates an empty object for the unspecified_length_vector_of_strings field,
  // which you can then populate/modify as desired. The field must not be
  // populated yet.
  ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0> *
  add_unspecified_length_vector_of_strings() {
    CHECK(!unspecified_length_vector_of_strings_.has_value());
    constexpr size_t kVtableIndex = 28;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<::aos::fbs::String<0>, 0, false,
                                     0>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_unspecified_length_vector_of_strings;
      std::optional<std::span<uint8_t>> inserted_bytes = InsertBytes(
          buffer().data() + object_absolute_offset,
          ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>::kSize,
          ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_unspecified_length_vector_of_strings =
          object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_vector_of_strings_.emplace(
          BufferForObject(
              object_absolute_offset_unspecified_length_vector_of_strings,
              ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>::kSize),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_vector_of_strings_.emplace(
          BufferForObject(
              object_absolute_offset_unspecified_length_vector_of_strings,
              ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_unspecified_length_vector_of_strings,
        kVtableIndex,
        object_absolute_offset_unspecified_length_vector_of_strings -
            kInlineAbsoluteOffset_unspecified_length_vector_of_strings);
    return &unspecified_length_vector_of_strings_.value().t;
  }

  // Returns a pointer to the unspecified_length_vector_of_strings field, if
  // set. nullptr otherwise.
  const ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0> *
  unspecified_length_vector_of_strings() const {
    return unspecified_length_vector_of_strings_.has_value()
               ? &unspecified_length_vector_of_strings_.value().t
               : nullptr;
  }
  ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0> *
  mutable_unspecified_length_vector_of_strings() {
    return unspecified_length_vector_of_strings_.has_value()
               ? &unspecified_length_vector_of_strings_.value().t
               : nullptr;
  }

  // Clears the unspecified_length_vector_of_strings field. This will cause
  // has_unspecified_length_vector_of_strings() to return false.
  void clear_unspecified_length_vector_of_strings() {
    unspecified_length_vector_of_strings_.reset();
    ClearField(kInlineAbsoluteOffset_unspecified_length_vector_of_strings, 4,
               28);
  }

  // Returns true if the unspecified_length_vector_of_strings field is set and
  // can be accessed.
  bool has_unspecified_length_vector_of_strings() const {
    return AsFlatbuffer().has_unspecified_length_vector_of_strings();
  }

  // Creates an empty object for the vector_of_tables field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0> *
  add_vector_of_tables() {
    CHECK(!vector_of_tables_.has_value());
    constexpr size_t kVtableIndex = 20;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3,
                                     false, 0>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_vector_of_tables;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3,
                                         false, 0>::kSize,
                      ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_vector_of_tables = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_tables_.emplace(
          BufferForObject(object_absolute_offset_vector_of_tables,
                          ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic,
                                             3, false, 0>::kSize),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_tables_.emplace(
          BufferForObject(object_absolute_offset_vector_of_tables,
                          ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic,
                                             3, false, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_vector_of_tables, kVtableIndex,
        object_absolute_offset_vector_of_tables -
            kInlineAbsoluteOffset_vector_of_tables);
    return &vector_of_tables_.value().t;
  }

  // Returns a pointer to the vector_of_tables field, if set. nullptr otherwise.
  const ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0> *
  vector_of_tables() const {
    return vector_of_tables_.has_value() ? &vector_of_tables_.value().t
                                         : nullptr;
  }
  ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0> *
  mutable_vector_of_tables() {
    return vector_of_tables_.has_value() ? &vector_of_tables_.value().t
                                         : nullptr;
  }

  // Clears the vector_of_tables field. This will cause has_vector_of_tables()
  // to return false.
  void clear_vector_of_tables() {
    vector_of_tables_.reset();
    ClearField(kInlineAbsoluteOffset_vector_of_tables, 4, 20);
  }

  // Returns true if the vector_of_tables field is set and can be accessed.
  bool has_vector_of_tables() const {
    return AsFlatbuffer().has_vector_of_tables();
  }

  // Creates an empty object for the vector_aligned field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::Vector<int32_t, 3, true, 64> *add_vector_aligned() {
    CHECK(!vector_aligned_.has_value());
    constexpr size_t kVtableIndex = 16;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<int32_t, 3, true, 64>::kPreallocatedSize ==
                  0) {
      const size_t object_absolute_offset =
          object_absolute_offset_vector_aligned;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::Vector<int32_t, 3, true, 64>::kSize,
                      ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_vector_aligned = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_aligned_.emplace(
          BufferForObject(
              object_absolute_offset_vector_aligned,
              ::aos::fbs::Vector<int32_t, 3, true, 64>::RoundedLength(
                  inserted_bytes.value().size())),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_aligned_.emplace(
          BufferForObject(object_absolute_offset_vector_aligned,
                          ::aos::fbs::Vector<int32_t, 3, true, 64>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_vector_aligned, kVtableIndex,
        object_absolute_offset_vector_aligned -
            kInlineAbsoluteOffset_vector_aligned);
    return &vector_aligned_.value().t;
  }

  // Returns a pointer to the vector_aligned field, if set. nullptr otherwise.
  const ::aos::fbs::Vector<int32_t, 3, true, 64> *vector_aligned() const {
    return vector_aligned_.has_value() ? &vector_aligned_.value().t : nullptr;
  }
  ::aos::fbs::Vector<int32_t, 3, true, 64> *mutable_vector_aligned() {
    return vector_aligned_.has_value() ? &vector_aligned_.value().t : nullptr;
  }

  // Clears the vector_aligned field. This will cause has_vector_aligned() to
  // return false.
  void clear_vector_aligned() {
    vector_aligned_.reset();
    ClearField(kInlineAbsoluteOffset_vector_aligned, 4, 16);
  }

  // Returns true if the vector_aligned field is set and can be accessed.
  bool has_vector_aligned() const {
    return AsFlatbuffer().has_vector_aligned();
  }

  // Creates an empty object for the vector_of_strings field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0> *
  add_vector_of_strings() {
    CHECK(!vector_of_strings_.has_value());
    constexpr size_t kVtableIndex = 10;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<::aos::fbs::String<10>, 3, false,
                                     0>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_vector_of_strings;
      std::optional<std::span<uint8_t>> inserted_bytes = InsertBytes(
          buffer().data() + object_absolute_offset,
          ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>::kSize,
          ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_vector_of_strings = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_strings_.emplace(
          BufferForObject(
              object_absolute_offset_vector_of_strings,
              ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>::kSize),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_strings_.emplace(
          BufferForObject(
              object_absolute_offset_vector_of_strings,
              ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_vector_of_strings, kVtableIndex,
        object_absolute_offset_vector_of_strings -
            kInlineAbsoluteOffset_vector_of_strings);
    return &vector_of_strings_.value().t;
  }

  // Returns a pointer to the vector_of_strings field, if set. nullptr
  // otherwise.
  const ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0> *
  vector_of_strings() const {
    return vector_of_strings_.has_value() ? &vector_of_strings_.value().t
                                          : nullptr;
  }
  ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0> *
  mutable_vector_of_strings() {
    return vector_of_strings_.has_value() ? &vector_of_strings_.value().t
                                          : nullptr;
  }

  // Clears the vector_of_strings field. This will cause has_vector_of_strings()
  // to return false.
  void clear_vector_of_strings() {
    vector_of_strings_.reset();
    ClearField(kInlineAbsoluteOffset_vector_of_strings, 4, 10);
  }

  // Returns true if the vector_of_strings field is set and can be accessed.
  bool has_vector_of_strings() const {
    return AsFlatbuffer().has_vector_of_strings();
  }

  // Creates an empty object for the vector_of_scalars field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::Vector<int32_t, 3, true, 0> *add_vector_of_scalars() {
    CHECK(!vector_of_scalars_.has_value());
    constexpr size_t kVtableIndex = 6;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<int32_t, 3, true, 0>::kPreallocatedSize ==
                  0) {
      const size_t object_absolute_offset =
          object_absolute_offset_vector_of_scalars;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::Vector<int32_t, 3, true, 0>::kSize,
                      ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_vector_of_scalars = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_scalars_.emplace(
          BufferForObject(
              object_absolute_offset_vector_of_scalars,
              ::aos::fbs::Vector<int32_t, 3, true, 0>::RoundedLength(
                  inserted_bytes.value().size())),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      vector_of_scalars_.emplace(
          BufferForObject(object_absolute_offset_vector_of_scalars,
                          ::aos::fbs::Vector<int32_t, 3, true, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_vector_of_scalars, kVtableIndex,
        object_absolute_offset_vector_of_scalars -
            kInlineAbsoluteOffset_vector_of_scalars);
    return &vector_of_scalars_.value().t;
  }

  // Returns a pointer to the vector_of_scalars field, if set. nullptr
  // otherwise.
  const ::aos::fbs::Vector<int32_t, 3, true, 0> *vector_of_scalars() const {
    return vector_of_scalars_.has_value() ? &vector_of_scalars_.value().t
                                          : nullptr;
  }
  ::aos::fbs::Vector<int32_t, 3, true, 0> *mutable_vector_of_scalars() {
    return vector_of_scalars_.has_value() ? &vector_of_scalars_.value().t
                                          : nullptr;
  }

  // Clears the vector_of_scalars field. This will cause has_vector_of_scalars()
  // to return false.
  void clear_vector_of_scalars() {
    vector_of_scalars_.reset();
    ClearField(kInlineAbsoluteOffset_vector_of_scalars, 4, 6);
  }

  // Returns true if the vector_of_scalars field is set and can be accessed.
  bool has_vector_of_scalars() const {
    return AsFlatbuffer().has_vector_of_scalars();
  }

  // Creates an empty object for the unspecified_length_string field, which you
  // can then populate/modify as desired. The field must not be populated yet.
  ::aos::fbs::String<0> *add_unspecified_length_string() {
    CHECK(!unspecified_length_string_.has_value());
    constexpr size_t kVtableIndex = 26;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::String<0>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_unspecified_length_string;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::String<0>::kSize, ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_unspecified_length_string = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_string_.emplace(
          BufferForObject(object_absolute_offset_unspecified_length_string,
                          ::aos::fbs::String<0>::RoundedLength(
                              inserted_bytes.value().size())),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_string_.emplace(
          BufferForObject(object_absolute_offset_unspecified_length_string,
                          ::aos::fbs::String<0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_unspecified_length_string, kVtableIndex,
        object_absolute_offset_unspecified_length_string -
            kInlineAbsoluteOffset_unspecified_length_string);
    return &unspecified_length_string_.value().t;
  }

  // Returns a pointer to the unspecified_length_string field, if set. nullptr
  // otherwise.
  const ::aos::fbs::String<0> *unspecified_length_string() const {
    return unspecified_length_string_.has_value()
               ? &unspecified_length_string_.value().t
               : nullptr;
  }
  ::aos::fbs::String<0> *mutable_unspecified_length_string() {
    return unspecified_length_string_.has_value()
               ? &unspecified_length_string_.value().t
               : nullptr;
  }

  // Clears the unspecified_length_string field. This will cause
  // has_unspecified_length_string() to return false.
  void clear_unspecified_length_string() {
    unspecified_length_string_.reset();
    ClearField(kInlineAbsoluteOffset_unspecified_length_string, 4, 26);
  }

  // Returns true if the unspecified_length_string field is set and can be
  // accessed.
  bool has_unspecified_length_string() const {
    return AsFlatbuffer().has_unspecified_length_string();
  }

  // Creates an empty object for the unspecified_length_vector field, which you
  // can then populate/modify as desired. The field must not be populated yet.
  ::aos::fbs::Vector<uint8_t, 0, true, 0> *add_unspecified_length_vector() {
    CHECK(!unspecified_length_vector_.has_value());
    constexpr size_t kVtableIndex = 24;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::Vector<uint8_t, 0, true, 0>::kPreallocatedSize ==
                  0) {
      const size_t object_absolute_offset =
          object_absolute_offset_unspecified_length_vector;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::Vector<uint8_t, 0, true, 0>::kSize,
                      ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_unspecified_length_vector = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_vector_.emplace(
          BufferForObject(
              object_absolute_offset_unspecified_length_vector,
              ::aos::fbs::Vector<uint8_t, 0, true, 0>::RoundedLength(
                  inserted_bytes.value().size())),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      unspecified_length_vector_.emplace(
          BufferForObject(object_absolute_offset_unspecified_length_vector,
                          ::aos::fbs::Vector<uint8_t, 0, true, 0>::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_unspecified_length_vector, kVtableIndex,
        object_absolute_offset_unspecified_length_vector -
            kInlineAbsoluteOffset_unspecified_length_vector);
    return &unspecified_length_vector_.value().t;
  }

  // Returns a pointer to the unspecified_length_vector field, if set. nullptr
  // otherwise.
  const ::aos::fbs::Vector<uint8_t, 0, true, 0> *unspecified_length_vector()
      const {
    return unspecified_length_vector_.has_value()
               ? &unspecified_length_vector_.value().t
               : nullptr;
  }
  ::aos::fbs::Vector<uint8_t, 0, true, 0> *mutable_unspecified_length_vector() {
    return unspecified_length_vector_.has_value()
               ? &unspecified_length_vector_.value().t
               : nullptr;
  }

  // Clears the unspecified_length_vector field. This will cause
  // has_unspecified_length_vector() to return false.
  void clear_unspecified_length_vector() {
    unspecified_length_vector_.reset();
    ClearField(kInlineAbsoluteOffset_unspecified_length_vector, 4, 24);
  }

  // Returns true if the unspecified_length_vector field is set and can be
  // accessed.
  bool has_unspecified_length_vector() const {
    return AsFlatbuffer().has_unspecified_length_vector();
  }

  // Creates an empty object for the included_table field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  aos::fbs::testing::included::IncludedTableStatic *add_included_table() {
    CHECK(!included_table_.has_value());
    constexpr size_t kVtableIndex = 22;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (aos::fbs::testing::included::IncludedTableStatic::
                      kPreallocatedSize == 0) {
      const size_t object_absolute_offset =
          object_absolute_offset_included_table;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      aos::fbs::testing::included::IncludedTableStatic::kSize,
                      ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_included_table = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      included_table_.emplace(
          BufferForObject(
              object_absolute_offset_included_table,
              aos::fbs::testing::included::IncludedTableStatic::kSize),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      included_table_.emplace(
          BufferForObject(
              object_absolute_offset_included_table,
              aos::fbs::testing::included::IncludedTableStatic::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_included_table, kVtableIndex,
        object_absolute_offset_included_table -
            kInlineAbsoluteOffset_included_table);
    return &included_table_.value().t;
  }

  // Returns a pointer to the included_table field, if set. nullptr otherwise.
  const aos::fbs::testing::included::IncludedTableStatic *included_table()
      const {
    return included_table_.has_value() ? &included_table_.value().t : nullptr;
  }
  aos::fbs::testing::included::IncludedTableStatic *mutable_included_table() {
    return included_table_.has_value() ? &included_table_.value().t : nullptr;
  }

  // Clears the included_table field. This will cause has_included_table() to
  // return false.
  void clear_included_table() {
    included_table_.reset();
    ClearField(kInlineAbsoluteOffset_included_table, 4, 22);
  }

  // Returns true if the included_table field is set and can be accessed.
  bool has_included_table() const {
    return AsFlatbuffer().has_included_table();
  }

  // Creates an empty object for the subtable field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  aos::fbs::testing::SubTableStatic *add_subtable() {
    CHECK(!subtable_.has_value());
    constexpr size_t kVtableIndex = 14;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (aos::fbs::testing::SubTableStatic::kPreallocatedSize == 0) {
      const size_t object_absolute_offset = object_absolute_offset_subtable;
      std::optional<std::span<uint8_t>> inserted_bytes = InsertBytes(
          buffer().data() + object_absolute_offset,
          aos::fbs::testing::SubTableStatic::kSize, ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_subtable = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      subtable_.emplace(
          BufferForObject(object_absolute_offset_subtable,
                          aos::fbs::testing::SubTableStatic::kSize),
          this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      subtable_.emplace(
          BufferForObject(object_absolute_offset_subtable,
                          aos::fbs::testing::SubTableStatic::kSize),
          this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_subtable, kVtableIndex,
        object_absolute_offset_subtable - kInlineAbsoluteOffset_subtable);
    return &subtable_.value().t;
  }

  // Returns a pointer to the subtable field, if set. nullptr otherwise.
  const aos::fbs::testing::SubTableStatic *subtable() const {
    return subtable_.has_value() ? &subtable_.value().t : nullptr;
  }
  aos::fbs::testing::SubTableStatic *mutable_subtable() {
    return subtable_.has_value() ? &subtable_.value().t : nullptr;
  }

  // Clears the subtable field. This will cause has_subtable() to return false.
  void clear_subtable() {
    subtable_.reset();
    ClearField(kInlineAbsoluteOffset_subtable, 4, 14);
  }

  // Returns true if the subtable field is set and can be accessed.
  bool has_subtable() const { return AsFlatbuffer().has_subtable(); }

  // Creates an empty object for the string field, which you can
  // then populate/modify as desired.
  // The field must not be populated yet.
  ::aos::fbs::String<20> *add_string() {
    CHECK(!string_.has_value());
    constexpr size_t kVtableIndex = 8;
    // If this object does not normally have its initial memory statically
    // allocated, allocate it now (this is used for zero-length vectors).
    if constexpr (::aos::fbs::String<20>::kPreallocatedSize == 0) {
      const size_t object_absolute_offset = object_absolute_offset_string;
      std::optional<std::span<uint8_t>> inserted_bytes =
          InsertBytes(buffer().data() + object_absolute_offset,
                      ::aos::fbs::String<20>::kSize, ::aos::fbs::SetZero::kYes);
      if (!inserted_bytes.has_value()) {
        return nullptr;
      }
      // Undo changes to the object absolute offset that will have been made by
      // the InsertBytes call.
      // The InsertBytes() call normally goes through and "fixes" any offsets
      // that will have been affected by the memory insertion. Unfortunately,
      // if this object currently takes up zero bytes then the InsertBytes()
      // cannot distinguish between this offset and the (identical) offsets for
      // any other objects that may have been "sharing" this location. The
      // effect of this logic is that the first object that gets populated at
      // any given location will bump all other objects to later. This is fine,
      // although it does mean that the order in which objects appear in memory
      // may vary depending on the order in which they are constructed (if they
      // start out sharing a start pointer).
      object_absolute_offset_string = object_absolute_offset;

      // Construct the *Static object that we will use for managing this
      // subtable.
      string_.emplace(BufferForObject(object_absolute_offset_string,
                                      ::aos::fbs::String<20>::RoundedLength(
                                          inserted_bytes.value().size())),
                      this);
    } else {
      // Construct the *Static object that we will use for managing this
      // subtable.
      string_.emplace(BufferForObject(object_absolute_offset_string,
                                      ::aos::fbs::String<20>::kSize),
                      this);
    }
    // Actually set the appropriate fields in the flatbuffer memory itself.
    SetField<::flatbuffers::uoffset_t>(
        kInlineAbsoluteOffset_string, kVtableIndex,
        object_absolute_offset_string - kInlineAbsoluteOffset_string);
    return &string_.value().t;
  }

  // Returns a pointer to the string field, if set. nullptr otherwise.
  const ::aos::fbs::String<20> *string() const {
    return string_.has_value() ? &string_.value().t : nullptr;
  }
  ::aos::fbs::String<20> *mutable_string() {
    return string_.has_value() ? &string_.value().t : nullptr;
  }

  // Clears the string field. This will cause has_string() to return false.
  void clear_string() {
    string_.reset();
    ClearField(kInlineAbsoluteOffset_string, 4, 8);
  }

  // Returns true if the string field is set and can be accessed.
  bool has_string() const { return AsFlatbuffer().has_string(); }

  // Sets the scalar field, causing it to be populated if it is not already.
  // This will populate the field even if the specified value is the default.
  void set_scalar(const int32_t &value) {
    SetField<int32_t>(kInlineAbsoluteOffset_scalar, 4, value);
  }

  // Returns the value of scalar if set; nullopt otherwise.
  std::optional<int32_t> scalar() const {
    return has_scalar()
               ? std::make_optional(Get<int32_t>(kInlineAbsoluteOffset_scalar))
               : std::nullopt;
  }
  // Returns a pointer to modify the scalar field.
  // The pointer may be invalidated by mutations/movements of the underlying
  // buffer. Returns nullptr if the field is not set.
  int32_t *mutable_scalar() {
    return has_scalar() ? MutableGet<int32_t>(kInlineAbsoluteOffset_scalar)
                        : nullptr;
  }

  // Clears the scalar field. This will cause has_scalar() to return false.
  void clear_scalar() { ClearField(kInlineAbsoluteOffset_scalar, 4, 4); }

  // Returns true if the scalar field is set and can be accessed.
  bool has_scalar() const { return AsFlatbuffer().has_scalar(); }

  // Clears every field of the table, removing any existing state.
  void Clear() {
    clear_substruct();
    clear_vector_of_structs();
    clear_unspecified_length_vector_of_strings();
    clear_vector_of_tables();
    clear_vector_aligned();
    clear_vector_of_strings();
    clear_vector_of_scalars();
    clear_unspecified_length_string();
    clear_unspecified_length_vector();
    clear_included_table();
    clear_subtable();
    clear_string();
    clear_scalar();
  }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // This is a deep copy, and will call FromFlatbuffer on any constituent
  // objects.
  [[nodiscard]] bool FromFlatbuffer([[maybe_unused]] const Flatbuffer &other) {
    Clear();

    if (other.has_substruct()) {
      set_substruct(*other.substruct());
    }

    if (other.has_vector_of_structs()) {
      ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>
          *added_vector_of_structs = add_vector_of_structs();
      CHECK(added_vector_of_structs != nullptr);
      if (!added_vector_of_structs->FromFlatbuffer(other.vector_of_structs())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_unspecified_length_vector_of_strings()) {
      ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>
          *added_unspecified_length_vector_of_strings =
              add_unspecified_length_vector_of_strings();
      CHECK(added_unspecified_length_vector_of_strings != nullptr);
      if (!added_unspecified_length_vector_of_strings->FromFlatbuffer(
              other.unspecified_length_vector_of_strings())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_vector_of_tables()) {
      ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0>
          *added_vector_of_tables = add_vector_of_tables();
      CHECK(added_vector_of_tables != nullptr);
      if (!added_vector_of_tables->FromFlatbuffer(other.vector_of_tables())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_vector_aligned()) {
      ::aos::fbs::Vector<int32_t, 3, true, 64> *added_vector_aligned =
          add_vector_aligned();
      CHECK(added_vector_aligned != nullptr);
      if (!added_vector_aligned->FromFlatbuffer(other.vector_aligned())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_vector_of_strings()) {
      ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>
          *added_vector_of_strings = add_vector_of_strings();
      CHECK(added_vector_of_strings != nullptr);
      if (!added_vector_of_strings->FromFlatbuffer(other.vector_of_strings())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_vector_of_scalars()) {
      ::aos::fbs::Vector<int32_t, 3, true, 0> *added_vector_of_scalars =
          add_vector_of_scalars();
      CHECK(added_vector_of_scalars != nullptr);
      if (!added_vector_of_scalars->FromFlatbuffer(other.vector_of_scalars())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_unspecified_length_string()) {
      ::aos::fbs::String<0> *added_unspecified_length_string =
          add_unspecified_length_string();
      CHECK(added_unspecified_length_string != nullptr);
      if (!added_unspecified_length_string->FromFlatbuffer(
              other.unspecified_length_string())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_unspecified_length_vector()) {
      ::aos::fbs::Vector<uint8_t, 0, true, 0> *added_unspecified_length_vector =
          add_unspecified_length_vector();
      CHECK(added_unspecified_length_vector != nullptr);
      if (!added_unspecified_length_vector->FromFlatbuffer(
              other.unspecified_length_vector())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_included_table()) {
      aos::fbs::testing::included::IncludedTableStatic *added_included_table =
          add_included_table();
      CHECK(added_included_table != nullptr);
      if (!added_included_table->FromFlatbuffer(other.included_table())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_subtable()) {
      aos::fbs::testing::SubTableStatic *added_subtable = add_subtable();
      CHECK(added_subtable != nullptr);
      if (!added_subtable->FromFlatbuffer(other.subtable())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_string()) {
      ::aos::fbs::String<20> *added_string = add_string();
      CHECK(added_string != nullptr);
      if (!added_string->FromFlatbuffer(other.string())) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.has_scalar()) {
      set_scalar(other.scalar());
    }

    return true;
  }
  // Equivalent to FromFlatbuffer(const Flatbuffer&); this overload is provided
  // to ease implementation of the aos::fbs::Vector internals.
  [[nodiscard]] bool FromFlatbuffer(const Flatbuffer *other) {
    CHECK(other != nullptr);
    return FromFlatbuffer(*other);
  }

  // Copies the contents of the provided flatbuffer into this flatbuffer,
  // returning true on success.
  // Because the Flatbuffer Object API does not provide any concept of an
  // optionally populated scalar field, all scalar fields will be populated
  // after a call to FromFlatbufferObject().
  // This is a deep copy, and will call FromFlatbufferObject on
  // any constituent objects.
  [[nodiscard]] bool FromFlatbuffer(
      [[maybe_unused]] const Flatbuffer::NativeTableType &other) {
    Clear();

    if (other.substruct) {
      set_substruct(*other.substruct);
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>
          *added_vector_of_structs = add_vector_of_structs();
      CHECK(added_vector_of_structs != nullptr);
      if (!added_vector_of_structs->FromFlatbuffer(other.vector_of_structs)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>
          *added_unspecified_length_vector_of_strings =
              add_unspecified_length_vector_of_strings();
      CHECK(added_unspecified_length_vector_of_strings != nullptr);
      if (!added_unspecified_length_vector_of_strings->FromFlatbuffer(
              other.unspecified_length_vector_of_strings)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0>
          *added_vector_of_tables = add_vector_of_tables();
      CHECK(added_vector_of_tables != nullptr);
      if (!added_vector_of_tables->FromFlatbuffer(other.vector_of_tables)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<int32_t, 3, true, 64> *added_vector_aligned =
          add_vector_aligned();
      CHECK(added_vector_aligned != nullptr);
      if (!added_vector_aligned->FromFlatbuffer(other.vector_aligned)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>
          *added_vector_of_strings = add_vector_of_strings();
      CHECK(added_vector_of_strings != nullptr);
      if (!added_vector_of_strings->FromFlatbuffer(other.vector_of_strings)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<int32_t, 3, true, 0> *added_vector_of_scalars =
          add_vector_of_scalars();
      CHECK(added_vector_of_scalars != nullptr);
      if (!added_vector_of_scalars->FromFlatbuffer(other.vector_of_scalars)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::String<0> *added_unspecified_length_string =
          add_unspecified_length_string();
      CHECK(added_unspecified_length_string != nullptr);
      if (!added_unspecified_length_string->FromFlatbuffer(
              other.unspecified_length_string)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::Vector<uint8_t, 0, true, 0> *added_unspecified_length_vector =
          add_unspecified_length_vector();
      CHECK(added_unspecified_length_vector != nullptr);
      if (!added_unspecified_length_vector->FromFlatbuffer(
              other.unspecified_length_vector)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.included_table) {
      aos::fbs::testing::included::IncludedTableStatic *added_included_table =
          add_included_table();
      CHECK(added_included_table != nullptr);
      if (!added_included_table->FromFlatbuffer(*other.included_table)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    if (other.subtable) {
      aos::fbs::testing::SubTableStatic *added_subtable = add_subtable();
      CHECK(added_subtable != nullptr);
      if (!added_subtable->FromFlatbuffer(*other.subtable)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    // Unconditionally copy strings/vectors, even if it will just end up
    // being 0-length (this maintains consistency with the flatbuffer Pack()
    // behavior).
    {
      ::aos::fbs::String<20> *added_string = add_string();
      CHECK(added_string != nullptr);
      if (!added_string->FromFlatbuffer(other.string)) {
        // Fail if we were unable to copy (e.g., if we tried to copy in a long
        // vector and do not have the space for it).
        return false;
      }
    }

    set_scalar(other.scalar);

    return true;
  }
  [[nodiscard]] bool FromFlatbuffer(
      const flatbuffers::unique_ptr<Flatbuffer::NativeTableType> &other) {
    return FromFlatbuffer(*other);
  }

 private:
  // We need to provide a MoveConstructor to allow this table to be
  // used inside of vectors, but we do not want it readily available to
  // users. See TableMover for more details.
  TestTableStatic(TestTableStatic &&) = default;
  friend struct ::aos::fbs::internal::TableMover<TestTableStatic>;

  // Offset from the start of the buffer to the inline data for the substruct
  // field.
  static constexpr size_t kInlineAbsoluteOffset_substruct = 4;

  // Members relating to the vector_of_structs field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>>>
      vector_of_structs_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetvector_of_structs =
      ::aos::fbs::AlignOffset(
          (kVtableStart + kVtableSize) - kAlignOffset,
          ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true, 0>::kAlign,
          ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true,
                             0>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_vector_of_structs =
      kDefaultObjectAbsoluteOffsetvector_of_structs;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_vector_of_structs = 20;

  // Members relating to the unspecified_length_vector_of_strings field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>>>
      unspecified_length_vector_of_strings_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t
      kDefaultObjectAbsoluteOffsetunspecified_length_vector_of_strings =
          ::aos::fbs::AlignOffset(
              kDefaultObjectAbsoluteOffsetvector_of_structs +
                  ::aos::fbs::Vector<aos::fbs::testing::SubStruct, 3, true,
                                     0>::kPreallocatedSize -
                  kAlignOffset,
              ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false, 0>::kAlign,
              ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false,
                                 0>::kAlignOffset) +
          kAlignOffset;
  size_t object_absolute_offset_unspecified_length_vector_of_strings =
      kDefaultObjectAbsoluteOffsetunspecified_length_vector_of_strings;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t
      kInlineAbsoluteOffset_unspecified_length_vector_of_strings = 24;

  // Members relating to the vector_of_tables field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false, 0>>>
      vector_of_tables_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetvector_of_tables =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetunspecified_length_vector_of_strings +
              ::aos::fbs::Vector<::aos::fbs::String<0>, 0, false,
                                 0>::kPreallocatedSize -
              kAlignOffset,
          ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false,
                             0>::kAlign,
          ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false,
                             0>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_vector_of_tables =
      kDefaultObjectAbsoluteOffsetvector_of_tables;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_vector_of_tables = 28;

  // Members relating to the vector_aligned field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      ::aos::fbs::Vector<int32_t, 3, true, 64>>>
      vector_aligned_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetvector_aligned =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetvector_of_tables +
              ::aos::fbs::Vector<aos::fbs::testing::SubTableStatic, 3, false,
                                 0>::kPreallocatedSize -
              kAlignOffset,
          ::aos::fbs::Vector<int32_t, 3, true, 64>::kAlign,
          ::aos::fbs::Vector<int32_t, 3, true, 64>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_vector_aligned =
      kDefaultObjectAbsoluteOffsetvector_aligned;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_vector_aligned = 32;

  // Members relating to the vector_of_strings field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>>>
      vector_of_strings_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetvector_of_strings =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetvector_aligned +
              ::aos::fbs::Vector<int32_t, 3, true, 64>::kPreallocatedSize -
              kAlignOffset,
          ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false, 0>::kAlign,
          ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false,
                             0>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_vector_of_strings =
      kDefaultObjectAbsoluteOffsetvector_of_strings;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_vector_of_strings = 36;

  // Members relating to the vector_of_scalars field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<
      ::aos::fbs::internal::TableMover<::aos::fbs::Vector<int32_t, 3, true, 0>>>
      vector_of_scalars_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetvector_of_scalars =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetvector_of_strings +
              ::aos::fbs::Vector<::aos::fbs::String<10>, 3, false,
                                 0>::kPreallocatedSize -
              kAlignOffset,
          ::aos::fbs::Vector<int32_t, 3, true, 0>::kAlign,
          ::aos::fbs::Vector<int32_t, 3, true, 0>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_vector_of_scalars =
      kDefaultObjectAbsoluteOffsetvector_of_scalars;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_vector_of_scalars = 40;

  // Members relating to the unspecified_length_string field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<::aos::fbs::String<0>>>
      unspecified_length_string_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t
      kDefaultObjectAbsoluteOffsetunspecified_length_string =
          ::aos::fbs::AlignOffset(
              kDefaultObjectAbsoluteOffsetvector_of_scalars +
                  ::aos::fbs::Vector<int32_t, 3, true, 0>::kPreallocatedSize -
                  kAlignOffset,
              ::aos::fbs::String<0>::kAlign,
              ::aos::fbs::String<0>::kAlignOffset) +
          kAlignOffset;
  size_t object_absolute_offset_unspecified_length_string =
      kDefaultObjectAbsoluteOffsetunspecified_length_string;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_unspecified_length_string = 44;

  // Members relating to the unspecified_length_vector field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<
      ::aos::fbs::internal::TableMover<::aos::fbs::Vector<uint8_t, 0, true, 0>>>
      unspecified_length_vector_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t
      kDefaultObjectAbsoluteOffsetunspecified_length_vector =
          ::aos::fbs::AlignOffset(
              kDefaultObjectAbsoluteOffsetunspecified_length_string +
                  ::aos::fbs::String<0>::kPreallocatedSize - kAlignOffset,
              ::aos::fbs::Vector<uint8_t, 0, true, 0>::kAlign,
              ::aos::fbs::Vector<uint8_t, 0, true, 0>::kAlignOffset) +
          kAlignOffset;
  size_t object_absolute_offset_unspecified_length_vector =
      kDefaultObjectAbsoluteOffsetunspecified_length_vector;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_unspecified_length_vector = 48;

  // Members relating to the included_table field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<
      aos::fbs::testing::included::IncludedTableStatic>>
      included_table_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetincluded_table =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetunspecified_length_vector +
              ::aos::fbs::Vector<uint8_t, 0, true, 0>::kPreallocatedSize -
              kAlignOffset,
          aos::fbs::testing::included::IncludedTableStatic::kAlign,
          aos::fbs::testing::included::IncludedTableStatic::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_included_table =
      kDefaultObjectAbsoluteOffsetincluded_table;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_included_table = 52;

  // Members relating to the subtable field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<
      ::aos::fbs::internal::TableMover<aos::fbs::testing::SubTableStatic>>
      subtable_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetsubtable =
      ::aos::fbs::AlignOffset(kDefaultObjectAbsoluteOffsetincluded_table +
                                  aos::fbs::testing::included::
                                      IncludedTableStatic::kPreallocatedSize -
                                  kAlignOffset,
                              aos::fbs::testing::SubTableStatic::kAlign,
                              aos::fbs::testing::SubTableStatic::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_subtable = kDefaultObjectAbsoluteOffsetsubtable;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_subtable = 56;

  // Members relating to the string field.
  //
  // *Static object used for managing this subtable. Will be nullopt
  // when the field is not populated.
  // We use the TableMover to be able to make this object moveable.
  std::optional<::aos::fbs::internal::TableMover<::aos::fbs::String<20>>>
      string_;
  // Offset from the start of the buffer to the start of the actual
  // data for this field. Will be updated even when the table is not
  // populated, so that we know where to construct it when requested.
  static constexpr size_t kDefaultObjectAbsoluteOffsetstring =
      ::aos::fbs::AlignOffset(
          kDefaultObjectAbsoluteOffsetsubtable +
              aos::fbs::testing::SubTableStatic::kPreallocatedSize -
              kAlignOffset,
          ::aos::fbs::String<20>::kAlign,
          ::aos::fbs::String<20>::kAlignOffset) +
      kAlignOffset;
  size_t object_absolute_offset_string = kDefaultObjectAbsoluteOffsetstring;
  // Offset from the start of the buffer to the offset in the inline data for
  // this field.
  static constexpr size_t kInlineAbsoluteOffset_string = 60;

  // Offset from the start of the buffer to the inline data for the scalar
  // field.
  static constexpr size_t kInlineAbsoluteOffset_scalar = 64;

  size_t NumberOfSubObjects() const final { return 11; }
  using ::aos::fbs::ResizeableObject::SubObject;
  SubObject GetSubObject(size_t index) final {
    SubObject object;
    // Note: The below arrays are local variables rather than class members to
    // avoid having to deal with what happens to them if the object is moved.

    // Array of the members that we use for tracking where the buffers for
    // each subobject belong.
    // Pointers because these may need to be modified when memory is
    // inserted into the buffer.
    const std::array<size_t *, 11> subobject_object_offsets{
        &object_absolute_offset_vector_of_structs,
        &object_absolute_offset_unspecified_length_vector_of_strings,
        &object_absolute_offset_vector_of_tables,
        &object_absolute_offset_vector_aligned,
        &object_absolute_offset_vector_of_strings,
        &object_absolute_offset_vector_of_scalars,
        &object_absolute_offset_unspecified_length_string,
        &object_absolute_offset_unspecified_length_vector,
        &object_absolute_offset_included_table,
        &object_absolute_offset_subtable,
        &object_absolute_offset_string};
    // Actual subobjects; note that the pointers will be invalid when the
    // field is not populated.
    const std::array<::aos::fbs::ResizeableObject *, 11> subobject_objects{
        &vector_of_structs_->t,
        &unspecified_length_vector_of_strings_->t,
        &vector_of_tables_->t,
        &vector_aligned_->t,
        &vector_of_strings_->t,
        &vector_of_scalars_->t,
        &unspecified_length_string_->t,
        &unspecified_length_vector_->t,
        &included_table_->t,
        &subtable_->t,
        &string_->t};
    // Absolute offsets from the start of the buffer to where the inline
    // entry is for each table. These offsets do not need to change at
    // runtime (because memory is never inserted into the start of
    // a given table), but the offsets pointed to by these offsets
    // may need to be updated.
    const std::array<size_t, 11> subobject_inline_offsets{
        kInlineAbsoluteOffset_vector_of_structs,
        kInlineAbsoluteOffset_unspecified_length_vector_of_strings,
        kInlineAbsoluteOffset_vector_of_tables,
        kInlineAbsoluteOffset_vector_aligned,
        kInlineAbsoluteOffset_vector_of_strings,
        kInlineAbsoluteOffset_vector_of_scalars,
        kInlineAbsoluteOffset_unspecified_length_string,
        kInlineAbsoluteOffset_unspecified_length_vector,
        kInlineAbsoluteOffset_included_table,
        kInlineAbsoluteOffset_subtable,
        kInlineAbsoluteOffset_string};
    object.inline_entry =
        MutableGet<::flatbuffers::uoffset_t>(subobject_inline_offsets[index]);
    object.object =
        (*object.inline_entry == 0) ? nullptr : subobject_objects[index];
    object.absolute_offset = subobject_object_offsets[index];
    return object;
  }

 public:
  // Nominal size of this object, in bytes. The object may grow beyond this
  // size, but will always start at this size and so the initial buffer must
  // match this size.
  static constexpr size_t kSize =
      ::aos::fbs::AlignOffset(kDefaultObjectAbsoluteOffsetstring +
                                  ::aos::fbs::String<20>::kPreallocatedSize -
                                  kAlignOffset,
                              kAlign, kAlignOffset) +
      kAlignOffset;
  // Always statically allocate memory for tables (set for consistency with
  // static_vector.h).
  static constexpr size_t kPreallocatedSize = kSize;
  // Size required for a buffer that includes a root table offset at the start.
  static constexpr size_t kRootSize =
      ::aos::fbs::AlignOffset(kSize + sizeof(::flatbuffers::uoffset_t), kAlign);
};
}  // namespace aos::fbs::testing
