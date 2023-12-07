#ifndef AOS_FLATBUFFERS_STATIC_TABLE_H_
#define AOS_FLATBUFFERS_STATIC_TABLE_H_
#include <algorithm>
#include <span>

#include "flatbuffers/base.h"
#include "glog/logging.h"

#include "aos/flatbuffers/base.h"
namespace aos::fbs {

// This Table object is used as the parent class to the generated code for every
// flatbuffer table that we generate code for.
// This object primarily serves to provide some useful common methods for
// mutating the flatbuffer memory.
//
// Every table will be aligned to the greatest alignment of all of its members
// and its size will be equal to a multiple of the alignment. Each table shall
// have the following layout: [vtable offset; inline data with padding; vtable;
// padding; table/vector data with padding]
class Table : public ResizeableObject {
 public:
  // Prints out a debug string of the raw flatbuffer memory. Does not currently
  // do anything intelligent ot traverse down into the subobjects of the
  // flatbuffer (if you want that, then use the flatbuffer binary
  // annotator---this code mostly exists for debugging the static flatbuffers
  // implementation itself).
  std::string SerializationDebugString() const {
    std::stringstream str;
    str << "Size: " << buffer_.size() << " alignment: " << Alignment() << "\n";
    str << "Observed Vtable offset " << Get<soffset_t>(0) << "\n";
    str << "Inline Size " << InlineTableSize() << " Inline Bytes:\n";
    internal::DebugBytes(internal::GetSubSpan(buffer_, 4, InlineTableSize()),
                         str);
    str << "Vtable offset " << FixedVtableOffset() << " Vtable size "
        << VtableSize() << " Vtable contents:\n";
    internal::DebugBytes(
        internal::GetSubSpan(buffer_, FixedVtableOffset(), VtableSize()), str);
    str << "Offset data offset " << OffsetDataStart() << "\n";
    // Actual contents can be big; don't print them out until we run into a
    // situation where we need to debug that.
    return str.str();
  }

 protected:
  static constexpr size_t kMinAlign = alignof(uoffset_t);

  Table(std::span<uint8_t> buffer, ResizeableObject *parent)
      : ResizeableObject(buffer, parent) {}
  Table(std::span<uint8_t> buffer, Allocator *allocator)
      : ResizeableObject(buffer, allocator) {}
  Table(std::span<uint8_t> buffer, ::std::unique_ptr<Allocator> allocator)
      : ResizeableObject(buffer, ::std::move(allocator)) {}
  Table(Table &&) = default;
  virtual ~Table() {}
  virtual size_t FixedVtableOffset() const = 0;
  virtual size_t VtableSize() const = 0;
  virtual size_t InlineTableSize() const = 0;
  virtual size_t OffsetDataStart() const = 0;
  size_t AbsoluteOffsetOffset() const override { return 0; }
  void PopulateVtable() {
    // Zero out everything up to the start of the sub-messages/tables, which are
    // responsible for doing their own memory initialization.
    internal::ClearSpan(internal::GetSubSpan(buffer_, 0, OffsetDataStart()));
    // Set the offset to the start of the vtable (points backwards, hence the
    // sign inversion).
    Set<soffset_t>(0, -FixedVtableOffset());
    // First element of the vtable is the size of the table.
    Set<voffset_t>(FixedVtableOffset(), VtableSize());
    // Second element of the vtable is the size of the inlined data (not really
    // used by anything...).
    Set<voffset_t>(FixedVtableOffset() + sizeof(voffset_t), InlineTableSize());
  }

  template <typename T>
  void SetField(size_t absolute_offset, size_t vtable_offset, const T &value) {
    Set<T>(absolute_offset, value);
    CHECK_EQ(0u, (absolute_offset + reinterpret_cast<size_t>(buffer_.data())) %
                     alignof(T));
    Set<voffset_t>(FixedVtableOffset() + vtable_offset, absolute_offset);
  }

  void ClearField(size_t absolute_offset, size_t inline_size,
                  size_t vtable_offset) {
    // TODO: Remove/account for any excess allocated memory.
    internal::ClearSpan(
        internal::GetSubSpan(buffer_, absolute_offset, inline_size));
    Set<voffset_t>(FixedVtableOffset() + vtable_offset, 0);
  }

  template <typename T>
  const T &Get(size_t absolute_offset) const {
    return *reinterpret_cast<const T *>(buffer_.data() + absolute_offset);
  }

  template <typename T>
  T *MutableGet(size_t absolute_offset) {
    return reinterpret_cast<T *>(buffer_.data() + absolute_offset);
  }

  template <typename T>
  T *GetMutableFlatbuffer() {
    return reinterpret_cast<T *>(buffer_.data());
  }

  template <typename T>
  const T *GetFlatbuffer() const {
    return reinterpret_cast<const T *>(buffer_.data());
  }

 private:
  template <typename T>
  void Set(size_t absolute_offset, const T &value) {
    *reinterpret_cast<T *>(buffer_.data() + absolute_offset) = value;
  }
};

}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_STATIC_TABLE_H_
