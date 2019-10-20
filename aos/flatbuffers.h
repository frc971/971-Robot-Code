#ifndef AOS_FLATBUFFERS_H_
#define AOS_FLATBUFFERS_H_

#include "flatbuffers/flatbuffers.h"

namespace aos {

// TODO(austin): FlatbufferBase ?  We can associate the type table with it, and
// make it generic.

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class Flatbuffer {
 public:
  // Builds a Flatbuffer by taking ownership of the buffer.
  Flatbuffer(flatbuffers::DetachedBuffer &&buffer)
      : buffer_(::std::move(buffer)) {}

  // Builds a flatbuffer by taking ownership of the buffer from the other
  // flatbuffer.
  Flatbuffer(Flatbuffer &&fb) : buffer_(::std::move(fb.buffer_)) {}
  Flatbuffer &operator=(Flatbuffer &&fb) {
    ::std::swap(buffer_, fb.buffer_);
    return *this;
  }

  // Constructs an empty flatbuffer of type T.
  static Flatbuffer<T> Empty() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(1);
    const auto end = fbb.EndTable(fbb.StartTable());
    fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(end));
    return Flatbuffer<T>(fbb.Release());
  }

  // Returns the MiniReflectTypeTable for T.
  static const flatbuffers::TypeTable *MiniReflectTypeTable() {
    return T::MiniReflectTypeTable();
  }

  // Returns a message from the buffer.
  const T &message() const { return *flatbuffers::GetRoot<T>(buffer_.data()); }
  // Returns a mutable message.  It can be mutated via the flatbuffer rules.
  T *mutable_message() {
    return flatbuffers::GetMutableRoot<T>(buffer_.data());
  }

  // Returns references to the buffer, and the data.
  const flatbuffers::DetachedBuffer &buffer() const { return buffer_; }
  const uint8_t *data() const { return buffer_.data(); }

 private:
  flatbuffers::DetachedBuffer buffer_;
};

// TODO(austin): Need a fixed buffer version of Flatbuffer.
// TODO(austin): Need a way to get our hands on the max size.  Can start with
// "large" for now.

}  // namespace aos

#endif  // AOS_FLATBUFFERS_H_
