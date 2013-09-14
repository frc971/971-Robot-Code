// Copyright 2012 Google Inc. All Rights Reserved.
//
// A buffer for dealing with data.  Some special support for PTP is
// available.

#ifndef _GLIBUSB_GBUFFER_H_
#define _GLIBUSB_GBUFFER_H_

#include <stdint.h>
#include <string>
#include <vector>

namespace glibusb {

// Buffer of bytes.
class Buffer {
 public:
  // Underlying byte store type.
  typedef std::vector<uint8_t> ByteBuffer;
  typedef ByteBuffer::size_type size_type;

  // Destructor.
  ~Buffer();

  // Constructs an empty buffer.
  Buffer();

  // Constructs a buffer from memory of given length.  Data is copied.
  // (This is an interface to C functions, chiefly, libusb, and should
  // only be used for such purposes.)
  Buffer(const void *src, size_type length);

  // Returns true iff the buffers contain the same data.
  bool operator==(const Buffer &other) const;

  // Returns true iff the buffer does not contain the same data.
  bool operator!=(const Buffer &other) const;

  // Returns a new allocated slice of the buffer.
  Buffer *MakeSlice(size_type offset, size_type length) const;

  // Returns the length.
  size_type Length() const { return buffer_.size(); }

  // Clears (as in std::vector) the buffer.
  void Clear();

  // Resizes (as in std::vector) the buffer.
  void Resize(size_type length);

  // Returns a pointer to the underlying store that of a guaranteed
  // length (or CHECK), possibly beginning at a given offset.  (These
  // are interfaces to C functions, chiefly, libusb, and should only
  // be used for such purposes.)
  void *GetBufferPointer(size_type length);
  const void *GetBufferPointer(size_type length) const;
  void *GetBufferPointer(size_type offset, size_type length);
  const void *GetBufferPointer(size_type offset, size_type length) const;

  // Gets the value of integral type T from the buffer at the offset
  // byte_offset, places it in value_out, and returns the length of
  // the marshalled data.  The integer is expected to be in little
  // endian format. This template is specialized for
  // {int,uint}{8,16,32,64}_t.
  template <class T>
  size_type Get(size_type byte_offset, T *value_out) const;

  // Gets an ASCII string from the buffer at the offset byte_offset,
  // places it in value_out, and returns the length of the marshalled
  // data.  The string data in the buffer is expected to be
  // null-terminated.
  size_type Get(size_type byte_offset, std::string *value_out) const;

  // Puts the value of integral type T into the buffer offset
  // byte_offset and returns the length of the marshalled data.  Data
  // are put in little endian format. This template is specialized for
  // {int,uint}{8,16,32,64}_t.
  template <class T>
  size_type Put(size_type byte_offset, T value);

  // Appends the value of type T into the buffer offset byte_offset
  // and returns the length of the marshalled data.  Data are appended
  // in little endian format.  This template is available for
  // {int,uint}{8,16,32,64}_t.
  template <class T> void Append(const T value);

  // Append a buffer.
  void Append(const Buffer &buffer);

  // Inserts length bytes of (uninitialized) space at the beginning of
  // the buffer.
  void AddHeader(size_type length);

  // Removes length bytes of space from the beginning of the buffer.
  void RemoveHeader(size_type length);

  // Copies the source buffer.
  void Copy(const Buffer &source);

#if 0
  // Writes the contents of the buffer to the file, or dies.
  void WriteOrDie(File *fp) const;

  // Writes the contents of the buffer to the path, or dies.
  void WriteToPathOrDie(const char *name) const;
#endif

  // Returns a hex dump of the buffer.
  std::string Dump() const;

 private:
  // The underlying byte store.
  ByteBuffer buffer_;

  Buffer(const Buffer &) = delete;
  void operator=(const Buffer &) = delete;
};


// Template for Buffer::Append for integral values.
template <typename T>
void Buffer::Append(const T value) {
  size_type offset = Length();
  Resize(offset + sizeof(T));
  Put(offset, value);
}

}  // namespace glibusb

#endif  // _GLIBUSB_GBUFFER_H_
