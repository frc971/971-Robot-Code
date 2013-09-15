// Copyright 2012 Google Inc. All Rights Reserved.
//
// Modified by FRC Team 971.
//

#include "gbuffer.h"

#include <stddef.h>
#include <stdint.h>
#include <cstring>

#include "aos/common/logging/logging.h"

#include "ghexdump.h"

namespace glibusb {

Buffer::Buffer() {
  buffer_ = NULL;
  length_ = allocated_length_ = 0;
}

Buffer::~Buffer() {}

Buffer::Buffer(const void *src, Buffer::size_type length) : Buffer() {
  Resize(length);
  if (length > 0) {
    memcpy(buffer_, src, length);
  }
}

bool Buffer::operator==(const Buffer &other) const {
  return length_ == other.length_ &&
      memcmp(buffer_, other.buffer_, length_) == 0;
}

bool Buffer::operator!=(const Buffer &other) const {
  return !(*this == other);
}

Buffer *Buffer::MakeSlice(Buffer::size_type offset,
                          Buffer::size_type length) const {
  CHECK_LE(offset + length, length_);
  if (length == 0) {
    return new Buffer();
  } else {
    const uint8_t *p = &(buffer_[offset]);
    return new Buffer(p, length);
  }
}

void Buffer::Clear() {
  length_ = 0;
}

void Buffer::Resize(Buffer::size_type length) {
  if (length > allocated_length_) {
    if (length_ > 0) {
      uint8_t *old = buffer_;
      buffer_ = new uint8_t[length];
      memcpy(buffer_, old, length_);
      delete[] old;
    } else {
      delete[] buffer_;
      buffer_ = new uint8_t[length];
    }
  } else {
    memset(&buffer_[length_], 0, length_ - length);
  }
  length_ = length;
}

void *Buffer::GetBufferPointer(Buffer::size_type length) {
  return GetBufferPointer(0, length);
}

const void *Buffer::GetBufferPointer(Buffer::size_type length) const {
  return GetBufferPointer(0, length);
}

void *Buffer::GetBufferPointer(Buffer::size_type offset,
                               Buffer::size_type length) {
  if (length == 0) {
    return NULL;
  } else {
    CHECK_LE(offset + length, length_);
    uint8_t *p = &(buffer_[offset]);
    return static_cast<void *>(p);
  }
}

const void *Buffer::GetBufferPointer(Buffer::size_type offset,
                                     Buffer::size_type length) const {
  if (length == 0) {
    return NULL;
  } else {
    CHECK_LE(offset + length, length_);
    const uint8_t *p = &(buffer_[offset]);
    return static_cast<const void *>(p);
  }
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              uint8_t *value_out) const {
  CHECK_LT(byte_offset, length_);
  *CHECK_NOTNULL(value_out) = buffer_[byte_offset];
  return sizeof(uint8_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              int8_t *value_out) const {
  uint8_t value;
  Get(byte_offset, &value);
  *CHECK_NOTNULL(value_out) = static_cast<int8_t>(value);
  return sizeof(int8_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              uint16_t *value_out) const {
  CHECK_LT(byte_offset + 1, length_);
  uint16_t byte0 = static_cast<uint16_t>(buffer_[byte_offset]);
  uint16_t byte1 = static_cast<uint16_t>(buffer_[byte_offset + 1]);
  uint16_t value = byte0 | (byte1 << 8);
  *CHECK_NOTNULL(value_out) = value;
  return sizeof(uint16_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              int16_t *value_out) const {
  uint16_t value;
  Get(byte_offset, &value);
  *CHECK_NOTNULL(value_out) = static_cast<int16_t>(value);
  return sizeof(int16_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              uint32_t *value_out) const {
  CHECK_LT(byte_offset + 3, length_);
  uint32_t byte0 = static_cast<uint32_t>(buffer_[byte_offset]);
  uint32_t byte1 = static_cast<uint32_t>(buffer_[byte_offset + 1]);
  uint32_t byte2 = static_cast<uint32_t>(buffer_[byte_offset + 2]);
  uint32_t byte3 = static_cast<uint32_t>(buffer_[byte_offset + 3]);
  uint32_t value = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24);
  *CHECK_NOTNULL(value_out) = value;
  return sizeof(uint32_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              int32_t *value_out) const {
  uint32_t value;
  Get(byte_offset, &value);
  *CHECK_NOTNULL(value_out) = static_cast<int32_t>(value);
  return sizeof(int32_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              uint64_t *value_out) const {
  CHECK_LT(byte_offset + 7, length_);
  uint64_t byte0 = static_cast<uint64_t>(buffer_[byte_offset]);
  uint64_t byte1 = static_cast<uint64_t>(buffer_[byte_offset + 1]);
  uint64_t byte2 = static_cast<uint64_t>(buffer_[byte_offset + 2]);
  uint64_t byte3 = static_cast<uint64_t>(buffer_[byte_offset + 3]);
  uint64_t byte4 = static_cast<uint64_t>(buffer_[byte_offset + 4]);
  uint64_t byte5 = static_cast<uint64_t>(buffer_[byte_offset + 5]);
  uint64_t byte6 = static_cast<uint64_t>(buffer_[byte_offset + 6]);
  uint64_t byte7 = static_cast<uint64_t>(buffer_[byte_offset + 7]);
  uint64_t value =
      byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24) |
      (byte4 << 32) | (byte5 << 40) | (byte6 << 48) | (byte7 << 56);
  *CHECK_NOTNULL(value_out) = value;
  return sizeof(uint64_t);
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              int64_t *value_out) const {
  uint64_t value;
  Get(byte_offset, &value);
  *CHECK_NOTNULL(value_out) = static_cast<int64_t>(value);
  return sizeof(int64_t);
}

Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              std::string *out) const {
  CHECK_NOTNULL(out);
  out->clear();
  size_type n = 0;
  for (size_t i = byte_offset; /**/; ++i, ++n) {
    uint8_t p;
    Get(i, &p);
    if (!p) {
      break;
    }
    out->push_back(static_cast<char>(p));
  }

  // strings are always padded out to 4 bytes
  n = (n + 3) & ~3;
  return n;
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, uint8_t value) {
  CHECK_LT(byte_offset, length_);
  buffer_[byte_offset] = value;
  return sizeof(uint8_t);
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, int8_t value) {
  return Put(byte_offset, static_cast<uint8_t>(value));
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, uint16_t value) {
  CHECK_LT(byte_offset + 1, length_);
  uint8_t byte_0 = static_cast<uint8_t>(value & 0xff);
  uint8_t byte_1 = static_cast<uint8_t>((value >> 8) & 0xff);
  buffer_[byte_offset] = byte_0;
  buffer_[byte_offset + 1] = byte_1;
  return sizeof(uint16_t);
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, int16_t value) {
  return Put(byte_offset, static_cast<uint16_t>(value));
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, uint32_t value) {
  CHECK_LT(byte_offset + 3, length_);
  uint8_t byte_0 = static_cast<uint8_t>(value & 0xff);
  uint8_t byte_1 = static_cast<uint8_t>((value >> 8) & 0xff);
  uint8_t byte_2 = static_cast<uint8_t>((value >> 16) & 0xff);
  uint8_t byte_3 = static_cast<uint8_t>((value >> 24) & 0xff);
  buffer_[byte_offset] = byte_0;
  buffer_[byte_offset + 1] = byte_1;
  buffer_[byte_offset + 2] = byte_2;
  buffer_[byte_offset + 3] = byte_3;
  return sizeof(uint32_t);
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, int32_t value) {
  return Put(byte_offset, static_cast<uint32_t>(value));
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, uint64_t value) {
  CHECK_LT(byte_offset + 7, length_);
  uint8_t byte_0 = static_cast<uint8_t>(value & 0xff);
  uint8_t byte_1 = static_cast<uint8_t>((value >> 8) & 0xff);
  uint8_t byte_2 = static_cast<uint8_t>((value >> 16) & 0xff);
  uint8_t byte_3 = static_cast<uint8_t>((value >> 24) & 0xff);
  uint8_t byte_4 = static_cast<uint8_t>((value >> 32) & 0xff);
  uint8_t byte_5 = static_cast<uint8_t>((value >> 40) & 0xff);
  uint8_t byte_6 = static_cast<uint8_t>((value >> 48) & 0xff);
  uint8_t byte_7 = static_cast<uint8_t>((value >> 56) & 0xff);
  buffer_[byte_offset] = byte_0;
  buffer_[byte_offset + 1] = byte_1;
  buffer_[byte_offset + 2] = byte_2;
  buffer_[byte_offset + 3] = byte_3;
  buffer_[byte_offset + 4] = byte_4;
  buffer_[byte_offset + 5] = byte_5;
  buffer_[byte_offset + 6] = byte_6;
  buffer_[byte_offset + 7] = byte_7;
  return sizeof(uint64_t);
}

// Specialized template for Put
template <>
Buffer::size_type Buffer::Put(Buffer::size_type byte_offset, int64_t value) {
  return Put(byte_offset, static_cast<uint64_t>(value));
}

void Buffer::Append(const Buffer &source) {
  if (source.length_ == 0) return;
  size_type start_length = length_;
  Resize(length_ + source.length_);
  memcpy(&buffer_[start_length], source.buffer_, source.length_);
}

void Buffer::AddHeader(Buffer::size_type length) {
  size_type start_length = length_;
  Resize(length_ + length);
  memmove(&buffer_[length], buffer_, start_length);
  memset(buffer_, 0, length);
}

void Buffer::RemoveHeader(Buffer::size_type length) {
  if (length > 0) {
    CHECK_LE(length, length_);
    size_type end_length = length_ - length;
    memmove(buffer_, &buffer_[length], end_length);
    Resize(end_length);
  }
}

void Buffer::Copy(const Buffer &source) {
  Resize(source.length_);
  memcpy(buffer_, source.buffer_, length_);
}

#if 0
void Buffer::WriteOrDie(File *fp) const {
  size_type n = Length();
  const void *p = GetBufferPointer(n);
  fp->WriteOrDie(p, n);
}

void Buffer::WriteToPathOrDie(const char *name) const {
  FileCloser file(File::OpenOrDie(name, "w"));
  WriteOrDie(file.get());
}
#endif

std::string Buffer::Dump() const {
  size_type n = Length();
  if (n == 0) {
    return "";
  } else {
    return glibusb::Dump(&(buffer_[0]), n);
  }
}

}  // namespace glibusb
