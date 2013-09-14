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

Buffer::Buffer() {}

Buffer::~Buffer() {}

Buffer::Buffer(const void *src, Buffer::size_type length) {
  buffer_.resize(length);
  if (length > 0) {
    uint8_t *dst = &(buffer_[0]);
    memcpy(dst, src, length);
  }
}

bool Buffer::operator==(const Buffer &other) const {
  return buffer_ == other.buffer_;
}

bool Buffer::operator!=(const Buffer &other) const {
  return !(*this == other);
}

Buffer *Buffer::MakeSlice(Buffer::size_type offset,
                          Buffer::size_type length) const {
  CHECK_LE(offset + length, buffer_.size());
  if (length == 0) {
    return new Buffer();
  } else {
    const uint8_t *p = &(buffer_[offset]);
    return new Buffer(p, length);
  }
}

void Buffer::Clear() {
  buffer_.clear();
}

void Buffer::Resize(Buffer::size_type length) {
  buffer_.resize(length);
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
    CHECK_LE(offset + length, buffer_.size());
    uint8_t *p = &(buffer_[offset]);
    return static_cast<void *>(p);
  }
}

const void *Buffer::GetBufferPointer(Buffer::size_type offset,
                                     Buffer::size_type length) const {
  if (length == 0) {
    return NULL;
  } else {
    CHECK_LE(offset + length, buffer_.size());
    const uint8_t *p = &(buffer_[offset]);
    return static_cast<const void *>(p);
  }
}

// Specialized template for Get
template <>
Buffer::size_type Buffer::Get(Buffer::size_type byte_offset,
                              uint8_t *value_out) const {
  CHECK_LT(byte_offset, buffer_.size());
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
  CHECK_LT(byte_offset + 1, buffer_.size());
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
  CHECK_LT(byte_offset + 3, buffer_.size());
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
  CHECK_LT(byte_offset + 7, buffer_.size());
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
  CHECK_LT(byte_offset, buffer_.size());
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
  CHECK_LT(byte_offset + 1, buffer_.size());
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
  CHECK_LT(byte_offset + 3, buffer_.size());
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
  CHECK_LT(byte_offset + 7, buffer_.size());
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
  buffer_.insert(buffer_.end(), source.buffer_.begin(), source.buffer_.end());
}

void Buffer::AddHeader(Buffer::size_type length) {
  buffer_.insert(buffer_.begin(), length, 0);
}

void Buffer::RemoveHeader(Buffer::size_type length) {
  if (length > 0) {
    CHECK_LE(length, buffer_.size());
    buffer_.erase(buffer_.begin(), buffer_.begin() + length);
  }
}

void Buffer::Copy(const Buffer &source) {
  buffer_.assign(source.buffer_.begin(), source.buffer_.end());
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
