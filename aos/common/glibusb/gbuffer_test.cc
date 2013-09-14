// Copyright 2012 Google Inc. All Rights Reserved.
// Author: charliehotel@google.com (Christopher Hoover)
//
// Tests for Buffer.

#include "../gbuffer.h"

#include <stdint.h>
#include <limits>

#include <boost/scoped_ptr.hpp>
#include <gtest/gtest.h>

#define GG_LONGLONG(x) x##LL
#define GG_ULONGLONG(x) x##ULL
#define ARRAYSIZE(x) (static_cast<int>(sizeof(x)/sizeof(x[0])))

namespace glibusb {
namespace {

typedef ::testing::Types<int8_t, int16_t, int32_t, int64_t,
                         uint8_t, uint16_t, uint32_t, uint64_t> AllIntegerTypes;

// Tests that a newly constructed buffer is empty.
TEST(BufferTest, EmptyBufferLength) {
  Buffer buffer;
  EXPECT_EQ(0, buffer.Length());
}

// Tests clearing.
TEST(BufferTest, EmptyBufferClear) {
  Buffer buffer;
  buffer.Append(uint8_t(1));
  buffer.Clear();
  EXPECT_EQ(0, buffer.Length());
}

// Tests resizing.
TEST(BufferTest, EmptyBufferResize) {
  Buffer buffer;
  const int kSize = 100;
  buffer.Resize(kSize);
  EXPECT_EQ(kSize, buffer.Length());
}

// Tests getting a pointer on an empty buffer.
TEST(BufferTest, EmptyBufferGetPointer) {
  Buffer buffer;
  void *p;
  const void *cp;
  p = buffer.GetBufferPointer(0);
  EXPECT_EQ(NULL, p);
  cp = buffer.GetBufferPointer(0);
  EXPECT_EQ(NULL, cp);
  p = buffer.GetBufferPointer(0, 0);
  EXPECT_EQ(NULL, p);
  cp = buffer.GetBufferPointer(0, 0);
  EXPECT_EQ(NULL, cp);
}

// Tests getting a pointer on an empty buffer.
TEST(BufferTest, ConstEmptyBufferGetPointer) {
  const Buffer buffer;
  const void *cp;
  cp = buffer.GetBufferPointer(0);
  EXPECT_EQ(NULL, cp);
  cp = buffer.GetBufferPointer(0, 0);
  EXPECT_EQ(NULL, cp);
}


// Tests Get on an empty buffer.
template <typename T>
class EmptyBufferGetDeathTest : public ::testing::Test {
 public:
  void Check() {
    Buffer buffer;
    T value;
    EXPECT_DEATH(buffer.Get(0, &value), "Check failed");
  }
};

// Tests Get for all types on an empty bufer.
TYPED_TEST_CASE(EmptyBufferGetDeathTest, AllIntegerTypes);
TYPED_TEST(EmptyBufferGetDeathTest, Check) {
  this->Check();
}


// Tests Put on an empty buffer.
template <typename T>
class EmptyBufferPutDeathTest : public ::testing::Test {
 public:
  void Check() {
    Buffer buffer;
    T value(0);
    EXPECT_DEATH(buffer.Put(0, value), "Check failed");
  }
};

// Tests Put for all types on an empty bufer.
TYPED_TEST_CASE(EmptyBufferPutDeathTest, AllIntegerTypes);
TYPED_TEST(EmptyBufferPutDeathTest, Check) {
  this->Check();
}


// Tests getting a string on an empty buffer.
TEST(BufferDeathTest, EmptyBufferGetString) {
  Buffer buffer;
  std::string s;
  EXPECT_DEATH(buffer.Get(0, &s), "Check failed");
}


// Tests removing the header from an empty buffer.
TEST(BufferDeathTest, EmptyBufferRemoveHeader) {
  Buffer buffer;
  buffer.RemoveHeader(0);
  EXPECT_EQ(0, buffer.Length());
  EXPECT_DEATH(buffer.RemoveHeader(1), "Check failed");
}


// Tests adding a header of size 0.
TEST(BufferTest, EmptyBufferAddHeader) {
  Buffer buffer;
  buffer.AddHeader(0);
  EXPECT_EQ(0, buffer.Length());
}


// Tests adding a header of size > 0.
TEST(BufferTest, EmptyBufferAddHeader2) {
  Buffer buffer;
  const int kSize = 100;
  buffer.AddHeader(kSize);
  EXPECT_EQ(kSize, buffer.Length());
}


// Tests copying an empty buffer.
TEST(BufferTest, EmptyBufferCopy) {
  Buffer buffer;
  Buffer buffer2;
  buffer2.Append(uint8_t(1));
  buffer2.Copy(buffer);
  EXPECT_EQ(0, buffer2.Length());
}

// Tests dumping an empty buffer.
TEST(BufferTest, EmptyBufferDump) {
  Buffer buffer;
  std::string s = buffer.Dump();
  EXPECT_EQ("", s);
}


// Tests slicing an empty buffer.
TEST(BufferTest, EmptyBufferSlice) {
  Buffer buffer;
  boost::scoped_ptr<Buffer> slice(buffer.MakeSlice(0, 0));
  EXPECT_EQ(slice->Length(), 0);
}


// Tests Get, Put and Append for signed and unsigned integers.
template <typename T>
class PutGetAppendIntegerTest : public ::testing::Test {
 public:
  void Check() {
    static const T kValues[] = {
      std::numeric_limits<T>::max(),
      T(0),
      std::numeric_limits<T>::min()
    };

    for (int i = 0; i < ARRAYSIZE(kValues); ++i) {
      const T kValue = kValues[i];

      // Tests Put - Get
      {
        Buffer buffer;
        buffer.Resize(sizeof(T));
        buffer.Put(0, kValue);
        T check;
        buffer.Get(0, &check);
        EXPECT_EQ(kValue, check);
      }

      // Tests Append - Get
      {
        Buffer buffer;
        buffer.Append(kValue);
        T check;
        buffer.Get(0, &check);
        EXPECT_EQ(kValue, check);
      }
    }
  }
};

// Tests Get, Put and Append for all signed integers.
TYPED_TEST_CASE(PutGetAppendIntegerTest, AllIntegerTypes);
TYPED_TEST(PutGetAppendIntegerTest, Check) {
  this->Check();
}

const uint8_t kDeadBeef[] = {
  0xef, 0xbe, 0xad, 0xde
};

// Test harness for a buffer construct from "C" data.
class ConstructedFromDataBufferTest : public testing::Test {
 protected:
  void SetUp() {
    buffer.reset(new Buffer(kDeadBeef, sizeof(kDeadBeef)));
  }

  boost::scoped_ptr<Buffer> buffer;
};

typedef ConstructedFromDataBufferTest ConstructedFromDataBufferDeathTest;

// Tests constructing a buffer from "C" data.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataBufferLength) {
  EXPECT_EQ(sizeof(kDeadBeef), buffer->Length());
}

// Tests that a buffer constructed from "C" data contains the right
// data.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataByteAccess) {
  for (int i = 0; i < ARRAYSIZE(kDeadBeef); ++i) {
    uint8_t u8;
    buffer->Get(i, &u8);
    EXPECT_EQ(kDeadBeef[i], u8);
  }
}

// Tests clearing.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataClear) {
  buffer->Clear();
  EXPECT_EQ(0, buffer->Length());
}

// Tests resizing.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataResize) {
  const int kSize = 100;
  buffer->Resize(kSize);
  EXPECT_EQ(kSize, buffer->Length());
}

// Tests that getting a pointer works.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataGetPointer) {
  void *p;
  const void *cp;
  p = buffer->GetBufferPointer(0);
  EXPECT_EQ(NULL, p);
  cp = buffer->GetBufferPointer(0);
  EXPECT_EQ(NULL, cp);
  p = buffer->GetBufferPointer(0, 0);
  EXPECT_EQ(NULL, p);
  cp = buffer->GetBufferPointer(0, 0);
  EXPECT_EQ(NULL, cp);

  p = buffer->GetBufferPointer(2);
  EXPECT_TRUE(p != NULL);
  EXPECT_EQ(kDeadBeef[0], *static_cast<uint8_t *>(p));
  EXPECT_EQ(kDeadBeef[1], *(static_cast<uint8_t *>(p) + 1));
  cp = buffer->GetBufferPointer(2);
  EXPECT_TRUE(p != NULL);
  EXPECT_EQ(kDeadBeef[0], *static_cast<uint8_t *>(p));
  EXPECT_EQ(kDeadBeef[1], *(static_cast<uint8_t *>(p) + 1));

  p = buffer->GetBufferPointer(1, 2);
  EXPECT_TRUE(p != NULL);
  EXPECT_EQ(kDeadBeef[1], *static_cast<uint8_t *>(p));
  EXPECT_EQ(kDeadBeef[2], *(static_cast<uint8_t *>(p) + 1));
  cp = buffer->GetBufferPointer(1, 2);
  EXPECT_TRUE(p != NULL);
  EXPECT_EQ(kDeadBeef[1], *static_cast<uint8_t *>(p));
  EXPECT_EQ(kDeadBeef[2], *(static_cast<uint8_t *>(p) + 1));

  const Buffer &const_buffer(*buffer);
  cp = const_buffer.GetBufferPointer(1, 2);
  EXPECT_TRUE(p != NULL);
  EXPECT_EQ(kDeadBeef[1], *static_cast<uint8_t *>(p));
  EXPECT_EQ(kDeadBeef[2], *(static_cast<uint8_t *>(p) + 1));
}

// Tests that Get{S,U}{8,16,32,64} work on zero.
TEST(BufferTest, GetZero) {
  boost::scoped_ptr<Buffer> buffer(new Buffer());
  for (int i = 0; i < 8; ++i) {
    buffer->Append(uint8_t(0));
  }
  int8_t s8;
  uint8_t u8;
  int16_t s16;
  uint16_t u16;
  int32_t s32;
  uint32_t u32;
  int64_t s64;
  uint64_t u64;
  buffer->Get(0, &s8);
  EXPECT_EQ(0, s8);
  buffer->Get(0, &u8);
  EXPECT_EQ(0, u8);
  buffer->Get(0, &s16);
  EXPECT_EQ(0, s16);
  buffer->Get(0, &u16);
  EXPECT_EQ(0, u16);
  buffer->Get(0, &s32);
  EXPECT_EQ(0, s32);
  buffer->Get(0, &u32);
  EXPECT_EQ(0, u32);
  buffer->Get(0, &s64);
  EXPECT_EQ(0, s64);
  buffer->Get(0, &u64);
  EXPECT_EQ(0, u64);
}

// Tests that GetU{8,16,32,64} work.
TEST(BufferTest, GetUXX) {
  boost::scoped_ptr<Buffer> buffer(new Buffer());
  buffer->Append(uint8_t(0x88));
  buffer->Append(uint8_t(0x77));
  buffer->Append(uint8_t(0x66));
  buffer->Append(uint8_t(0x55));
  buffer->Append(uint8_t(0x44));
  buffer->Append(uint8_t(0x33));
  buffer->Append(uint8_t(0x22));
  buffer->Append(uint8_t(0x11));
  uint8_t u8;
  uint16_t u16;
  uint32_t u32;
  uint64_t u64;
  buffer->Get(0, &u8);
  EXPECT_EQ(0x88, u8);
  buffer->Get(0, &u16);
  EXPECT_EQ(0x7788, u16);
  buffer->Get(0, &u32);
  EXPECT_EQ(0x55667788, u32);
  buffer->Get(0, &u64);
  EXPECT_EQ(GG_ULONGLONG(0x1122334455667788), u64);
}

// Tests that GetS{8,16,32,64} work for positive values.
TEST(BufferTest, GetSXXPositive) {
  boost::scoped_ptr<Buffer> buffer(new Buffer());
  buffer->Append(uint8_t(0x08));
  buffer->Append(uint8_t(0x07));
  buffer->Append(uint8_t(0x06));
  buffer->Append(uint8_t(0x05));
  buffer->Append(uint8_t(0x04));
  buffer->Append(uint8_t(0x03));
  buffer->Append(uint8_t(0x02));
  buffer->Append(uint8_t(0x01));
  int8_t s8;
  int16_t s16;
  int32_t s32;
  int64_t s64;
  buffer->Get(0, &s8);
  EXPECT_EQ(0x08, s8);
  buffer->Get(0, &s16);
  EXPECT_EQ(0x0708, s16);
  buffer->Get(0, &s32);
  EXPECT_EQ(0x05060708, s32);
  buffer->Get(0, &s64);
  EXPECT_EQ(GG_ULONGLONG(0x0102030405060708), s64);
}

// Tests that GetS{8,16,32,64} work for negative values.
TEST(BufferTest, GetSXXNegative) {
  boost::scoped_ptr<Buffer> buffer(new Buffer());
  buffer->Append(uint8_t(0xF8));
  buffer->Append(uint8_t(0xF7));
  buffer->Append(uint8_t(0x06));
  buffer->Append(uint8_t(0xF5));
  buffer->Append(uint8_t(0x04));
  buffer->Append(uint8_t(0x03));
  buffer->Append(uint8_t(0x02));
  buffer->Append(uint8_t(0xF1));

  // Calculate directly the the signed (2's complement) value that we
  // should expect.
  const int8_t kExpected8 = 0xF8 - 0xFF - 1;
  int8_t s8;
  buffer->Get(0, &s8);
  EXPECT_EQ(kExpected8, s8);

  const int16_t kExpected16 = 0xF7F8 - 0xFFFF - 1;
  int16_t s16;
  buffer->Get(0, &s16);
  EXPECT_EQ(kExpected16, s16);

  const int32_t kExpected32 = 0xF506F7F8 - 0xFFFFFFFF - 1;
  int32_t s32;
  buffer->Get(0, &s32);
  EXPECT_EQ(kExpected32, s32);

  const int64_t kExpected64 = (GG_LONGLONG(0xF1020304F506F7F8) -
                               GG_LONGLONG(0xFFFFFFFFFFFFFFFF) -
                               GG_LONGLONG(1));
  int64_t s64;
  buffer->Get(0, &s64);
  EXPECT_EQ(kExpected64, s64);
}

// Tests that getting a string works.
TEST_F(ConstructedFromDataBufferDeathTest, ConstructedFromDataGetString) {
  std::string s;
  EXPECT_DEATH(buffer->Get(0, &s), "Check failed");
  buffer->Append(uint8_t(0));
  Buffer::size_type n = buffer->Get(0, &s);
  EXPECT_STREQ("\xEF\xBE\xAD\xDE", s.c_str());
  EXPECT_EQ(4, n);
  n = buffer->Get(1, &s);
  EXPECT_STREQ("\xBE\xAD\xDE", s.c_str());
  EXPECT_EQ(4, n);
}

// Tests removing a header.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataRemoveHeader) {
  buffer->RemoveHeader(0);
  EXPECT_EQ(sizeof(kDeadBeef), buffer->Length());
  buffer->RemoveHeader(2);
  EXPECT_EQ(sizeof(kDeadBeef) - 2, buffer->Length());
  uint16_t u16;
  buffer->Get(0, &u16);
  EXPECT_EQ(0xdead, u16);
}

// Tests adding a zero-length header.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataAddHeader) {
  buffer->AddHeader(0);
  EXPECT_EQ(sizeof(kDeadBeef), buffer->Length());
}

// Tests adding a header of size > 0.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataAddHeader2) {
  const int kSize = 100;
  buffer->AddHeader(kSize);
  EXPECT_EQ(sizeof(kDeadBeef) + kSize, buffer->Length());
  uint32_t u32;
  buffer->Get(kSize, &u32);
  EXPECT_EQ(0xdeadbeef, u32);
}

// Tests copying.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataCopy) {
  buffer->Append(uint8_t(0));
  std::string s;
  buffer->Get(0, &s);

  Buffer buffer2;
  buffer2.Copy(*buffer);
  std::string s2;
  buffer2.Get(0, &s2);

  EXPECT_STREQ(s.c_str(), s2.c_str());
}

// Tests dumping.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataDump) {
  const char kExpected[] =
      "0x00000000: ef be ad de                                      | ....\n";
  std::string s = buffer->Dump();
  EXPECT_STREQ(kExpected, s.c_str());
}

// Tests emtpy slicing.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataSlice) {
  boost::scoped_ptr<Buffer> slice(buffer->MakeSlice(0, 0));
  EXPECT_EQ(slice->Length(), 0);
}

// Tests slicing.
TEST_F(ConstructedFromDataBufferTest, ConstructedFromDataSlice2) {
  boost::scoped_ptr<Buffer> slice(buffer->MakeSlice(0, 2));
  EXPECT_EQ(slice->Length(), 2);
  uint16_t u16;
  slice->Get(0, &u16);
  EXPECT_EQ(0xbeef, u16);
}

// Tests dumping.
TEST(BufferTest, DumpTest) {
  const char kData[] = "Hello";
  const char kExpected[] =
      "0x00000000: 48 65 6c 6c 6f 00                       "
      "         | Hello.\n";
  Buffer buffer(kData, sizeof(kData));
  std::string s = buffer.Dump();
  EXPECT_STREQ(kExpected, s.c_str());
}

#if 0
// Tests writing to a file.
TEST(BufferTest, FileTest) {
  const char kData[] = "Hello";
  Buffer buffer(kData, sizeof(kData));
  string out;
  FileCloser file(MutableStringFile("file", &out,
                                    DO_NOT_TAKE_OWNERSHIP,
                                    DO_NOT_ALLOW_MMAP));
  buffer.WriteOrDie(file.get());
  EXPECT_STREQ(kData, out.c_str());
}

TEST(BufferTest, WritePathOrDieTest) {
  const char kData[] = "Hello";
  Buffer buffer(kData, sizeof(kData));
  buffer.WriteToPathOrDie("/dev/null");
}
#endif

// Tests appending.
TEST(BufferTest, AppendBuffer) {
  const char kData1[] = "Hello ";
  const char kData2[] = "World";
  Buffer buffer1(kData1, sizeof(kData1) - 1);
  EXPECT_EQ(sizeof(kData1) - 1, buffer1.Length());
  Buffer buffer2(kData2, sizeof(kData2));
  EXPECT_EQ(sizeof(kData2), buffer2.Length());
  Buffer buffer;
  EXPECT_EQ(0, buffer.Length());
  buffer.Append(Buffer());
  buffer.Append(buffer1);
  buffer.Append(buffer2);
  std::string s;
  buffer.Get(0, &s);
  EXPECT_STREQ("Hello World", s.c_str());
}

// Tests operator==
TEST(Buffer, OpEqualTrue) {
  Buffer b1;
  Buffer b2;
  EXPECT_EQ(b1, b2);
  b1.Append(uint8_t(1));
  b2.Append(uint8_t(1));
  EXPECT_EQ(b1, b2);
}

// Tests operator==
TEST(Buffer, OpEqualFalse) {
  Buffer empty;
  Buffer b1;
  Buffer b2;
  Buffer b12;
  b1.Append(uint8_t(1));
  b2.Append(uint8_t(2));
  b12.Append(uint8_t(1));
  b12.Append(uint8_t(2));
  EXPECT_NE(empty, b1);
  EXPECT_NE(b1, empty);
  EXPECT_NE(b1, b2);
  EXPECT_NE(b1, b12);
  EXPECT_NE(b12, b1);
}

TEST(Buffer, Dump) {
  Buffer b;
  b.Append(uint8_t(1));
  std::string s = b.Dump();
}

}  // namespace
}  // namespace glibusb
