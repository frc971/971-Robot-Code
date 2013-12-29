// Unit tests for the uart packet-finding algorithms.

#include "bbb/packet_finder.h"

#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <termios.h>

#include "aos/atom_code/init.h"
#include "gtest/gtest.h"

namespace bbb {
namespace {

// Special subclass for PacketFinder to feed it testing data.
class TestingReader : public PacketFinder {
 public:
  TestingReader() : reader_buf_(new AlignedChar[kPacketSize + 4]) {}
  ~TestingReader() {
    delete reader_buf_;
  }

  // Called by superclass, which thinks it's getting the 
  // bytes from a Uart.
  // Returns the amount of data read, or -1 if it reads nothing.
  // (This is so that whatever's reading registers an error if 
  // there's nothing, a behavior we want for testing.)
  virtual ssize_t ReadBytes(AlignedChar *dest, const size_t max_bytes) {
    ssize_t bytes2read = std::min(bytes_remaining_, max_bytes);
    memmove(dest, reader_buf_, bytes2read);
    bytes_remaining_ -= bytes2read;
    overshoot_ = max_bytes - bytes2read;
    return bytes2read || !max_bytes ? bytes2read : -1;
  } 
  // Called by testers to load testing data into the buffer.
  // Retuns the amount of data written.
  // It expects you to not use the data in source after this call.
  size_t LoadBytes(uint32_t *source, const size_t max_bytes) {
    size_t bytes2write = std::max((int32_t)((kPacketSize + 4) - bytes_remaining_), (int32_t)max_bytes);
    memmove(reader_buf_, source, bytes2write);
    bytes_remaining_ += bytes2write;
    return bytes2write;
  }
  // Returns how much the last call to ReadBytes overestimated
  // how much data we had available.
  inline size_t GetOvershoot() {
    return overshoot_;
  }
  // Run the FindPacket routine to read our data into the packet finder.
  inline bool DoFindPacket() {
    return FindPacket();
  }

 private:
  AlignedChar *reader_buf_;
  size_t bytes_remaining_ = 0;
  size_t overshoot_ = 0;
};

class PacketFinderTest : public ::testing::Test {
 protected:
  PacketFinderTest () 
      : read_tester_(TestingReader()) {}
  
  // Fills an array with random data.
  // size is the size in bytes of array.
  void InitRandom(uint32_t *array, size_t size) {
    ASSERT_EQ(size % 4, (size_t)0);
    srand(time(NULL));
    ASSERT_EQ(sizeof(rand()), (size_t)4);
    for (size_t filling = 0; filling < size / 4; ++filling) {
      array[filling] = rand(); 
    }
  }

  TestingReader read_tester_;
};

TEST_F(PacketFinderTest, StressTest) {
  // How many packets to throw at it during our stress test.
  constexpr uint32_t kStressPackets = 1000;

  const size_t send_size = read_tester_.kPacketSize + 4;
  ASSERT_EQ(send_size % 4, (size_t)0);
  uint32_t packets [kStressPackets] [send_size / 4];
  // Manufacture something that looks like a packet.
  for (size_t filling = 0; filling < kStressPackets; ++filling) {
    InitRandom(packets[filling], sizeof(packets[filling]));
    packets[filling][0] = 0; // Zero word packet identifier.
  }
  for (size_t filling = 0; filling < kStressPackets; ++filling) {
    // This is basically a stress test.
    // We'll make it read really fast and see if we can break it.
    ASSERT_EQ(read_tester_.LoadBytes(packets[filling], sizeof(packets[filling])),
        sizeof(packets[filling])) 
        << "Your TestingReader buffer isn't big enough, or your packet is too large.\n";
  
    // The only way this could possibly return false is if we dropped bytes.
    EXPECT_TRUE(read_tester_.DoFindPacket())
        << "I hope you have PacketFinder insurance because I broke yours.\n It dropped "
        << read_tester_.GetOvershoot()
        << " bytes.\n";
  }
}

}  // namespace
}  // namespace bbb

int main(int argc, char **argv) {
  aos::InitCreate();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
