// Unit tests for the uart packet-finding algorithms.

#include "bbb/packet_finder.h"

#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <termios.h>

#include "gtest/gtest.h"

#include "cape/cows.h"

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
    return bytes2read == 0 ? -1 : bytes2read;
  } 
  // Called by testers to load testing data into the buffer.
  // Retuns the amount of data written.
  // It expects you to not use the data in source after this call.
  size_t LoadBytes(AlignedChar *source, const size_t max_bytes) {
    size_t bytes2write = std::max((kPacketSize + 4) - bytes_remaining_, max_bytes);
    memmove(reader_buf_, source, bytes2write);
    bytes_remaining_ += bytes2write;
    return bytes2write;
  }
  // Returns how much the last call to ReadBytes overestimated
  // how much data we had available.
  inline size_t GetOvershoot() {
    return overshoot_;
  }
 private:
  AlignedChar *reader_buf_;
  size_t bytes_remaining_ = 0;
  size_t overshoot_ = 0;
};

class PacketFinderTest : public ::testing::Test {
 protected:
  PacketFinderTest () 
      : test_instance_(PacketFinder())
        read_tester_(TestingReader()){}
  
  // Fills an array with random data.
  // size is the size in bytes of array.
  void InitRandom(uint32_t *array, size_t size) {
    ASSERT_EQ(size % 4, 0);
    srand(time(NULL));
    ASSERT_EQ(sizeof(rand()), 32);
    // TODO (danielp) figure out size of rand().
    for (size_t filling = 0; filling < size; ++filling) {
      array[filling] = rand(); 
    }
  }

  PacketFinder test_instance_;
  TestingReader read_tester_;
};

TEST_F(PacketFinderTest, StressTest) {
  // How many packets to throw at it during our stress test.
  constexpr uint32_t kStressPackets = 1000;

  const size_t send_size = test_instance_.kPacketSize + 4;
  ASSERT_EQ(send_size % 4, 0);
  uint32_t [send_size / 4] [kStressPackets] packets;
  // Manufacture something that looks like a packet.
  for (size_t filling; filling < kStressPackets; ++filling) {
    InitRandom(packets[filling], sizeof(packets[filling]));
    packet[0] = 0; // Zero word packet identifier.
  }
  for (size_t filling; filling < kStressPackets; ++filling) {
    // This is basically a stress test.
    // We'll make it read really fast and see if we can break it.
    ASSERT_EQ(read_tester.LoadBytes(static_cast<AlignedChar *>(packet), sizeof(packet)),
        sizeof(packet)) 
        << "Your TestingReader buffer isn't big enough, or your packet is too large.\n";
  
    // The only way this could possibly return false is if we dropped bytes.
    EXPECT_TRUE(FindPacket())
        << "I hope you have PacketFinder insurance because I broke yours.\n It dropped "
        << GetOvershoot()
        << " bytes.\n";
  }
}

}  // namespace
}  // namespace bbb
