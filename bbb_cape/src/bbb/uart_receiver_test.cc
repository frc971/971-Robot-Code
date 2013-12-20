#include <termios.h>

#include "gtest/gtest.h"
#include "uart_receiver.h"

namespace bbb {
namespace {

class UartReceiverTest : public ::testing::Test {
protected:
  UartReceiver test_instance_;

public:
  UartReceiverTest () : test_instance_(UartReceiver(3000000)) {}
};

TEST_F(UartReceiverTest, SetUpTest) {
  // Test its ability to open a file descriptor and set a baud rate.
  ASSERT_EQ(test_instance_.SetUp(), 0);
}

} //namespace
} //bbb
