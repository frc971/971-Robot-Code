#include "bbb/uart_reader.h"

#include <termios.h>

#include "gtest/gtest.h"

namespace bbb {
namespace {

class UartReaderTest : public ::testing::Test {
protected:
  UartReader test_instance_;

public:
  UartReaderTest () : test_instance_(UartReader(3000000)) {}
};

#if 0
TEST_F(UartReaderTest, SetUpTest) {
  // Test its ability to open a file descriptor and set a baud rate.
  ASSERT_EQ(test_instance_.SetUp(), 0);
}
#endif

}  // namespace
}  // namespace bbb
