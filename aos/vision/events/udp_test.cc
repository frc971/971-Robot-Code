#include "aos/vision/events/udp.h"

#include "gtest/gtest.h"

namespace aos {
namespace events {

TEST(UDPTest, SendRecv) {
  RXUdpSocket rx(1109);
  TXUdpSocket tx("127.0.0.01", 1109);

  int txdata[] = {1, 2, 3, 4};
  int rxdata[4];
  tx.Send(reinterpret_cast<const char *>(&txdata), sizeof(txdata));
  rx.Recv(reinterpret_cast<char *>(&rxdata), sizeof(rxdata));
  EXPECT_EQ(txdata[0], rxdata[0]);
  EXPECT_EQ(txdata[1], rxdata[1]);
  EXPECT_EQ(txdata[2], rxdata[2]);
  EXPECT_EQ(txdata[3], rxdata[3]);
}

}  // namespace events
}  // namespace aos
