#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "bbb_cape/src/cape/cows.h"
#include "uart_receiver.h"

// This is the code for receiving data from the cape via UART.
// NOTE: In order for this to work, you MUST HAVE "capemgr.enable_partno=BB_UART1"
// in your BBB's /media/BEAGLEBONE/uEnv.txt file!

int UartReceiver::ReadUart(size_t packet_size) {
  termios cape;
  char byte_in[packet_size];
  int fd, bread;

  if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0) {
    LOG(FATAL, "Open() failed with error %d. 
      (Did you read my note in uart_receiver.cc?)\n", fd);
  }

  if ((ret = tcgetattr(fd, &cape)) != 0) {
    LOG(ERROR, "Tcgetattr() failed with error %d.\n", ret);
    return -1;
  }
}
