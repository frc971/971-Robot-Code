#include "y2019/jevois/serial.h"

#include "aos/logging/logging.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace y2019 {
namespace jevois {

int open_via_terminos(const char *tty_name) {
  int itsDev = ::open(tty_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (itsDev == -1) {
    AOS_LOG(FATAL, "problem opening: %s\n", tty_name);
  }

  termios options = {};
  if (tcgetattr(itsDev, &options) == -1)
    AOS_LOG(FATAL, "Failed to get options");

  // get raw input from the port
  options.c_cflag |= (CLOCAL     // ignore modem control lines
                      | CREAD);  // enable the receiver

  options.c_iflag &= ~(IGNBRK    // ignore BREAK condition on input
                       | BRKINT  // If IGNBRK is not set, generate SIGINT on
                                 // BREAK condition, else read BREAK as \0
                       | PARMRK | ISTRIP  // strip off eighth bit
                       | INLCR            // donot translate NL to CR on input
                       | IGNCR            // ignore CR
                       | ICRNL            // translate CR to newline on input
                       | IXON  // disable XON/XOFF flow control on output
                       );

  // disable implementation-defined output processing
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ECHO      // dont echo i/p chars
                       | ECHONL  // do not echo NL under any circumstance
                       | ICANON  // disable cannonical mode
                       | ISIG    // do not signal for INTR, QUIT, SUSP etc
                       | IEXTEN  // disable platform dependent i/p processing
                       );

  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // Set the number of bits:
  options.c_cflag &= ~CSIZE;  // mask off the 'size' bits
  options.c_cflag |= CS8;

  // Set parity option:
  options.c_cflag &= ~(PARENB | PARODD);

  // Set the stop bits option:
  options.c_cflag &= ~CSTOPB;

  // Set the flow control:
  options.c_cflag &= ~CRTSCTS;
  options.c_iflag &= ~(IXON | IXANY | IXOFF);

  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 2;

  // Flow control:
  // flow soft:
  // options.c_iflag |= (IXON | IXANY | IXOFF);
  // flow hard:
  // options.c_cflag |= CRTSCTS;

  if (tcsetattr(itsDev, TCSANOW, &options) == -1)
    AOS_LOG(FATAL, "Failed to set port options");
  return itsDev;
}

}  // namespace jevois
}  // namespace y2019
