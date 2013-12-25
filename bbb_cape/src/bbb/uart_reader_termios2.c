// This file is to work around #include conflicts when including the kernel's
// termios.h file to get the new termios2 struct.

#include <asm/termios.h>

// We can't even #include <sys/ioctl.h>...
extern int ioctl(int d, int request, ...);

int aos_uart_reader_set_tty_options(int fd, int baud_rate) {
	struct termios2 options;
	if (ioctl(fd, TCGETS2, &options) != 0) {
    return -1;
	}

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
											 ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	options.c_ispeed = baud_rate;
	options.c_ospeed = baud_rate;
	options.c_cflag = (options.c_cflag & ~CBAUD) | BOTHER;

  options.c_cflag |= CLOCAL;  // ignore modem flow control
  options.c_cflag |= CREAD;  // enable receiver
  options.c_cflag &= ~CRTSCTS;  // disable flow control
  //options.c_cflag |= PARENB;  // enable parity; defaults to even
  options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;  // 8 bits
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_iflag = 0;
  // Replace any received characters with parity or framing errors with 0s.
  options.c_iflag = (options.c_iflag & ~(IGNPAR | PARMRK)) | INPCK;
  //options.c_iflag |= IGNCR | PARMRK;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VMIN] = 20;
  options.c_cc[VTIME] = 0;

	if (ioctl(fd, TCSETS2, &options) != 0) {
    return -1;
	}

  return 0;
}
