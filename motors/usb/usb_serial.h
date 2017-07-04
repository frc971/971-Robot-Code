/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef USBserial_h_
#define USBserial_h_

#include "motors/usb/usb_desc.h"

#include <inttypes.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif

void usb_serial_init(void);

// Reads the next character (if any) and returns it. Returns -1 if no characters
// are available.
int usb_serial_getchar(int port);

// Returns the next character (if any) or -1.
int usb_serial_peekchar(int port);

// Reads as many bytes (up to size) as are available now. Returns 0 immediately
// if no bytes are available. Returns the number of bytes read.
int usb_serial_read(int port, void *buffer, uint32_t size);

// Drops any unread input until the most recent packet sent.
void usb_serial_flush_input(int port);

// Writes data. Returns -1 if it times out or 0 if it succeeds.
//
// NOTE: This does not send immediately. The data is buffered
int usb_serial_write(int port, const void *buffer, uint32_t size);

// Writes a single character. Returns -1 if it times out or 1 if it succeeds.
static inline int usb_serial_putchar(int port, uint8_t c) {
  return usb_serial_write(port, &c, 1);
}

// Immediately flushes all written data.
//
// TODO(Brian): What exactly are the semantics here?
void usb_serial_flush_output(int port);

#ifdef __cplusplus
}
#endif

#endif // USBserial_h_
