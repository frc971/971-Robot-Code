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

#include "motors/usb/usb_dev.h"
#include "motors/usb/usb_serial.h"
#include <string.h>

// These are shared between the two serial ports because they're ignored
// anyways.
uint32_t usb_cdc_line_coding[2];
volatile uint32_t usb_cdc_line_rtsdtr_millis;
volatile uint8_t usb_cdc_line_rtsdtr=0;

typedef struct {
	usb_packet_t *rx_packet;
	usb_packet_t *tx_packet;

	uint32_t rx_endpoint;
	uint32_t tx_endpoint;

	volatile uint8_t tx_noautoflush;
	volatile uint8_t transmit_flush_timer;
} State;
State states[2];

volatile uint8_t *usb_cdc_transmit_flush_timer =
    &states[0].transmit_flush_timer;
volatile uint8_t *usb_cdc2_transmit_flush_timer =
    &states[1].transmit_flush_timer;

#define TRANSMIT_FLUSH_TIMEOUT	5   /* in milliseconds */

void usb_serial_init(void) {
	states[0].rx_endpoint = CDC_RX_ENDPOINT;
	states[0].tx_endpoint = CDC_TX_ENDPOINT;
	states[1].rx_endpoint = CDC2_RX_ENDPOINT;
	states[1].tx_endpoint = CDC2_TX_ENDPOINT;
}

// get the next character, or -1 if nothing received
int usb_serial_getchar(int port)
{
	State *const state = &states[port];
	unsigned int i;
	int c;

	if (!state->rx_packet) {
		if (!usb_configuration) return -1;
		state->rx_packet = usb_rx(state->rx_endpoint);
		if (!state->rx_packet) return -1;
	}
	i = state->rx_packet->index;
	c = state->rx_packet->buf[i++];
	if (i >= state->rx_packet->len) {
		usb_free(state->rx_packet);
		state->rx_packet = NULL;
	} else {
		state->rx_packet->index = i;
	}
	return c;
}

// peek at the next character, or -1 if nothing received
int usb_serial_peekchar(int port)
{
	State *const state = &states[port];
	if (!state->rx_packet) {
		if (!usb_configuration) return -1;
		state->rx_packet = usb_rx(state->rx_endpoint);
		if (!state->rx_packet) return -1;
	}
	if (!state->rx_packet) return -1;
	return state->rx_packet->buf[state->rx_packet->index];
}

// read a block of bytes to a buffer
int usb_serial_read(int port, void *buffer, uint32_t size)
{
	State *const state = &states[port];
	uint8_t *p = (uint8_t *)buffer;
	uint32_t qty, count=0;

	while (size) {
		if (!usb_configuration) break;
		if (!state->rx_packet) {
			rx:
			state->rx_packet = usb_rx(state->rx_endpoint);
			if (!state->rx_packet) break;
			if (state->rx_packet->len == 0) {
				usb_free(state->rx_packet);
				goto rx;
			}
		}
		qty = state->rx_packet->len - state->rx_packet->index;
		if (qty > size) qty = size;
		memcpy(p, state->rx_packet->buf + state->rx_packet->index, qty);
		p += qty;
		count += qty;
		size -= qty;
		state->rx_packet->index += qty;
		if (state->rx_packet->index >= state->rx_packet->len) {
			usb_free(state->rx_packet);
			state->rx_packet = NULL;
		}
	}
	return count;
}

// discard any buffered input
void usb_serial_flush_input(int port)
{
	State *const state = &states[port];
	usb_packet_t *rx;

	if (!usb_configuration) return;
	if (state->rx_packet) {
		usb_free(state->rx_packet);
		state->rx_packet = NULL;
	}
	while (1) {
		rx = usb_rx(state->rx_endpoint);
		if (!rx) break;
		usb_free(rx);
	}
}

// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
#define TX_PACKET_LIMIT 8

// When the PC isn't listening, how long do we wait before discarding data?  If this is
// too short, we risk losing data during the stalls that are common with ordinary desktop
// software.  If it's too long, we stall the user's program when no software is running.
#define TX_TIMEOUT_MSEC 70

#if F_CPU == 240000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1600)
#elif F_CPU == 216000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1440)
#elif F_CPU == 192000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1280)
#elif F_CPU == 180000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1200)
#elif F_CPU == 168000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1100)
#elif F_CPU == 144000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 932)
#elif F_CPU == 120000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 764)
#elif F_CPU == 96000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 596)
#elif F_CPU == 72000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 512)
#elif F_CPU == 48000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 428)
#elif F_CPU == 24000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 262)
#endif

int usb_serial_write(int port, const void *buffer, uint32_t size)
{
	State *const state = &states[port];
	uint32_t len;
	const uint8_t *src = (const uint8_t *)buffer;
	uint8_t *dest;

	state->tx_noautoflush = 1;
	while (size > 0) {
		if (!state->tx_packet) {
			while (1) {
				if (!usb_configuration) {
					state->tx_noautoflush = 0;
					return -1;
				}
				if (usb_tx_packet_count(state->tx_endpoint) < TX_PACKET_LIMIT) {
					state->tx_noautoflush = 1;
					state->tx_packet = usb_malloc();
					if (state->tx_packet) break;
					state->tx_noautoflush = 0;
				}
				return -1;
			}
		}
		len = CDC_TX_SIZE - state->tx_packet->index;
		if (len > size) len = size;
		dest = state->tx_packet->buf + state->tx_packet->index;
		state->tx_packet->index += len;
		size -= len;
		while (len-- > 0) *dest++ = *src++;
		if (state->tx_packet->index >= CDC_TX_SIZE) {
			state->tx_packet->len = CDC_TX_SIZE;
			usb_tx(state->tx_endpoint, state->tx_packet);
			state->tx_packet = NULL;
		}
		state->transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	}
	state->tx_noautoflush = 0;
	return size;
}

void usb_serial_flush_output(int port)
{
	State *const state = &states[port];
	if (!usb_configuration) return;
	state->tx_noautoflush = 1;
	if (state->tx_packet) {
		state->transmit_flush_timer = 0;
		state->tx_packet->len = state->tx_packet->index;
		usb_tx(state->tx_endpoint, state->tx_packet);
		state->tx_packet = NULL;
	} else {
		usb_packet_t *tx = usb_malloc();
		if (tx) {
			state->transmit_flush_timer = 0;
			usb_tx(state->tx_endpoint, tx);
		} else {
			state->transmit_flush_timer = 1;
		}
	}
	state->tx_noautoflush = 0;
}

void usb_serial_flush_callback(int port)
{
	State *const state = &states[port];
	if (state->tx_noautoflush) return;
	if (state->tx_packet) {
		state->tx_packet->len = state->tx_packet->index;
		usb_tx(state->tx_endpoint, state->tx_packet);
		state->tx_packet = NULL;
	} else {
		usb_packet_t *tx = usb_malloc();
		if (tx) {
			usb_tx(state->tx_endpoint, tx);
		} else {
			state->transmit_flush_timer = 1;
		}
	}
}
