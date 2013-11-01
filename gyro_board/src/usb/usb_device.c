/*
  LPCUSB, an USB device driver for LPC microcontrollers
  Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
     derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#include "LPCUSB/usbapi.h"
#include "LPCUSB/usbdebug.h"
#include "LPCUSB/usbstruct.h"

// This file is marked private and most of the functions in its associated .c
// file started out static, but we want to use some of them to do frame handling
// stuff because we do special stuff with it (handle it ourselves for reduced
// jitter and actually deal with the frame number correctly), so it's nice to
// have the utility functions for accessing the hardware available instead of
// having to rewrite them.
#include "LPCUSB/usbhw_lpc.h"
unsigned char USBHwCmdRead(unsigned char bCmd);
void Wait4DevInt(unsigned long dwIntr);

#include "LPC17xx.h"

#include "fill_packet.h"

#define usbMAX_SEND_BLOCK    ( 20 / portTICK_RATE_MS )
#define usbRXBUFFER_LEN      ( 80 )
#define usbTXBUFFER_LEN      ( 600 )

// Read the processor manual for picking these.
#define BULK_IN_EP    0x82
#define BULK_OUT_EP   0x05
#define ISOC_IN_EP    0x83
#define NUM_ENDPOINTS 3

#define MAX_PACKET_SIZE  64
#define DATA_PACKET_SIZE DATA_STRUCT_SEND_SIZE

#define LE_WORD(x)    ((x)&0xFF),((x)>>8)

static xQueueHandle xRxedChars = NULL, xCharsForTx = NULL;

// This gets cleared each time the ISR is entered and then checked as it's
// returning so that we can still yield from the ISR to a woken task but not
// from the middle of the ISR like it would be if this was checked in each
// endpoint handler that needs it.
static portBASE_TYPE higher_priority_task_woken;

static const unsigned char abDescriptors[] = {
// Device descriptor
  0x12,
  DESC_DEVICE,
  LE_WORD(0x0200),    // bcdUSB
  0xFF,        // bDeviceClass
  0x00,        // bDeviceSubClass
  0x00,        // bDeviceProtocol
  MAX_PACKET_SIZE0,    // bMaxPacketSize
  LE_WORD(0x1424),    // idVendor
  LE_WORD(0xd243),    // idProduct
  LE_WORD(0x0153),    // bcdDevice
  0x03,        // iManufacturer
  0x02,        // iProduct
  0x01,        // iSerialNumber
  0x01,        // bNumConfigurations

// Configuration descriptor
  0x09,
  DESC_CONFIGURATION,
  LE_WORD(9 + 9 + 7 * NUM_ENDPOINTS),  // wTotalLength
  0x01,        // bNumInterfaces
  0x01,        // bConfigurationValue
  0x00,        // iConfiguration
  0xC0,        // bmAttributes
  0x32,        // bMaxPower
// Data class interface descriptor
  0x09,
  DESC_INTERFACE,
  0x00,        // bInterfaceNumber
  0x00,        // bAlternateSetting
  NUM_ENDPOINTS,  // bNumEndPoints
  0x0A,        // bInterfaceClass = data
  0x00,        // bInterfaceSubClass
  0x00,        // bInterfaceProtocol
  0x00,        // iInterface
// Debug EP OUT
  0x07,
  DESC_ENDPOINT,
  BULK_OUT_EP,  // bEndpointAddress
  0x02,        // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),  // wMaxPacketSize
  0x00,        // bInterval
// Debug EP in
  0x07,
  DESC_ENDPOINT,
  BULK_IN_EP,      // bEndpointAddress
  0x02,        // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),  // wMaxPacketSize
  0x00,        // bInterval
  // isoc data EP IN
  0x07,
  DESC_ENDPOINT,
  ISOC_IN_EP,            // bEndpointAddress
  0x0D,              // bmAttributes = isoc, synchronous, data endpoint
  LE_WORD(DATA_PACKET_SIZE),  // wMaxPacketSize
  0x01,            // bInterval

  // string descriptors
  0x04,
  DESC_STRING,
  LE_WORD(0x0409),

  0x0E,
  DESC_STRING,
  'A', 0, 'S', 0, 'C', 0, 'H', 0, 'U', 0, 'H', 0,

  0x14,
  DESC_STRING,
  'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'n', 0, 's', 0, 'o', 0, 'r', 0,

  0x12,
  DESC_STRING,
  'A', 0, 'O', 0, 'S', 0, '_', 0, 'G', 0, 'y', 0, 'r', 0, 'o', 0,

// terminating zero
  0
};

// Enables interrupts to write data instead of NAKing on the bulk in endpoints.
// This is in a centralized place so that other NAK interrupts can be enabled
// all of the time easily in the future.
static void bulk_in_nak_int(int have_data) {
  USBHwNakIntEnable(have_data ? INACK_BI : 0);
}

/**
 * Local function to handle incoming bulk data
 *
 * @param [in] bEP
 * @param [in] bEPStatus
 */
static void DebugOut(unsigned char bEP, unsigned char bEPStatus) {
  int i, iLen;
  unsigned char abBulkBuf[64];

  (void) bEPStatus;

  // get data from USB into intermediate buffer
  iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
  for (i = 0; i < iLen; i++) {
    // put into queue
    xQueueSendFromISR(xRxedChars, &abBulkBuf[i], &higher_priority_task_woken);
  }
}


/**
 * Local function to handle outgoing bulk data
 *
 * @param [in] bEP
 * @param [in] bEPStatus
 */
static void DebugIn(unsigned char bEP, unsigned char bEPStatus) {
  int i, iLen;
  unsigned char abBulkBuf[64];

  (void) bEPStatus;

  if (uxQueueMessagesWaitingFromISR(xCharsForTx) == 0) {
    // no more data
    bulk_in_nak_int(0);
    return;
  }

  // get bytes from transmit FIFO into intermediate buffer
  for (i = 0; i < MAX_PACKET_SIZE; i++) {
    if (xQueueReceiveFromISR(xCharsForTx, &abBulkBuf[i],
                             &higher_priority_task_woken) != pdPASS) {
      break;
    }
  }
  iLen = i;

  // send over USB
  if (iLen > 0) {
    USBHwEPWrite(bEP, abBulkBuf, iLen);
  }
}


/**
 * Writes one character to VCOM port
 *
 * @param [in] c character to write
 * @returns character written, or EOF if character could not be written
 */
int VCOM_putcharFromISR(int c, long *lHigherPriorityTaskWoken) {
    char cc = (char) c;

    if (xQueueSendFromISR(xCharsForTx, &cc,
                          lHigherPriorityTaskWoken) == pdPASS) {
        return c;
    } else {
        return EOF;
    }
}

int VCOM_putchar(int c) {
    char cc = (char) c;

    // Don't block if not connected to USB.
    if (xQueueSend(xCharsForTx, &cc,
                   USBIsConnected() ? usbMAX_SEND_BLOCK : 0) == pdPASS) {
        return c;
    } else {
        return EOF;
    }
}


/**
 * Reads one character from VCOM port
 *
 * @returns character read, or EOF if character could not be read
 */
int VCOM_getchar(void) {
    unsigned char c;

    /* Block the task until a character is available. */
    if(xQueueReceive(xRxedChars, &c, 0) == pdTRUE){  //portMAX_DELAY);
        return c;
    }
    return -1;
}

// Instead of registering an lpcusb handler for this, we do it ourself so that
// we can get the timing jitter down and deal with the frame number right.
static void HandleFrame(void) {
  USB->USBDevIntClr = FRAME;

  static struct DataStruct sensor_values;
  fillSensorPacket(&sensor_values);

  // What the last good frame number that we got was.
  // Values <0 are uninitialized.
  static int32_t current_frame = -1;
  // How many extra frames we're guessing happened since we got a good one.
  static int guessed_frames = 0;

  uint8_t error_status = USBHwCmdRead(CMD_DEV_READ_ERROR_STATUS);
  if (error_status & PID_ERR) {
    ++guessed_frames;
  } else {
    int16_t read_frame = USBHwCmdRead(CMD_DEV_READ_CUR_FRAME_NR);
    USB->USBCmdCode = 0x00000200 | (CMD_DEV_READ_CUR_FRAME_NR << 16);
    Wait4DevInt(CDFULL);
    read_frame |= USB->USBCmdData << 8;

    if (current_frame < 0) {
      current_frame = read_frame;
      guessed_frames = 0;
    } else {
      // All of the complicated stuff in here tracks the frame number from
      // hardware (which comes from the SOF tokens sent out by the host) except
      // deal with it if we miss a couple or get off by a little bit (and reset
      // completely if we get off by a lot or miss a lot in a row).

      static const uint32_t kMaxReadFrame = 0x800;
      static const uint32_t kReadMask = kMaxReadFrame - 1;
      if ((current_frame & kReadMask) == read_frame) {
        // This seems like it must mean that we didn't receive the SOF token.
        ++guessed_frames;
      } else {
        guessed_frames = 0;
        // The frame number that we think we should have gotten.
        int32_t expected_frame = current_frame + guessed_frames + 1;
        int16_t difference =
            read_frame - (int16_t)(expected_frame & kReadMask);
        // If we're off by only a little.
        if (difference > -10 && difference < 10) {
          current_frame = (expected_frame & ~kReadMask) | read_frame;
          // If we're ahead by only a little (or dead on) but we wrapped.
        } else if (difference > kMaxReadFrame - 10) {
          current_frame =
              ((expected_frame & ~kReadMask) - kMaxReadFrame) | read_frame;
          // If we're behind by only a little (or dead on) but the number in the
          // token wrapped.
        } else if (difference < -(kMaxReadFrame - 10)) {
          current_frame =
              ((expected_frame & ~kReadMask) + kMaxReadFrame) | read_frame;
        } else {
          // We're way off, so give up and reset.
          current_frame = -1;
        }
      }
    }
  }

  sensor_values.frame_number = current_frame + guessed_frames;
  sensor_values.unknown_frame = guessed_frames > 10;

  USBHwEPWrite(ISOC_IN_EP, (unsigned char *)&sensor_values, DATA_PACKET_SIZE);

  if (uxQueueMessagesWaitingFromISR(xCharsForTx) > 0) {
    // Data to send is available so enable interrupt instead of NAK.
    bulk_in_nak_int(1);
  } else {
    bulk_in_nak_int(0);
  }
}

void USB_IRQHandler(void) {
  higher_priority_task_woken = pdFALSE;
  uint32_t status = SC->USBIntSt;
  if (status & USB_INT_REQ_HP) {
    // We set the frame interrupt to get routed to the high priority line.
    HandleFrame();
  }
  //if (status & USB_INT_REQ_LP) {
    // Call lpcusb to let it handle all of the other interrupts.
    USBHwISR();
  //}
  portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void usb_init(void) {
  DBG("Initialising USB stack\n");

  xRxedChars = xQueueCreate(usbRXBUFFER_LEN, sizeof(char));
  xCharsForTx = xQueueCreate(usbTXBUFFER_LEN, sizeof(char));

  if ((xRxedChars == NULL) || (xCharsForTx == NULL)) {
    /* Not enough heap available to create the buffer queues, can't do
       anything so just delete ourselves. */
    vTaskDelete(NULL);
  }

  // Initialise the USB stack.
  USBInit();

  // register descriptors
  USBRegisterDescriptors(abDescriptors);

  // register class request handler
  //USBRegisterRequestHandler(REQTYPE_TYPE_CLASS,
  //                          HandleClassRequest, abClassReqData);

  // register endpoint handlers
  USBHwRegisterEPIntHandler(BULK_IN_EP, DebugIn);
  USBHwRegisterEPIntHandler(BULK_OUT_EP, DebugOut);

  USB->USBDevIntPri = 1;  // route frame interrupt to high priority line
  USB->USBDevIntEn |= FRAME;  // enable frame interrupt

  // register frame handler
  //USBHwRegisterFrameHandler(USBFrameHandler);

  DBG("Starting USB communication\n");

  NVIC_SetPriority(USB_IRQn, configUSB_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(USB_IRQn);

  // connect to bus

  DBG("Connecting to USB bus\n");
  USBHwConnect(TRUE);

  // Enable USB.  The PC has probably disconnected it now.
  USBHwAllowConnect();
}
