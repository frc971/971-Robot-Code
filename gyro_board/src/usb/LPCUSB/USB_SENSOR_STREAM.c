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

#include "usbapi.h"
#include "usbdebug.h"
#include "usbstruct.h"

#include "LPC17xx.h"

#include "analog.h"

#define usbMAX_SEND_BLOCK    ( 20 / portTICK_RATE_MS )
#define usbRXBUFFER_LEN      ( 80 )
#define usbTXBUFFER_LEN      ( 600 )

#define INT_IN_EP     0x81 //read manual for picking these...
#define INT_OUT_EP    0x04
#define BULK_IN_EP    0x82
#define BULK_OUT_EP   0x05
#define ISOC_IN_EP    0x83

#define MAX_PACKET_SIZE  64

#define LE_WORD(x)    ((x)&0xFF),((x)>>8)

static struct DataStruct usbPacket;

static xQueueHandle xRxedChars = NULL, xCharsForTx = NULL;

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
  LE_WORD(46),      // wTotalLength
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
  0x05,        // bNumEndPoints
  0x0A,        // bInterfaceClass = data
  0x00,        // bInterfaceSubClass
  0x00,        // bInterfaceProtocol
  0x00,        // iInterface
// Debug EP OUT
  0x07,
  DESC_ENDPOINT,
  BULK_OUT_EP,      // bEndpointAddress
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
// Data EP OUT
  0x07,
  DESC_ENDPOINT,
  INT_OUT_EP,      // bEndpointAddress
  0x03,        // bmAttributes = intr
  LE_WORD(MAX_PACKET_SIZE),  // wMaxPacketSize
  0x01,        // bInterval
// Data EP in
  0x07,
  DESC_ENDPOINT,
  INT_IN_EP,      // bEndpointAddress
  0x03,        // bmAttributes = intr
  LE_WORD(MAX_PACKET_SIZE),  // wMaxPacketSize
  0x01,        // bInterval
  
  // isoc data EP IN
  0x07,
  DESC_ENDPOINT,
  ISOC_IN_EP,            // bEndpointAddress
  0x0D,              // bmAttributes = isoc, synchronous, data endpoint
  LE_WORD(MAX_PACKET_SIZE),  // wMaxPacketSize
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


/**
 * Local function to handle incoming bulk data
 *
 * @param [in] bEP
 * @param [in] bEPStatus
 */
static void DebugOut(unsigned char bEP, unsigned char bEPStatus) {
  int i, iLen;
  long lHigherPriorityTaskWoken = pdFALSE;
  unsigned char abBulkBuf[64];

  (void) bEPStatus;

  // get data from USB into intermediate buffer
  iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
  for (i = 0; i < iLen; i++) {
    // put into queue
    xQueueSendFromISR(xRxedChars, &(abBulkBuf[ i ]), &lHigherPriorityTaskWoken);
  }

  portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}


/**
 * Local function to handle outgoing bulk data
 *
 * @param [in] bEP
 * @param [in] bEPStatus
 */
static void DebugIn(unsigned char bEP, unsigned char bEPStatus) {
  int i, iLen;
  long lHigherPriorityTaskWoken = pdFALSE;
  unsigned char abBulkBuf[64];

  (void) bEPStatus;

  if (uxQueueMessagesWaitingFromISR(xCharsForTx) == 0) {
    // no more data, disable further NAK interrupts until next USB frame
    USBHwNakIntEnable(INACK_II);
    return;
  }

  // get bytes from transmit FIFO into intermediate buffer
  for (i = 0; i < MAX_PACKET_SIZE; i++) {
    if (xQueueReceiveFromISR(xCharsForTx, (&abBulkBuf[i]),
                             &lHigherPriorityTaskWoken) != pdPASS) {
      break;
    }
  }
  iLen = i;

  // send over USB
  if (iLen > 0) {
    USBHwEPWrite(bEP, abBulkBuf, iLen);
  }

  portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}


static unsigned char abDataBuf[64];
int VCOM_putcharFromISR(int c, long *woken);
static void DataOut(unsigned char bEP, unsigned char bEPStatus) {
    int iLen;
    long lHigherPriorityTaskWoken = pdFALSE;
    /*
       char *a = "hello\n";
       while(*a){
         VCOM_putcharFromISR(*a,&lHigherPriorityTaskWoken);
         a ++;
       }
       */
    iLen = USBHwEPRead(bEP, abDataBuf, sizeof(abDataBuf));
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}

static void DataIn(unsigned char bEP, unsigned char bEPStatus) {
    long lHigherPriorityTaskWoken = pdFALSE;
  	fillSensorPacket(&usbPacket);
    USBHwEPWrite(bEP, (unsigned char *)&usbPacket, sizeof(usbPacket));
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
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

/**
 * Interrupt handler
 *
 * Simply calls the USB ISR
 */
void USB_IRQHandler(void) {
  USBHwISR();
}

static void USBFrameHandler(unsigned short wFrame) {
  (void) wFrame;
  if (uxQueueMessagesWaitingFromISR(xCharsForTx) > 0) {
    // data available, enable interrupt instead of NAK on bulk in too
    USBHwNakIntEnable(INACK_BI | INACK_II);
  } else {
    USBHwNakIntEnable(INACK_II);
  }

  fillSensorPacket(&usbPacket);
  USBHwEPWrite(ISOC_IN_EP, (unsigned char *)&usbPacket, sizeof(usbPacket));
}

void vUSBTask(void *pvParameters) {
  portTickType xLastFlashTime;

  (void) pvParameters;
  DBG("Initialising USB stack\n");

  xRxedChars = xQueueCreate(usbRXBUFFER_LEN, sizeof(char));
  xCharsForTx = xQueueCreate(usbTXBUFFER_LEN, sizeof(char));

  if ((xRxedChars == NULL) || (xCharsForTx == NULL)) {
    /* Not enough heap available to create the buffer queues, can't do
       anything so just delete ourselves. */
    vTaskDelete(NULL);
  }

  // initialise stack
  USBInit();

  // register descriptors
  USBRegisterDescriptors(abDescriptors);

  // register class request handler
  //USBRegisterRequestHandler(REQTYPE_TYPE_CLASS,
  //                          HandleClassRequest, abClassReqData);

  // register endpoint handlers
  USBHwRegisterEPIntHandler(INT_IN_EP, DataIn);
  USBHwRegisterEPIntHandler(INT_OUT_EP, DataOut);
  USBHwRegisterEPIntHandler(BULK_IN_EP, DebugIn);
  USBHwRegisterEPIntHandler(BULK_OUT_EP, DebugOut);

  // register frame handler
  USBHwRegisterFrameHandler(USBFrameHandler);

  // enable bulk-in interrupts on NAKs
  USBHwNakIntEnable(INACK_BI);

  DBG("Starting USB communication\n");

  NVIC_SetPriority(USB_IRQn, configUSB_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(USB_IRQn);

  // connect to bus

  DBG("Connecting to USB bus\n");
  USBHwConnect(TRUE);

  xLastFlashTime = xTaskGetTickCount();

  vTaskDelayUntil(&xLastFlashTime, 1000 / portTICK_RATE_MS * 100);

  //USBHwAllowConnect();
  // echo any character received (do USB stuff in interrupt)
  for (;;) {
    //  c = VCOM_getchar();
    //  if (c != EOF) {
    //    // Echo character back with INCREMENT_ECHO_BY offset, so for example if
    //    // INCREMENT_ECHO_BY is 1 and 'A' is received, 'B' will be echoed back.
    //    VCOM_putchar(c + INCREMENT_ECHO_BY);
    //  }
    vTaskDelayUntil(&xLastFlashTime, 1000 / portTICK_RATE_MS);
  }
}

