/* Standard includes. */
#include "stdio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo app includes. */
#include "flash.h"
#include "partest.h"
#include "analog.h"
#include "spi.h"
#include "LPCUSB/usbapi.h"
#include "CAN.h"

static xQueueHandle CAN_tx_queue = NULL, CAN_rx_queue = NULL;

/** Send a CAN message.  The message is stored by copy.
 *
 * \returns Zero on success, and one on failure to queue the message.
 */
int CAN_send(can_message *message) {
  uint32_t sr = CAN1->SR;
  // Queue the message up in a queue!
  if (sr & 0x00040404) {
    uint32_t IFx = (message->RTR << 30) +
                   (message->length << 16) +
                   (message->priority);

    uint32_t data0 = message->wdata[0];
    uint32_t data1 = message->wdata[1];

    if(sr & 0x00000004){
      CAN1->TFI1 = IFx;
      CAN1->TID1 = message->id;
      CAN1->TDA1 = data0; 
      CAN1->TDB1 = data1;
      CAN1->CMR = 0x00000021;
      printf("Writing 1\n");
      return 0;
    } else if(sr & 0x00000400) {
      CAN1->TFI2 = IFx;
      CAN1->TID2 = message->id;
      CAN1->TDA2 = data0; 
      CAN1->TDB2 = data1;
      CAN1->CMR = 0x00000041;
      return 0;
    } else if(sr & 0x00040000) {
      CAN1->TFI3 = IFx;
      CAN1->TID3 = message->id;
      CAN1->TDA3 = data0; 
      CAN1->TDB3 = data1;
      CAN1->CMR = 0x00000081;
      return 0;
    }
    return 1;
  }
  return xQueueSend(CAN_tx_queue, message, 0) != pdPASS;
}

/**
 * Get a CAN message, blocking if one isn't available.
 * This is useful for a read/dispatch thread.
 *
 * \returns 0 on Success, and nonzero on failure
 */
int CAN_get(can_message *message) {
  // Blocking read the queue.
  if (xQueueReceive(CAN_rx_queue, message, portMAX_DELAY) == pdFALSE) {
    return 1;
  } else {
    return message->error;
  }
}

void CAN_IRQHandler(void) {
  uint32_t interrupts = CAN1->ICR;
  long lHigherPriorityTaskWoken = pdFALSE;
  can_message message;
  // CAN1->CMR contains all the action items, like initiating a send et all.
  // On a bus error interrupt, set an error semaphore so the handler can set the LED, or set it ourselfs.
  printf("CAN interrupt\n");

  if (interrupts & 0x1) {
    // Recieved a packet.
    uint32_t rfs = CAN1->RFS;

    message.RTR = (rfs & 0x40000000) >> 30;
    message.length = (rfs & 0x000f0000) >> 16;
    message.id = CAN1->RID & 0x000007ff;

    message.wdata[0] = CAN1->RDA;
    message.wdata[1] = CAN1->RDB;

    // Release the registers
    CAN1->CMR = 0x00000004;

    message.error = 0;

    xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }

  if (interrupts & 0x602) {
    // TX buffers are no longer empty (now empty??)
    if (xQueueReceiveFromISR(CAN_tx_queue, &message, &lHigherPriorityTaskWoken)) {
      uint32_t IFx = (message.RTR << 30) +
                     (message.length << 16) +
                     (message.priority);
      uint32_t data0 = message.wdata[0];
      uint32_t data1 = message.wdata[1];

      // Message was in the queue, and will now be sent.
      if (interrupts & 0x2) {
        // TX buffer 1 is no longer empty
        CAN1->TFI1 = IFx;
        CAN1->TID1 = message.id;
        CAN1->TDA1 = data0; 
        CAN1->TDB1 = data1;
        CAN1->CMR = 0x00000021;
      } else if (interrupts & 0x200) {
        // TX buffer 2 is no longer empty
        CAN1->TFI2 = IFx;
        CAN1->TID2 = message.id;
        CAN1->TDA2 = data0; 
        CAN1->TDB2 = data1;
        CAN1->CMR = 0x00000041;
      } else if (interrupts & 0x400) {
        // TX buffer 3 is no longer empty
        CAN1->TFI3 = IFx;
        CAN1->TID3 = message.id;
        CAN1->TDA3 = data0; 
        CAN1->TDB3 = data1;
        CAN1->CMR = 0x00000081;
      }
    }
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }

  if (interrupts & 0x4) {
    // Error Warning interrupt
    message.error = 0x01;
    if (CAN1->GSR & 0x00000040) {
      // Reached the limit.
      message.error = 0x01;
    } else if (CAN1->GSR & 0x00000080) {
      // No longer allowed to send.
      message.error = 0x20;
    } else {
      message.error = 0x20;
    }
    xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }

  if (interrupts & 0x8) {
    // Data overrun
    CAN1->CMR = 0x00000008;
    message.error = 0x02;
    xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }

  if (interrupts & 0x20) {
    // Error Passive Interrupt
    message.error = 0x04;
    xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }

  if (interrupts & 0x80) {
    // Bus Error Interrupt
    message.error = 0x08;
    xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    return;
  }
  message.error = 0x10;
  xQueueSendFromISR(CAN_rx_queue, &message, &lHigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}

extern int VCOM_getchar(void);

static portTASK_FUNCTION(vCAN1Write, pvParameters) {
  can_message message;
  message.RTR = 0;
  message.id = 12;
  message.priority = 1;
  message.length = 4;

  // Enable the pins.
  PINCON->PINSEL3 = (PINCON->PINSEL3 & 0xffc3cf3f) | 0x00141040;

  portTickType xLastFlashTime;
  xLastFlashTime = xTaskGetTickCount();

  for (;;) {
    printf("hello\n");

    int c = VCOM_getchar();
    while (c != -1) {
      printf("hello\n");
      int j = c;
      printf("Sending data 0x%x\n", j);
      message.data[0] = j++;
      message.data[1] = j++;
      message.data[2] = j++;
      message.data[3] = j;
      CAN_send(&message);
      c = VCOM_getchar();
    }

    vTaskDelayUntil(&xLastFlashTime, 500);
  }
}
static portTASK_FUNCTION(vCAN1, pvParameters) {
  portTickType xLastFlashTime;

  CAN_rx_queue = xQueueCreate(20, sizeof(can_message));
  CAN_tx_queue = xQueueCreate(5, sizeof(char));

  if ((CAN_rx_queue == NULL) || (CAN_tx_queue == NULL)) {
    /* Not enough heap available to create the buffer queues, can't do
     * anything so just delete ourselves.
     */
    vTaskDelete(NULL);
  }

  // Enable CAN
  SC->PCONP |= PCONP_PCCAN1;

  PINCON->PINSEL0 = (PINCON->PINSEL0 & 0xfffffff0) | 0x00000005;
  PINCON->PINMODE0 = (PINCON->PINMODE0 & 0xfffffffc) | 0x00000001;

  // Enable RX, TX, overrun, Bus Error,
  // Error Passive/Active, and Error Warning Interupts.
  NVIC_SetPriority(CAN_IRQn, configCAN_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(CAN_IRQn);

  CAN1->IER = 0x000006af;

  // Priority depends on the TX priority register
  CAN1->MOD = 0x00000008;

  CAN1->CMR = 0x00000004;

  CANAF->AFMR = 0x00000002;

  // CAN clocks. (Defaults look fine)

  /* We need to initialise xLastFlashTime prior to the first call to
  vTaskDelayUntil(). */
  xLastFlashTime = xTaskGetTickCount();

  can_message message;

  xTaskCreate(vCAN1Write, (signed char *) "CAN1wx", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 4, NULL);

  for (;;) {
    /* Delay for half the flash period then turn the LED on. */
    if (CAN_get(&message)) {
      printf("Message error 0x%x\n", message.error);
    } else {
      printf("Got a message with a length of %d\n", message.length);
      printf("data[0] = 0x%x\n", message.data[0]);
      printf("data[1] = 0x%x\n", message.data[1]);
      printf("data[2] = 0x%x\n", message.data[2]);
      printf("data[3] = 0x%x\n", message.data[3]);
    }
  }
}


void initCAN(void){
  xTaskCreate(vCAN1, (signed char *) "CAN1rx", configMINIMAL_STACK_SIZE + 400, NULL, tskIDLE_PRIORITY + 1, NULL);
}
