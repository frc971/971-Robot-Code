// ****************************************************************************
// CopyLeft qwerk Robotics unINC. 2010 All Rights Reserved.
// ****************************************************************************

// ****************************************************************************
// **************** IO Pin Setup
// ****************************************************************************

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "analog.h"

// How long (in ms) to wait after a falling edge on the bottom indexer sensor
// before reading the indexer encoder.
static const int kBottomFallDelayTime = 32;

void analog_init(void) {
  // b[1:0] CAN RD1 p0.0
  // b[3:2] CAN TD1 p0.1
  //PINCON->PINSEL0 = 0x00000005;

  // b[29:28] USB_DMIN   p0.30
  // b[27:26] USB_DPLUS  p0.29
  // b[21:20] AD0.3  p0.26
  // b[19:18] AD0.2  p0.25
  // PINCON->PINSEL1 = 0x14140000;

  // PINCON->PINSEL2 = 0x0;

  // b[31:30] AD0.5  p1.31
  // b[29:28] V_BUS  p1.30
  // b[21:20] MCOB1  p1.26
  // b[19:18] MCOA1  p1.25
  // b[15:14] MCI1  p1.23
  // b[13:12] MCOB0  p1.22
  // b[09:08] MCI0  p1.20
  // b[07:06] MCOA0  p1.19
  // b[05:04] USB_UP_LED  p1.18
  //PINCON->PINSEL3 = 0xE0145150;
  SC->PCONP |= PCONP_PCAD;

  // Enable AD0.0, AD0.1, AD0.2, AD0.3
  PINCON->PINSEL1 &= 0xFFC03FFF;
  PINCON->PINSEL1 |= 0x00D54000;
  ADC->ADCR = 0x00200500;
}

// ****************************************************************************
// **************** ADC Functions
// ****************************************************************************


// **************** macros
// starts conversion [26:24] = 001

// **************** functions
int analog(int channel) {
  ADC->ADCR = ((ADC->ADCR & 0xF8FFFF00) | (0x01000000 | (1 << channel)));

  // Poll until it is done.
  while(!(ADC->ADGDR & 0x80000000));

  return ((ADC->ADGDR & 0x0000FFF0) >> 4);
}
// GPIO1 P0.4
// GPIO2 P0.5
// GPIO3 P0.6
// GPIO4 P0.7
// GPIO5 P0.8
// GPIO6 P0.9
// GPIO7 P2.0
// GPIO8 P2.1
// GPIO9 P2.2
// GPIO10 P2.3
// GPIO11 P2.4
// GPIO12 P2.5

// DIP0 P1.29
// DIP1 P2.13
// DIP2 P0.11
// DIP3 P0.10
#define readGPIO(gpio, chan) ((((gpio)->FIOPIN) >> (chan)) & 1)
inline int readGPIO_inline(int major, int minor) {
  switch (major) {
    case 0:
      return readGPIO(GPIO0, minor);
    case 1:
      return readGPIO(GPIO1, minor);
    case 2:
      return readGPIO(GPIO2, minor);
    default:
      return -1;
  }
}
int digital(int channel) {
  if (channel < 1) {
    return -1;
  } else if (channel < 7) {
    int chan = channel + 3;
    return readGPIO(GPIO0, chan);
  } else if (channel < 13) {
    int chan = channel - 7;
    return readGPIO(GPIO2, chan);
  }
  return -1;
}
int dip(int channel) {
  switch (channel) {
    case 0:
      return readGPIO(GPIO1, 29);
    case 1:
      return readGPIO(GPIO2, 13);
    case 2:
      return readGPIO(GPIO0, 11);
    case 3:
      return readGPIO(GPIO0, 10);
    default:
      return -1;
  }
}
// ENC0A 1.20
// ENC0B 1.23
// ENC1A 2.11
// ENC1B 2.12
// ENC2A 0.21
// ENC2B 0.22
// ENC3A 0.19
// ENC3B 0.20

#define ENC(gpio, a, b) readGPIO(gpio, a) * 2 + readGPIO(gpio, b)
int encoder_bits(int channel) {
  switch (channel) {
    case 0:
      return ENC(GPIO1, 20, 23);
    case 1:
      return ENC(GPIO2, 11, 12);
    case 2:  
      return ENC(GPIO0, 21, 22);
    case 3:  
      return ENC(GPIO0, 19, 20);
    default:
      return -1;
  }
  return -1;
}
#undef ENC

// Uses EINT1 and EINT2 on 2.11 and 2.12.
volatile int32_t encoder1_val;
// On GPIO pins 0.22 and 0.21.
volatile int32_t encoder2_val;
// On GPIO pins 0.20 and 0.19.
volatile int32_t encoder3_val;
// On GPIO pins 2.0 and 2.1.
volatile int32_t encoder4_val;
// On GPIO pins 2.2 and 2.3.
volatile int32_t encoder5_val;

// ENC1A 2.11
void EINT1_IRQHandler(void) {
  // TODO(brians): figure out why this has to be up here too
  SC->EXTINT = 0x2;
  int fiopin = GPIO2->FIOPIN;
  if (((fiopin >> 1) ^ fiopin) & 0x800) {
    ++encoder1_val;
  } else {
    --encoder1_val;
  }
  SC->EXTPOLAR ^= 0x2;
  SC->EXTINT = 0x2;
}
// ENC1B 2.12
void EINT2_IRQHandler(void) {
  SC->EXTINT = 0x4;
  int fiopin = GPIO2->FIOPIN;
  if (((fiopin >> 1) ^ fiopin) & 0x800) {
    --encoder1_val;
  } else {
    ++encoder1_val;
  }
  SC->EXTPOLAR ^= 0x4;
  SC->EXTINT = 0x4;
}

// GPIO Interrupt handlers
static void NoGPIO() {}
static void Encoder2ARise() {
  GPIOINT->IO0IntClr |= (1 << 22);
  if (GPIO0->FIOPIN & (1 << 21)) {
    ++encoder2_val;
  } else {
    --encoder2_val;
  }
}
static void Encoder2AFall() {
  GPIOINT->IO0IntClr |= (1 << 22);
  if (GPIO0->FIOPIN & (1 << 21)) {
    --encoder2_val;
  } else {
    ++encoder2_val;
  }
}
static void Encoder2BRise() {
  GPIOINT->IO0IntClr |= (1 << 21);
  if (GPIO0->FIOPIN & (1 << 22)) {
    --encoder2_val;
  } else {
    ++encoder2_val;
  }
}
static void Encoder2BFall() {
  GPIOINT->IO0IntClr |= (1 << 21);
  if (GPIO0->FIOPIN & (1 << 22)) {
    ++encoder2_val;
  } else {
    --encoder2_val;
  }
}

static void Encoder3ARise() {
  GPIOINT->IO0IntClr |= (1 << 20);
  if (GPIO0->FIOPIN & (1 << 19)) {
    ++encoder3_val;
  } else {
    --encoder3_val;
  }
}
static void Encoder3AFall() {
  GPIOINT->IO0IntClr |= (1 << 20);
  if (GPIO0->FIOPIN & (1 << 19)) {
    --encoder3_val;
  } else {
    ++encoder3_val;
  }
}
static void Encoder3BRise() {
  GPIOINT->IO0IntClr |= (1 << 19);
  if (GPIO0->FIOPIN & (1 << 20)) {
    --encoder3_val;
  } else {
    ++encoder3_val;
  }
}
static void Encoder3BFall() {
  GPIOINT->IO0IntClr |= (1 << 19);
  if (GPIO0->FIOPIN & (1 << 20)) {
    ++encoder3_val;
  } else {
    --encoder3_val;
  }
}

static void Encoder4ARise() {
  GPIOINT->IO2IntClr |= (1 << 0);
  if (GPIO2->FIOPIN & (1 << 1)) {
    ++encoder4_val;
  } else {
    --encoder4_val;
  }
}
static void Encoder4AFall() {
  GPIOINT->IO2IntClr |= (1 << 0);
  if (GPIO2->FIOPIN & (1 << 1)) {
    --encoder4_val;
  } else {
    ++encoder4_val;
  }
}
static void Encoder4BRise() {
  GPIOINT->IO2IntClr |= (1 << 1);
  if (GPIO2->FIOPIN & (1 << 0)) {
    --encoder4_val;
  } else {
    ++encoder4_val;
  }
}
static void Encoder4BFall() {
  GPIOINT->IO2IntClr |= (1 << 1);
  if (GPIO2->FIOPIN & (1 << 0)) {
    ++encoder4_val;
  } else {
    --encoder4_val;
  }
}

static void Encoder5ARise() {
  GPIOINT->IO2IntClr |= (1 << 2);
  if (GPIO2->FIOPIN & (1 << 3)) {
    ++encoder5_val;
  } else {
    --encoder5_val;
  }
}
static void Encoder5AFall() {
  GPIOINT->IO2IntClr |= (1 << 2);
  if (GPIO2->FIOPIN & (1 << 3)) {
    --encoder5_val;
  } else {
    ++encoder5_val;
  }
}
static void Encoder5BRise() {
  GPIOINT->IO2IntClr |= (1 << 3);
  if (GPIO2->FIOPIN & (1 << 2)) {
    --encoder5_val;
  } else {
    ++encoder5_val;
  }
}
static void Encoder5BFall() {
  GPIOINT->IO2IntClr |= (1 << 3);
  if (GPIO2->FIOPIN & (1 << 2)) {
    ++encoder5_val;
  } else {
    --encoder5_val;
  }
}

volatile int32_t capture_top_rise;
volatile int8_t top_rise_count;
static void IndexerTopRise() {
  GPIOINT->IO0IntClr |= (1 << 5);
  // edge counting   encoder capture
  ++top_rise_count;
  capture_top_rise = encoder3_val;
}
volatile int32_t capture_top_fall;
volatile int8_t top_fall_count;
static void IndexerTopFall() {
  GPIOINT->IO0IntClr |= (1 << 5);
  // edge counting   encoder capture
  ++top_fall_count;
  capture_top_fall = encoder3_val;
}
volatile int8_t bottom_rise_count;
static void IndexerBottomRise() {
  GPIOINT->IO0IntClr |= (1 << 4);
  // edge counting
  ++bottom_rise_count;
}
volatile int32_t capture_bottom_fall_delay;
volatile int8_t bottom_fall_delay_count;
volatile int32_t dirty_delay;
portTickType xDelayTimeFrom;
static portTASK_FUNCTION(vDelayCapture, pvParameters)
{
  portTickType xSleepFrom = xTaskGetTickCount();

  for (;;) {
    NVIC_DisableIRQ(EINT3_IRQn);
    if (dirty_delay != 0) {
      xSleepFrom = xDelayTimeFrom;
      dirty_delay = 0;
      NVIC_EnableIRQ(EINT3_IRQn);

      vTaskDelayUntil(&xSleepFrom, kBottomFallDelayTime / portTICK_RATE_MS);

      NVIC_DisableIRQ(EINT3_IRQn);
      ++bottom_fall_delay_count;
      capture_bottom_fall_delay = encoder3_val;
      NVIC_EnableIRQ(EINT3_IRQn);
    } else {
      NVIC_EnableIRQ(EINT3_IRQn);
      vTaskDelayUntil(&xSleepFrom, 10 / portTICK_RATE_MS);
    }
  }
}

volatile int8_t bottom_fall_count;
static void IndexerBottomFall() {
  GPIOINT->IO0IntClr |= (1 << 4);
  ++bottom_fall_count;
  // edge counting   start delayed capture
  xDelayTimeFrom = xTaskGetTickCount();
  dirty_delay = 1;
}
volatile int32_t capture_wrist_rise;
volatile int8_t wrist_rise_count;
static void WristHallRise() {
  GPIOINT->IO0IntClr |= (1 << 6);
  // edge counting   encoder capture
  ++wrist_rise_count;
  capture_wrist_rise = (int32_t)QEI->QEIPOS;
}
volatile int32_t capture_shooter_angle_rise;
volatile int8_t shooter_angle_rise_count;
static void ShooterHallRise() {
  GPIOINT->IO0IntClr |= (1 << 7);
  // edge counting   encoder capture
  ++shooter_angle_rise_count;
  capture_shooter_angle_rise = encoder2_val; 
}

// Count leading zeros.
// Returns 0 if bit 31 is set etc.
__attribute__((always_inline)) static __INLINE uint32_t __clz(uint32_t value) {
  uint32_t result;
  __asm__("clz %0, %1" : "=r" (result) : "r" (value));
  return result;
}
inline static void IRQ_Dispatch(void) {
  // TODO(brians): think about adding a loop here so that we can handle multiple
  // interrupts right on top of each other faster
  uint32_t index = __clz(GPIOINT->IO2IntStatR | GPIOINT->IO0IntStatR |
      (GPIOINT->IO2IntStatF << 28) | (GPIOINT->IO0IntStatF << 4));

  typedef void (*Handler)(void);
  const static Handler table[] = {
    Encoder5BFall,     // index 0: P2.3 Fall     #bit 31  //Encoder 5 B  //Dio 10
    Encoder5AFall,     // index 1: P2.2 Fall     #bit 30  //Encoder 5 A  //Dio 9
    Encoder4BFall,     // index 2: P2.1 Fall     #bit 29  //Encoder 4 B  //Dio 8
    Encoder4AFall,     // index 3: P2.0 Fall     #bit 28  //Encoder 4 A  //Dio 7
    NoGPIO,            // index 4: NO GPIO       #bit 27
    Encoder2AFall,     // index 5: P0.22 Fall    #bit 26  //Encoder 2 A
    Encoder2BFall,     // index 6: P0.21 Fall    #bit 25  //Encoder 2 B
    Encoder3AFall,     // index 7: P0.20 Fall    #bit 24  //Encoder 3 A
    Encoder3BFall,     // index 8: P0.19 Fall    #bit 23  //Encoder 3 B
    Encoder2ARise,     // index 9: P0.22 Rise    #bit 22  //Encoder 2 A
    Encoder2BRise,     // index 10: P0.21 Rise   #bit 21  //Encoder 2 B
    Encoder3ARise,     // index 11: P0.20 Rise   #bit 20  //Encoder 3 A
    Encoder3BRise,     // index 12: P0.19 Rise   #bit 19  //Encoder 3 B
    NoGPIO,            // index 13: NO GPIO      #bit 18
    NoGPIO,            // index 14: NO GPIO      #bit 17
    NoGPIO,            // index 15: NO GPIO      #bit 16
    NoGPIO,            // index 16: NO GPIO      #bit 15
    NoGPIO,            // index 17: NO GPIO      #bit 14
    NoGPIO,            // index 18: NO GPIO      #bit 13
    NoGPIO,            // index 19: NO GPIO      #bit 12
    ShooterHallRise,   // index 20: P0.7 Fall    #bit 11  //Shooter Hall   //Dio 4
    WristHallRise,     // index 21: P0.6 Fall    #bit 10  //Wrist Hall     //Dio 3
    IndexerTopRise,    // index 22: P0.5 Fall    #bit 9   //Indexer Top    //Dio 2
    IndexerBottomRise, // index 23: P0.4 Fall    #bit 8   //Indexer Bottom //Dio 1
    NoGPIO,            // index 24: NO GPIO      #bit 7
    NoGPIO,            // index 25: NO GPIO      #bit 6
    IndexerTopFall,    // index 26: P0.5 Rise    #bit 5   //Indexer Top    //Dio 2
    IndexerBottomFall, // index 27: P0.4 Rise    #bit 4   //Indexer Bottom //Dio 1
    Encoder5BRise,     // index 28: P2.3 Rise    #bit 3   //Encoder 5 B    //Dio 10
    Encoder5ARise,     // index 29: P2.2 Rise    #bit 2   //Encoder 5 A    //Dio 9
    Encoder4BRise,     // index 30: P2.1 Rise    #bit 1   //Encoder 4 B    //Dio 8
    Encoder4ARise,     // index 31: P2.0 Rise    #bit 0   //Encoder 4 A    //Dio 7
    NoGPIO             // index 32: NO BITS SET  #False Alarm
  };
  table[index]();
}
void EINT3_IRQHandler(void) {
  // Have to disable it here or else it re-fires the interrupt while the code
  // reads to figure out which pin the interrupt is for.
  // TODO(brians): figure out details + look for an alternative
  NVIC_DisableIRQ(EINT3_IRQn);
  IRQ_Dispatch();
  NVIC_EnableIRQ(EINT3_IRQn);
}
int32_t encoder_val(int chan) {
  int32_t val;
  switch (chan) {
    case 0: // Wrist
      return (int32_t)QEI->QEIPOS;
    case 1: // Shooter Wheel
      NVIC_DisableIRQ(EINT1_IRQn);
      NVIC_DisableIRQ(EINT2_IRQn);
      val = encoder1_val;
      NVIC_EnableIRQ(EINT2_IRQn);
      NVIC_EnableIRQ(EINT1_IRQn);
      return val;
    case 2: // Shooter Angle
      NVIC_DisableIRQ(EINT3_IRQn);
      val = encoder2_val;
      NVIC_EnableIRQ(EINT3_IRQn);
      return val;
    case 3: // Indexer
      NVIC_DisableIRQ(EINT3_IRQn);
      val = encoder3_val;
      NVIC_EnableIRQ(EINT3_IRQn);
      return val;
    case 4: // Drive R
      NVIC_DisableIRQ(EINT3_IRQn);
      val = encoder4_val;
      NVIC_EnableIRQ(EINT3_IRQn);
      return val;
    case 5: // Drive L
      NVIC_DisableIRQ(EINT3_IRQn);
      val = encoder5_val;
      NVIC_EnableIRQ(EINT3_IRQn);
      return val;
    default:
      return -1;
  }
}
void fillSensorPacket(struct DataStruct *packet) {
  packet->gyro_angle = gyro_angle;

  packet->shooter = encoder1_val;
  packet->left_drive = encoder4_val;
  packet->right_drive = encoder5_val;
  packet->shooter_angle = encoder2_val;
  packet->indexer = encoder3_val;

  NVIC_DisableIRQ(EINT1_IRQn);
  NVIC_DisableIRQ(EINT2_IRQn);

  packet->wrist = (int32_t)QEI->QEIPOS;
  packet->wrist_hall_effect = !digital(3);
  packet->capture_wrist_rise = capture_wrist_rise;
  packet->wrist_rise_count = wrist_rise_count;

  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);

  NVIC_DisableIRQ(EINT3_IRQn);

  packet->capture_top_rise = capture_top_rise;
  packet->top_rise_count = top_rise_count;
  packet->capture_top_fall = capture_top_fall;
  packet->top_fall_count = top_fall_count;
  packet->top_disc = !digital(2);

  packet->capture_bottom_fall_delay = capture_bottom_fall_delay;
  packet->bottom_fall_delay_count = bottom_fall_delay_count;
  packet->bottom_fall_count = bottom_fall_count;
  packet->bottom_disc = !digital(1);

  packet->loader_top = !digital(5);
  packet->loader_bottom = !digital(6);

  packet->capture_shooter_angle_rise = capture_shooter_angle_rise;
  packet->shooter_angle_rise_count = shooter_angle_rise_count;
  packet->angle_adjust_bottom_hall_effect = !digital(4);

  NVIC_EnableIRQ(EINT3_IRQn);

  packet->bottom_rise_count = bottom_rise_count;
}

void encoder_init(void) {
  // Setup the encoder interface.
  SC->PCONP |= PCONP_PCQEI;
  PINCON->PINSEL3 = ((PINCON->PINSEL3 & 0xffff3dff) | 0x00004100);
  // Reset the count and velocity.
  QEI->QEICON = 0x00000005;
  QEI->QEICONF = 0x00000004;
  // Wrap back to 0 when we wrap the int and vice versa.
  QEI->QEIMAXPOS = 0xFFFFFFFF;

  // Set up encoder 1.
  // Make GPIOs 2.11 and 2.12 trigger EINT1 and EINT2 (respectively).
  // PINSEL4[23:22] = {0 1}
  // PINSEL4[25:24] = {0 1}
  PINCON->PINSEL4 = (PINCON->PINSEL4 & ~(0x3 << 22)) | (0x1 << 22);
  PINCON->PINSEL4 = (PINCON->PINSEL4 & ~(0x3 << 24)) | (0x1 << 24);
  // Clear the interrupt flags for EINT1 and EINT2 (0x6 = 0b0110).
  SC->EXTMODE = 0x6;
  SC->EXTINT = 0x6;
  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);
  encoder1_val = 0;

  // Set up encoder 2.
  GPIOINT->IO0IntEnF |= (1 << 22);  // Set GPIO falling interrupt.
  GPIOINT->IO0IntEnR |= (1 << 22);  // Set GPIO rising interrupt.
  GPIOINT->IO0IntEnF |= (1 << 21);  // Set GPIO falling interrupt.
  GPIOINT->IO0IntEnR |= (1 << 21);  // Set GPIO rising interrupt.
  // Make sure they're in mode 00 (the default, aka nothing special).
  PINCON->PINSEL1 &= ~(0x3 << 12);
  PINCON->PINSEL1 &= ~(0x3 << 10);
  encoder2_val = 0;

  // Set up encoder 3.
  GPIOINT->IO0IntEnF |= (1 << 20);  // Set GPIO falling interrupt.
  GPIOINT->IO0IntEnR |= (1 << 20);  // Set GPIO rising interrupt.
  GPIOINT->IO0IntEnF |= (1 << 19);  // Set GPIO falling interrupt.
  GPIOINT->IO0IntEnR |= (1 << 19);  // Set GPIO rising interrupt.
  // Make sure they're in mode 00 (the default, aka nothing special).
  PINCON->PINSEL1 &= ~(0x3 << 8);
  PINCON->PINSEL1 &= ~(0x3 << 6);
  encoder3_val = 0;

  // Set up encoder 4.
  GPIOINT->IO2IntEnF |= (1 << 0);  // Set GPIO falling interrupt.
  GPIOINT->IO2IntEnR |= (1 << 0);  // Set GPIO rising interrupt.
  GPIOINT->IO2IntEnF |= (1 << 1);  // Set GPIO falling interrupt.
  GPIOINT->IO2IntEnR |= (1 << 1);  // Set GPIO rising interrupt.
  // Make sure they're in mode 00 (the default, aka nothing special).
  PINCON->PINSEL4 &= ~(0x3 << 0);
  PINCON->PINSEL4 &= ~(0x3 << 2);
  encoder4_val = 0;

  // Set up encoder 5.
  GPIOINT->IO2IntEnF |= (1 << 2);  // Set GPIO falling interrupt.
  GPIOINT->IO2IntEnR |= (1 << 2);  // Set GPIO rising interrupt.
  GPIOINT->IO2IntEnF |= (1 << 3);  // Set GPIO falling interrupt.
  GPIOINT->IO2IntEnR |= (1 << 3);  // Set GPIO rising interrupt.
  // Make sure they're in mode 00 (the default, aka nothing special).
  PINCON->PINSEL4 &= ~(0x3 << 4);
  PINCON->PINSEL4 &= ~(0x3 << 6);
  encoder5_val = 0;

  // Enable interrupts from the GPIO pins.
  NVIC_EnableIRQ(EINT3_IRQn);

  xTaskCreate(vDelayCapture,
              (signed char *) "SENSORs",
              configMINIMAL_STACK_SIZE + 100,
              NULL /*parameters*/,
              tskIDLE_PRIORITY + 5,
              NULL /*return task handle*/);

  GPIOINT->IO0IntEnF |= (1 << 4);  // Set GPIO falling interrupt
  GPIOINT->IO0IntEnR |= (1 << 4);  // Set GPIO rising interrupt
  PINCON->PINSEL0 &= ~(0x3 << 8);

  GPIOINT->IO0IntEnF |= (1 << 5);  // Set GPIO falling interrupt
  GPIOINT->IO0IntEnR |= (1 << 5);  // Set GPIO rising interrupt
  PINCON->PINSEL0 &= ~(0x3 << 10);

  GPIOINT->IO0IntEnF |= (1 << 6);
  PINCON->PINSEL0 &= ~(0x3 << 12);

  GPIOINT->IO0IntEnF |= (1 << 7);
  PINCON->PINSEL0 &= ~(0x3 << 14);
}
