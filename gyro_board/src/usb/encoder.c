#include <string.h>

#include "fill_packet.h"
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

#include "digital.h"
#include "analog.h"
#include "gyro.h"

// How long (in ms) to wait after a falling edge on the bottom indexer sensor
// before reading the indexer encoder.
static const int kBottomFallDelayTime = 32;
// How long to wait for a revolution of the shooter wheel (on the third robot)
// before said wheel is deemed "stopped". (In secs)
static const uint8_t kWheelStopThreshold = 1;

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

// It is important to clear the various interrupt flags first thing in the ISRs.
// It doesn't seem to work otherwise, possibly because of the reason that Brian
// found poking around online: caches on the bus make it so that the clearing of
// the interrupt gets to the NVIC after the ISR returns, so it runs the ISR a
// second time. Also, by clearing them early, if a second interrupt arrives from
// the same source it will still get handled instead of getting lost.

// ENC1A 2.11
void EINT1_IRQHandler(void) {
  // Make sure to change this BEFORE clearing the interrupt like the datasheet
  // says you have to.
  SC->EXTPOLAR ^= 0x2;
  SC->EXTINT = 0x2;
  int fiopin = GPIO2->FIOPIN;
  // This looks like a weird way to XOR the 2 inputs, but it compiles down to
  // just 2 instructions, which is hard to beat.
  if (((fiopin >> 1) ^ fiopin) & 0x800) {
    ++encoder1_val;
  } else {
    --encoder1_val;
  }
}
// ENC1B 2.12
void EINT2_IRQHandler(void) {
  SC->EXTPOLAR ^= 0x4;
  SC->EXTINT = 0x4;
  int fiopin = GPIO2->FIOPIN;
  if (((fiopin >> 1) ^ fiopin) & 0x800) {
    --encoder1_val;
  } else {
    ++encoder1_val;
  }
}

static inline void reset_TC(void) {
  TIM2->TCR |= (1 << 1); // Put it into reset.
  while (TIM2->TC != 0) { // Wait for reset.
    continue;
  }
  TIM2->TCR = 1; // Take it out of reset + make sure it's enabled.
}

// TIM2
volatile uint32_t shooter_cycle_ticks;
void TIMER2_IRQHandler(void) {
  // Apparently, this handler runs regardless of a match or capture event.
  if (TIM2->IR & (1 << 4)) {
    // Capture
    TIM2->IR = (1 << 3); // Clear the interrupt.
    
    shooter_cycle_ticks = TIM2->CR0;
  
    reset_TC();
  } else if (TIM2->IR & 1) {
    // Match
    TIM2->IR = 1; // Clear the interrupt

    // Assume shooter is stopped.
    shooter_cycle_ticks = 0;

    // Disable timer.
    TIM2->TCR = 0;
  }

  // It will only handle one interrupt per run.
  // If there is another interrupt pending, it won't be cleared, and the ISR 
  // will be run again to handle it.
}

// TODO(brians): Have this indicate some kind of error instead of just looping
// infinitely in the ISR because it never clears it.
static void NoGPIO(void) {}
static void Encoder2ARise(void) {
  GPIOINT->IO0IntClr = (1 << 22);
  if (GPIO0->FIOPIN & (1 << 21)) {
    ++encoder2_val;
  } else {
    --encoder2_val;
  }
}
static void Encoder2AFall(void) {
  GPIOINT->IO0IntClr = (1 << 22);
  if (GPIO0->FIOPIN & (1 << 21)) {
    --encoder2_val;
  } else {
    ++encoder2_val;
  }
}
static void Encoder2BRise(void) {
  GPIOINT->IO0IntClr = (1 << 21);
  if (GPIO0->FIOPIN & (1 << 22)) {
    --encoder2_val;
  } else {
    ++encoder2_val;
  }
}
static void Encoder2BFall(void) {
  GPIOINT->IO0IntClr = (1 << 21);
  if (GPIO0->FIOPIN & (1 << 22)) {
    ++encoder2_val;
  } else {
    --encoder2_val;
  }
}

static void Encoder3ARise(void) {
  GPIOINT->IO0IntClr = (1 << 20);
  if (GPIO0->FIOPIN & (1 << 19)) {
    ++encoder3_val;
  } else {
    --encoder3_val;
  }
}
static void Encoder3AFall(void) {
  GPIOINT->IO0IntClr = (1 << 20);
  if (GPIO0->FIOPIN & (1 << 19)) {
    --encoder3_val;
  } else {
    ++encoder3_val;
  }
}
static void Encoder3BRise(void) {
  GPIOINT->IO0IntClr = (1 << 19);
  if (GPIO0->FIOPIN & (1 << 20)) {
    --encoder3_val;
  } else {
    ++encoder3_val;
  }
}
static void Encoder3BFall(void) {
  GPIOINT->IO0IntClr = (1 << 19);
  if (GPIO0->FIOPIN & (1 << 20)) {
    ++encoder3_val;
  } else {
    --encoder3_val;
  }
}

static void Encoder4ARise(void) {
  GPIOINT->IO2IntClr = (1 << 0);
  if (GPIO2->FIOPIN & (1 << 1)) {
    ++encoder4_val;
  } else {
    --encoder4_val;
  }
}
static void Encoder4AFall(void) {
  GPIOINT->IO2IntClr = (1 << 0);
  if (GPIO2->FIOPIN & (1 << 1)) {
    --encoder4_val;
  } else {
    ++encoder4_val;
  }
}
static void Encoder4BRise(void) {
  GPIOINT->IO2IntClr = (1 << 1);
  if (GPIO2->FIOPIN & (1 << 0)) {
    --encoder4_val;
  } else {
    ++encoder4_val;
  }
}
static void Encoder4BFall(void) {
  GPIOINT->IO2IntClr = (1 << 1);
  if (GPIO2->FIOPIN & (1 << 0)) {
    ++encoder4_val;
  } else {
    --encoder4_val;
  }
}

static void Encoder5ARise(void) {
  GPIOINT->IO2IntClr = (1 << 2);
  if (GPIO2->FIOPIN & (1 << 3)) {
    ++encoder5_val;
  } else {
    --encoder5_val;
  }
}
static void Encoder5AFall(void) {
  GPIOINT->IO2IntClr = (1 << 2);
  if (GPIO2->FIOPIN & (1 << 3)) {
    --encoder5_val;
  } else {
    ++encoder5_val;
  }
}
static void Encoder5BRise(void) {
  GPIOINT->IO2IntClr = (1 << 3);
  if (GPIO2->FIOPIN & (1 << 2)) {
    --encoder5_val;
  } else {
    ++encoder5_val;
  }
}
static void Encoder5BFall(void) {
  GPIOINT->IO2IntClr = (1 << 3);
  if (GPIO2->FIOPIN & (1 << 2)) {
    ++encoder5_val;
  } else {
    --encoder5_val;
  }
}

volatile int32_t capture_top_rise;
volatile int8_t top_rise_count;
static void IndexerTopRise(void) {
  GPIOINT->IO0IntClr = (1 << 5);
  // edge counting   encoder capture
  ++top_rise_count;
  capture_top_rise = encoder3_val;
}
volatile int32_t capture_top_fall;
volatile int8_t top_fall_count;
static void IndexerTopFall(void) {
  GPIOINT->IO0IntClr = (1 << 5);
  // edge counting   encoder capture
  ++top_fall_count;
  capture_top_fall = encoder3_val;
}
volatile int8_t bottom_rise_count;
static void IndexerBottomRise(void) {
  GPIOINT->IO0IntClr = (1 << 4);
  // edge counting
  ++bottom_rise_count;
}
volatile int32_t capture_bottom_fall_delay;
volatile int8_t bottom_fall_delay_count;
portTickType xDelayTimeFrom;
static portTASK_FUNCTION(vDelayCapture, pvParameters)
{
  portTickType xSleepFrom = xTaskGetTickCount();

  for (;;) {
    // Atomically (wrt the ISR) switch xDelayTimeFrom to 0 and store its old
    // value to use later.
    NVIC_DisableIRQ(EINT3_IRQn);
    portTickType new_time = xDelayTimeFrom;
    xDelayTimeFrom = 0;
    NVIC_EnableIRQ(EINT3_IRQn);

    if (new_time != 0) {
      xSleepFrom = new_time;

      vTaskDelayUntil(&xSleepFrom, kBottomFallDelayTime / portTICK_RATE_MS);

      // Make sure that the USB ISR doesn't look at inconsistent values.
      NVIC_DisableIRQ(USB_IRQn);
      capture_bottom_fall_delay = encoder3_val;
      ++bottom_fall_delay_count;
      NVIC_EnableIRQ(USB_IRQn);
    } else {
      // Wait 10ms and then check again.
      vTaskDelayUntil(&xSleepFrom, 10 / portTICK_RATE_MS);
    }
  }
}

volatile int8_t bottom_fall_count;
static void IndexerBottomFall(void) {
  GPIOINT->IO0IntClr = (1 << 4);
  ++bottom_fall_count;
  // edge counting   start delayed capture
  xDelayTimeFrom = xTaskGetTickCount();
}
volatile int32_t capture_wrist_rise;
volatile int8_t wrist_rise_count;
static void WristHallRise(void) {
  GPIOINT->IO0IntClr = (1 << 6);
  // edge counting   encoder capture
  ++wrist_rise_count;
  capture_wrist_rise = (int32_t)QEI->QEIPOS;
}
volatile int32_t capture_shooter_angle_rise;
volatile int8_t shooter_angle_rise_count;
static void ShooterHallRise(void) {
  GPIOINT->IO0IntClr = (1 << 7);
  // edge counting   encoder capture
  ++shooter_angle_rise_count;
  capture_shooter_angle_rise = encoder2_val; 
}

// Third robot shooter.
static void ShooterPhotoFall(void) {
  GPIOINT->IO0IntClr = (1 << 23);
  // We reset TC to make sure we don't get a crap
  // value from CR0 when the capture interrupt occurs
  // if the shooter is just starting up again, and so
  // that the match interrupt thing works right.
  reset_TC();
}

typedef void (*Handler)(void);
// Contains default pointers for ISR functions.
// (These can be used without modifications on the comp/practice bots.)
Handler ISRTable[] = {
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

// Count leading zeros.
// Returns 0 if bit 31 is set etc.
__attribute__((always_inline)) static __INLINE uint32_t __clz(uint32_t value) {
  uint32_t result;
  __asm__("clz %0, %1" : "=r" (result) : "r" (value));
  return result;
}
inline static void IRQ_Dispatch(void) {
  // There is no need to add a loop here to handle multiple interrupts at the
  // same time because the processor has tail chaining of interrupts which we
  // can't really beat with our own loop.
  // It would actually be bad because a loop here would block EINT1/2 for longer
  // lengths of time.

  uint32_t index = __clz(GPIOINT->IO2IntStatR | GPIOINT->IO0IntStatR |
      (GPIOINT->IO2IntStatF << 28) | (GPIOINT->IO0IntStatF << 4));

  ISRTable[index]();
}
void EINT3_IRQHandler(void) {
  IRQ_Dispatch();
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

void encoder_init(void) {
  // Setup the encoder interface.
  SC->PCONP |= PCONP_PCQEI;
  PINCON->PINSEL3 = ((PINCON->PINSEL3 & 0xffff3dff) | 0x00004100);
  // Reset the count and velocity.
  QEI->QEICON = 0x00000005;
  QEI->QEICONF = 0x00000004;
  // Wrap back to 0 when we wrap the int and vice versa.
  QEI->QEIMAXPOS = 0xFFFFFFFF;
  
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
  
  // Enable interrupts from the GPIO pins.
  NVIC_EnableIRQ(EINT3_IRQn);

  if (is_bot3) {
    // Modify robot handler table for third robot.
    ISRTable[23] = ShooterPhotoFall;

    // Set up timer for bot3 photosensor.
    // Make sure timer two is powered.
    SC->PCONP |= (1 << 22);
    // We don't need all the precision the CCLK can provide.
    // We'll use CCLK/8. (12.5 mhz).
    SC->PCLKSEL1 |= (0x3 << 12);
    // Use timer prescale to get that freq down to 500 hz.
    TIM2->PR = 25000;
    // Select capture 2.0 function on pin 0.4.
    PINCON->PINSEL0 |= (0x3 << 8);
    // Set timer to capture and interrupt on rising edge.
    TIM2->CCR = 0x5;
    // Set up match interrupt.
    TIM2->MR0 = kWheelStopThreshold * 500;
    TIM2->MCR = 1;
    // Enable timer IRQ, and make it lower priority than the encoders.
    NVIC_SetPriority(TIMER3_IRQn, 1);
    NVIC_EnableIRQ(TIMER3_IRQn);
    // Set up GPIO interrupt on other edge.
    GPIOINT->IO0IntEnF |= (1 << 23);

  } else {  // is main robot
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
}

void fillSensorPacket(struct DataStruct *packet) {
  if (gyro_output.initialized) {
    packet->gyro_angle = gyro_output.angle;
    packet->old_gyro_reading = gyro_output.last_reading_bad;
    packet->bad_gyro = gyro_output.gyro_bad;
  } else {
    packet->gyro_angle = 0;
    packet->old_gyro_reading = 1;
    packet->bad_gyro = 0;
  }

  packet->dip_switch0 = dip_switch(0);
  packet->dip_switch1 = dip_switch(1);
  packet->dip_switch2 = dip_switch(2);
  packet->dip_switch3 = dip_switch(3);

  // We disable EINT3 to avoid sending back inconsistent values. All of the
  // aligned reads from the variables are atomic, so disabling it isn't
  // necessary for just reading encoder values. We re-enable it periodically
  // because disabling and enabling is cheap (2 instructions) and we really rely
  // on low interrupt latencies.

  if (is_bot3) {
    packet->robot_id = 1;

    packet->main.left_drive = encoder3_val;
    packet->main.right_drive = encoder2_val;

    packet->bot3.shooter_cycle_ticks = shooter_cycle_ticks;
  } else {  // is main robot
    packet->robot_id = 0;

    packet->main.left_drive = encoder5_val;
    packet->main.right_drive = encoder4_val;

    packet->main.shooter = encoder1_val;
    packet->main.indexer = encoder3_val;

    NVIC_DisableIRQ(EINT3_IRQn);

    packet->main.wrist = (int32_t)QEI->QEIPOS;
    packet->main.wrist_hall_effect = !digital(3);
    packet->main.capture_wrist_rise = capture_wrist_rise;
    packet->main.wrist_rise_count = wrist_rise_count;

    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);

    packet->main.capture_top_rise = capture_top_rise;
    packet->main.top_rise_count = top_rise_count;
    packet->main.capture_top_fall = capture_top_fall;
    packet->main.top_fall_count = top_fall_count;
    packet->main.top_disc = !digital(2);

    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);

    packet->main.capture_bottom_fall_delay = capture_bottom_fall_delay;
    packet->main.bottom_fall_delay_count = bottom_fall_delay_count;
    packet->main.bottom_fall_count = bottom_fall_count;
    packet->main.bottom_disc = !digital(1);

    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);

    packet->main.loader_top = !digital(5);
    packet->main.loader_bottom = !digital(6);

    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);

    packet->main.shooter_angle = encoder2_val;
    packet->main.capture_shooter_angle_rise = capture_shooter_angle_rise;
    packet->main.shooter_angle_rise_count = shooter_angle_rise_count;
    packet->main.angle_adjust_bottom_hall_effect = !digital(4);

    NVIC_EnableIRQ(EINT3_IRQn);

    packet->main.bottom_rise_count = bottom_rise_count;
  }
}
