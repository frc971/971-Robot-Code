#include "cape/robot.h"

#include <STM32F2XX.h>

#include "cape/encoder.h"
#include "cape/analog.h"
#include "cape/digital.h"
#include "cape/util.h"

// TIM11.1 on PB9, aka digital input 6.
static volatile uint32_t ultrasonic_pulse_length = 0;

typedef struct {
  uint32_t posedges, negedges;
} EdgeCounts;

#define COPY_EDGE_COUNTS(edge_counts, hall_effect_edges) \
  hall_effect_edges.posedges = edge_counts.negedges;     \
  hall_effect_edges.negedges = edge_counts.posedges;

#define HALL_CAPTURE(num, edges, encoder, capture) \
  void digital_capture_##num##P(void) {            \
    ++edges.posedges;                              \
    capture.posedge = encoder_read(encoder);       \
  }                                                \
  void digital_capture_##num##N(void) {            \
    ++edges.negedges;                              \
    capture.negedge = encoder_read(encoder);       \
  }
#define HALL_CAPTURE_DECL(num, edges, encoder, capture) \
  static volatile EdgeCounts edges = {0, 0};            \
  HALL_CAPTURE(num, edges, encoder, capture)

static volatile struct {
  int32_t posedge, negedge;
} pusher_distal_captures, pusher_proximal_captures;

#define SHOOTER(plunger_num, pusher_distal_num, pusher_proximal_num,         \
                latch_num, encoder)                                          \
  HALL_CAPTURE_DECL(pusher_distal_num, pusher_distal, encoder,               \
                    pusher_distal_captures);                                 \
  HALL_CAPTURE_DECL(pusher_proximal_num, pusher_proximal, encoder,           \
                    pusher_proximal_captures);                               \
  static inline void fill_shooter_values(struct DataStruct *packet) {        \
    digital_capture_disable(pusher_distal_num);                              \
    digital_capture_disable(pusher_proximal_num);                            \
    packet->main.shooter_position = encoder_read(encoder);                   \
    packet->main.pusher_distal_posedge_position =                            \
        pusher_distal_captures.negedge;                                      \
    packet->main.pusher_proximal_posedge_position =                          \
        pusher_proximal_captures.negedge;                                    \
    packet->main.bools.pusher_distal = !digital_read(pusher_distal_num);     \
    packet->main.bools.pusher_proximal = !digital_read(pusher_proximal_num); \
    COPY_EDGE_COUNTS(pusher_distal, packet->main.pusher_distal);             \
    COPY_EDGE_COUNTS(pusher_proximal, packet->main.pusher_proximal);         \
    digital_capture_enable(pusher_distal_num);                               \
    digital_capture_enable(pusher_proximal_num);                             \
    packet->main.bools.plunger = !digital_read(plunger_num);                 \
    packet->main.bools.latch = !digital_read(latch_num);                     \
  }

typedef struct {
  int32_t posedge, negedge;
  EdgeCounts front, calibration, back;
} SingleClawCaptures;

#define CLAW(front_num, calibration_num, back_num, name, encoder)           \
  static volatile SingleClawCaptures name = {0, 0, {0, 0}, {0, 0}, {0, 0}}; \
  HALL_CAPTURE(front_num, name.front, encoder, name);                       \
  HALL_CAPTURE(calibration_num, name.calibration, encoder, name);           \
  HALL_CAPTURE(back_num, name.back, encoder, name);                         \
  static inline void fill_##name##_values(struct DataStruct *packet) {      \
    digital_capture_disable(front_num);                                     \
    digital_capture_disable(calibration_num);                               \
    digital_capture_disable(back_num);                                      \
    packet->main.name.position = encoder_read(encoder);                     \
    packet->main.name.negedge_position = name.posedge;                      \
    packet->main.name.posedge_position = name.negedge;                      \
    packet->main.name.bools.front = !digital_read(front_num);               \
    packet->main.name.bools.calibration = !digital_read(calibration_num);   \
    packet->main.name.bools.back = !digital_read(back_num);                 \
    COPY_EDGE_COUNTS(name.front, packet->main.name.front);                  \
    COPY_EDGE_COUNTS(name.calibration, packet->main.name.calibration);      \
    COPY_EDGE_COUNTS(name.back, packet->main.name.back);                    \
    digital_capture_enable(front_num);                                      \
    digital_capture_enable(calibration_num);                                \
    digital_capture_enable(back_num);                                       \
  }

CLAW(1, 2, 0, top_claw, 2);
CLAW(10, 11, 9, bottom_claw, 7);
SHOOTER(7, 5, 4, 8, 0)

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
  TIM11->SR = ~TIM_SR_CC1IF;
  if (digital_read(6)) {
    TIM11->EGR = TIM_EGR_UG;
  } else {
    ultrasonic_pulse_length = TIM11->CCR1;
  }
}

void robot_init(void) {
  gpio_setup_alt(GPIOB, 9, 3);
  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
  TIM11->CR1 = TIM_CR1_URS;
  TIM11->DIER = TIM_DIER_CC1IE;
  TIM11->CCMR1 = TIM_CCMR1_CC1S_0 /* input pin 1 -> timer input 1 */;
  TIM11->CCER = TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E;
  TIM11->EGR = TIM_EGR_UG;
  TIM11->PSC = 1200 - 1;  // 100kHZ timer
  TIM11->CR1 |= TIM_CR1_CEN;
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 3);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void robot_fill_packet(struct DataStruct *packet) {
  packet->main.left_drive = encoder_read(6);
  packet->main.right_drive = encoder_read(5);
  packet->main.low_left_drive_hall = analog_get(7);
  packet->main.low_right_drive_hall = analog_get(0);
  packet->main.high_left_drive_hall = analog_get(6);
  packet->main.high_right_drive_hall = analog_get(1);

  packet->main.ultrasonic_pulse_length = ultrasonic_pulse_length;

  fill_top_claw_values(packet);
  fill_bottom_claw_values(packet);
  fill_shooter_values(packet);
}
