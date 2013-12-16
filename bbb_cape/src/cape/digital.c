#include "cape/digital.h"

#include <STM32F2XX.h>

#include "cape/util.h"

static void digital_capture_default(void) {}

void digital_capture_0P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_0N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_1P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_1N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_2P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_2N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_3P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_3N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_4P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_4N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_5P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_5N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_6P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_6N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_7P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_7N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_8P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_8N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_9P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_9N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_10P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_10N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_11P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_11N(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_12P(void) ALIAS_WEAK(digital_capture_default);
void digital_capture_12N(void) ALIAS_WEAK(digital_capture_default);

typedef void (*EXTIHandler)(uint32_t);

void EXTI2_IRQHandler(void) {
  uint32_t inputs = GPIOB->IDR;
  EXTI->PR = EXTI_PR_PR2;
  if (inputs & (1 << 2)) {
    digital_capture_11N();
  } else {
    digital_capture_11P();
  }
}

void EXTI4_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  EXTI->PR = EXTI_PR_PR4;
  if (inputs & (1 << 4)) {
    digital_capture_0N();
  } else {
    digital_capture_0P();
  }
}

static void EXTI5_Handler(uint32_t inputs) {
  if (inputs & (1 << 5)) {
    digital_capture_1N();
  } else {
    digital_capture_1P();
  }
}

static void EXTI7_Handler(uint32_t inputs) {
  if (inputs & (1 << 7)) {
    digital_capture_10N();
  } else {
    digital_capture_10P();
  }
}

static void EXTI8_Handler(uint32_t inputs) {
  if (inputs & (1 << 7)) {
    digital_capture_7N();
  } else {
    digital_capture_7P();
  }
}

static void EXTI9_Handler(uint32_t inputs) {
  if (inputs & (1 << 9)) {
    digital_capture_6N();
  } else {
    digital_capture_6P();
  }
}

void EXTI9_5_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  uint32_t exti = __clz(EXTI->PR);
  EXTI->PR = (1 << 31) >> exti;
  switch (exti) {
    case 31 - 5:
      EXTI5_Handler(inputs);
      break;
    case 31 - 7:
      EXTI7_Handler(inputs);
      break;
    case 31 - 8:
      EXTI8_Handler(inputs);
      break;
    case 31 - 9:
      EXTI9_Handler(inputs);
      break;
  }
}

static void EXTI10_Handler(uint32_t inputs) {
  if (inputs & (1 << 10)) {
    digital_capture_5N();
  } else {
    digital_capture_5P();
  }
}

static void EXTI11_Handler(uint32_t inputs) {
  if (inputs & (1 << 11)) {
    digital_capture_9N();
  } else {
    digital_capture_9P();
  }
}

static void EXTI12_Handler(uint32_t inputs) {
  if (inputs & (1 << 12)) {
    digital_capture_8N();
  } else {
    digital_capture_8P();
  }
}

static void EXTI13_Handler(uint32_t inputs) {
  if (inputs & (1 << 13)) {
    digital_capture_2N();
  } else {
    digital_capture_2P();
  }
}

static void EXTI14_Handler(uint32_t inputs) {
  if (inputs & (1 << 14)) {
    digital_capture_3N();
  } else {
    digital_capture_3P();
  }
}

static void EXTI15_Handler(uint32_t inputs) {
  if (inputs & (1 << 15)) {
    digital_capture_4N();
  } else {
    digital_capture_4P();
  }
}

void EXTI15_10_IRQHandler(void) {
  uint32_t inputs = GPIOC->IDR;
  uint32_t exti = __clz(EXTI->PR);
  EXTI->PR = (1 << 31) >> exti;
  switch (exti) {
    case 31 - 10:
      EXTI10_Handler(inputs);
      break;
    case 31 - 11:
      EXTI11_Handler(inputs);
      break;
    case 31 - 12:
      EXTI12_Handler(inputs);
      break;
    case 31 - 13:
      EXTI13_Handler(inputs);
      break;
    case 31 - 14:
      EXTI14_Handler(inputs);
      break;
    case 31 - 15:
      EXTI15_Handler(inputs);
      break;
  }
}

static void init_exti(int exti, int port) {
  EXTI_set(exti, port);
  EXTI->IMR |= 1 << exti;
  EXTI->RTSR |= 1 << exti;
  EXTI->FTSR |= 1 << exti;
}

void digital_init(void) {
  init_exti(2, 1);
  init_exti(4, 2);
  init_exti(5, 2);
  init_exti(7, 0);
  init_exti(8, 1);
  init_exti(9, 1);
  init_exti(10, 1);
  init_exti(11, 0);
  init_exti(12, 0);
  init_exti(13, 2);
  init_exti(14, 2);
  init_exti(15, 2);

  NVIC_SetPriority(EXTI2_IRQn, 1);
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_SetPriority(EXTI4_IRQn, 1);
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, 1);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, 1);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}
