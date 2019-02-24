#ifndef MOTORS_UTIL_H_
#define MOTORS_UTIL_H_

#include <stdint.h>
#include <stddef.h>

#include "motors/core/kinetis.h"

#ifdef __cplusplus
extern "C"
{
#endif

// This bitband register for a specific bit of a given peripheral register.
//
// reg must be an address in one of the peripheral modules (on AIPS0, AIPS1, or
// GPIO) (0x40000000 - 0x400FFFFF).
#define PERIPHERAL_BITBAND(reg, bit)                                       \
  (*(volatile uint32_t *)(((uint32_t) & (reg)-0x40000000) * 32 + (bit)*4 + \
                          0x42000000))

#define NVIC_SET_SANE_PRIORITY(irqnum, priority) \
  NVIC_SET_PRIORITY(irqnum, ((priority)&0xF) << 4)
#define NVIC_GET_SANE_PRIORITY(irqnum) (NVIC_GET_PRIORITY(irqnum) >> 4)

// A sufficient memory barrier between writing some data and telling the
// hardware to read it or having the hardware say some data has been written and
// actually reading it.
static inline void DmaMemoryBarrier() {
  // Cortex-M3 and Cortex-M4 don't reorder loads or stores, so no DMB is
  // necessary here.
  __asm__ __volatile__("" ::: "memory");
}

static inline void InterruptMemoryBarrier() {
  // Cortex-M3 and Cortex-M4 don't reorder loads or stores, and evaluate
  // interrupts between every instruction, so no DSB or ISB is necessary here.
  // Note: up to two instructions may be executed after enabling/disabling an
  // interrupt in the NVIC before taking the new value into account, so that
  // would still require a DSB+ISB if we cared.
  // Note: enabling interrupts might not recognize interrupts immediately
  // without an ISB.
  __asm__ __volatile__("" ::: "memory", "cc");
}

// Definitions for the bits in some registers that are missing.
#define CAN_MCR_MDIS ((uint32_t)(1 << 31))
#define CAN_MCR_FRZ ((uint32_t)(1 << 30))
#define CAN_MCR_RFEN ((uint32_t)(1 << 29))
#define CAN_MCR_HALT ((uint32_t)(1 << 28))
#define CAN_MCR_NOTRDY ((uint32_t)(1 << 27))
#define CAN_MCR_WAKMSK ((uint32_t)(1 << 26))
#define CAN_MCR_SOFTRST ((uint32_t)(1 << 25))
#define CAN_MCR_FRZACK ((uint32_t)(1 << 24))
#define CAN_MCR_SUPV ((uint32_t)(1 << 23))
#define CAN_MCR_SLFWAK ((uint32_t)(1 << 22))
#define CAN_MCR_WRNEN ((uint32_t)(1 << 21))
#define CAN_MCR_LPMACK ((uint32_t)(1 << 20))
#define CAN_MCR_WAKSRC ((uint32_t)(1 << 19))
#define CAN_MCR_SRXDIS ((uint32_t)(1 << 17))
#define CAN_MCR_IRMQ ((uint32_t)(1 << 16))
#define CAN_MCR_LPRIOEN ((uint32_t)(1 << 13))
#define CAN_MCR_AEN ((uint32_t)(1 << 12))
#define CAN_MCR_IDAM(n) ((uint32_t)(((n) & 3) << 8))
#define CAN_MCR_MAXMB(n) ((uint32_t)((n) & 0x7F))
#define CAN_CTRL1_PRESDIV(n) ((uint32_t)(((n) & 0xFF) << 24))
#define CAN_CTRL1_RJW(n) ((uint32_t)(((n) & 3) << 22))
#define CAN_CTRL1_PSEG1(n) ((uint32_t)(((n) & 7) << 19))
#define CAN_CTRL1_PSEG2(n) ((uint32_t)(((n) & 7) << 16))
#define CAN_CTRL1_BOFFMSK ((uint32_t)(1 << 15))
#define CAN_CTRL1_ERRMSK ((uint32_t)(1 << 14))
#define CAN_CTRL1_CLKSRC ((uint32_t)(1 << 13))
#define CAN_CTRL1_LPB ((uint32_t)(1 << 12))
#define CAN_CTRL1_TWRNMSK ((uint32_t)((1 << 11))
#define CAN_CTRL1_RWRNMSK ((uint32_t)((1 << 10))
#define CAN_CTRL1_SMP ((uint32_t)(1 << 7))
#define CAN_CTRL1_BOFFREC ((uint32_t)(1 << 6)
#define CAN_CTRL1_TSYN ((uint32_t)(1 << 5))
#define CAN_CTRL1_LBUF ((uint32_t)(1 << 4))
#define CAN_CTRL1_LOM ((uint32_t)(1 << 3))
#define CAN_CTRL1_PROPSEG(n) ((uint32_t)((n) & 7))
#define CAN_ESR1_SYNCH ((uint32_t)(1 << 18))
#define CAN_ESR1_TWRNINT ((uint32_t)(1 << 17))
#define CAN_ESR1_RWRNINT ((uint32_t)(1 << 16))
#define CAN_ESR1_BIT1ERR ((uint32_t)(1 << 15))
#define CAN_ESR1_BIT0ERR ((uint32_t)(1 << 14))
#define CAN_ESR1_ACKERR ((uint32_t)(1 << 13))
#define CAN_ESR1_CRCERR ((uint32_t)(1 << 12))
#define CAN_ESR1_FRMERR ((uint32_t)(1 << 11))
#define CAN_ESR1_STFERR ((uint32_t)(1 << 10))
#define CAN_ESR1_TXWRN ((uint32_t)(1 << 9))
#define CAN_ESR1_RXWRN ((uint32_t)(1 << 8))
#define CAN_ESR1_IDLE ((uint32_t)(1 << 7))
#define CAN_ESR1_TX ((uint32_t)(1 << 6))
#define CAN_ESR1_RX ((uint32_t)(1 << 3))
#define CAN_ESR1_BOFFINT ((uint32_t)(1 << 2))
#define CAN_ESR1_ERRINT ((uint32_t)(1 << 1))
#define CAN_ESR1_WAKINT ((uint32_t)1)
#define CAN_CTRL2_WRMFRZ ((uint32_t)(1 << 28))
#define CAN_CTRL2_RFFN(n) ((uint32_t)(((n) & 0xF) << 24))
#define CAN_CTRL2_TASD(n) ((uint32_t)(((n) & 0x1F) << 19))
#define CAN_CTRL2_MRP ((uint32_t)(1 << 18))
#define CAN_CTRL2_RRS ((uint32_t)(1 << 17))
#define CAN_CTRL2_EACEN ((uint32_t)(1 << 16))
#define CAN_ESR2_VPS ((uint32_t)(1 << 14))
#define CAN_ESR2_IMB ((uint32_t)(1 << 13))

typedef struct {
  // Timestamp is the lower 16 bits.
  uint32_t control_timestamp;
  uint32_t prio_id;
  uint32_t data[2];
} CanMessageBuffer;
#define CAN0_MESSAGES ((volatile CanMessageBuffer *)0x40024080)
#define CAN0_RXIMRS ((volatile uint32_t *)0x40024880)
#define CAN1_MESSAGES ((volatile CanMessageBuffer *)0x400A4080)
#define CAN1_RXIMRS ((volatile uint32_t *)0x400A4880)
#define CAN_MB_CONTROL_INSERT_DLC(dlc) ((uint32_t)(((dlc) & 0xF) << 16))
#define CAN_MB_CONTROL_EXTRACT_DLC(control_timestamp) \
  ((control_timestamp >> 16) & 0xF)
#define CAN_MB_CONTROL_RTR ((uint32_t)(1 << 20))
#define CAN_MB_CONTROL_IDE ((uint32_t)(1 << 21))
#define CAN_MB_CONTROL_SRR ((uint32_t)(1 << 22))
#define CAN_MB_CONTROL_INSERT_CODE(n) ((uint32_t)(((n) & 0xF) << 24))
#define CAN_MB_CONTROL_EXTRACT_CODE(n) ((uint32_t)(((n) >> 24) & 0xF))
#define CAN_MB_CONTROL_CODE_BUSY_MASK CAN_MB_CONTROL_INSERT_CODE(1)
#define CAN_MB_PRIO_ID_PRIORITY_MASK ((uint32_t)((1 << 29) - 1))
#define CAN_MB_CODE_RX_INACTIVE 0
#define CAN_MB_CODE_RX_EMPTY 4
#define CAN_MB_CODE_RX_FULL 2
#define CAN_MB_CODE_RX_OVERRUN 6
#define CAN_MB_CODE_RX_RANSWER 0xA
#define CAN_MB_CODE_TX_INACTIVE 8
#define CAN_MB_CODE_TX_ABORT 9
#define CAN_MB_CODE_TX_DATA 0xC
#define CAN_MB_CODE_TX_REMOTE 0xC
#define CAN_MB_CODE_TX_TANSWER 0xE
#define CAN_MB_CODE_IS_BUSY(code) ((code) & 1)

// We have to define these, and leave them defined, because the C preprocessor
// is annoying...
#define REALLY_DO_CONCATENATE(x, y, z) x ## y ## z
#define DO_CONCATENATE(x, y, z) REALLY_DO_CONCATENATE(x, y, z)

// Index-parameterized access to various registers from various peripherals.
// This only includes ones somebody thought might be useful; add more if you
// want them.
#define dma_chN_isr(n) DO_CONCATENATE(dma_ch, n, _isr)
#define IRQ_DMA_CHn(n) DO_CONCATENATE(IRQ_DMA, _CH, n)

#define USB0_ENDPTn(n) (*(volatile uint8_t *)(0x400720C0 + ((n)*4)))

// TODO(Brian): Just write the structs out, and do all this in kinetis.h.
#define ALL_FTM_REGISTERS         \
  FOR_BOTH_FTM_REGISTER(SC)       \
  FOR_BOTH_FTM_REGISTER(CNT)      \
  FOR_BOTH_FTM_REGISTER(MOD)      \
  FOR_BOTH_FTM_REGISTER(C0SC)     \
  FOR_BOTH_FTM_REGISTER(C0V)      \
  FOR_BOTH_FTM_REGISTER(C1SC)     \
  FOR_BOTH_FTM_REGISTER(C1V)      \
  FOR_BIG_FTM_REGISTER(C2SC)      \
  FOR_BIG_FTM_REGISTER(C2V)       \
  FOR_BIG_FTM_REGISTER(C3SC)      \
  FOR_BIG_FTM_REGISTER(C3V)       \
  FOR_BIG_FTM_REGISTER(C4SC)      \
  FOR_BIG_FTM_REGISTER(C4V)       \
  FOR_BIG_FTM_REGISTER(C5SC)      \
  FOR_BIG_FTM_REGISTER(C5V)       \
  FOR_BIG_FTM_REGISTER(C6SC)      \
  FOR_BIG_FTM_REGISTER(C6V)       \
  FOR_BIG_FTM_REGISTER(C7SC)      \
  FOR_BIG_FTM_REGISTER(C7V)       \
  FOR_BOTH_FTM_REGISTER(CNTIN)    \
  FOR_BOTH_FTM_REGISTER(STATUS)   \
  FOR_BOTH_FTM_REGISTER(MODE)     \
  FOR_BOTH_FTM_REGISTER(SYNC)     \
  FOR_BOTH_FTM_REGISTER(OUTINIT)  \
  FOR_BOTH_FTM_REGISTER(OUTMASK)  \
  FOR_BOTH_FTM_REGISTER(COMBINE)  \
  FOR_BOTH_FTM_REGISTER(DEADTIME) \
  FOR_BOTH_FTM_REGISTER(EXTTRIG)  \
  FOR_BOTH_FTM_REGISTER(POL)      \
  FOR_BOTH_FTM_REGISTER(FMS)      \
  FOR_BOTH_FTM_REGISTER(FILTER)   \
  FOR_BOTH_FTM_REGISTER(FLTCTRL)  \
  FOR_LITTLE_FTM_REGISTER(QDCTRL) \
  FOR_BOTH_FTM_REGISTER(CONF)     \
  FOR_BOTH_FTM_REGISTER(FLTPOL)   \
  FOR_BOTH_FTM_REGISTER(SYNCONF)  \
  FOR_BOTH_FTM_REGISTER(INVCTRL)  \
  FOR_BOTH_FTM_REGISTER(SWOCTRL)  \
  FOR_BOTH_FTM_REGISTER(PWMLOAD)

typedef struct {
#define FOR_BIG_FTM_REGISTER(name) volatile uint32_t name;
#define FOR_BOTH_FTM_REGISTER(name) volatile uint32_t name;
#define FOR_LITTLE_FTM_REGISTER(name) const uint32_t _reserved_##name;
  ALL_FTM_REGISTERS
#undef FOR_BIG_FTM_REGISTER
#undef FOR_LITTLE_FTM_REGISTER
} BigFTM;

typedef struct {
#define FOR_BIG_FTM_REGISTER(name) const uint32_t _reserved_##name;
#define FOR_LITTLE_FTM_REGISTER(name) volatile uint32_t name;
        ALL_FTM_REGISTERS
#undef FOR_BIG_FTM_REGISTER
#undef FOR_LITTLE_FTM_REGISTER
#undef FOR_BOTH_FTM_REGISTER
} LittleFTM;

#define FTM0 ((BigFTM *)0x40038000)
#define FTM1 ((LittleFTM *)0x40039000)
#define FTM2 ((LittleFTM *)0x400B8000)
#define FTM3 ((BigFTM *)0x400B9000)

#undef ALL_FTM_REGISTERS

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// RAII class to disable interrupts temporarily.
class DisableInterrupts {
 public:
  DisableInterrupts() { __disable_irq(); }
  ~DisableInterrupts() { __enable_irq(); }

  DisableInterrupts(const DisableInterrupts &) = delete;
  DisableInterrupts &operator=(const DisableInterrupts &) = delete;
};

class ReenableInterrupts {
 public:
  ReenableInterrupts(DisableInterrupts *) {
    __enable_irq();
    // Because we're on a Cortex-M4, we don't need an ISB here to ensure
    // interrupts are processed.
  }
  ~ReenableInterrupts() { __disable_irq(); }

  ReenableInterrupts(const ReenableInterrupts &) = delete;
  ReenableInterrupts &operator=(const ReenableInterrupts &) = delete;
};

// constexpr log base 2, which fails to compile for non-power-of-2 inputs.
// This is a silly implementation to use at runtime.
template<typename T>
constexpr T ConstexprLog2(T i) {
  if (i == 0) {
    __builtin_abort();
  }
  if (i == 1) {
    return 0;
  }
  if (i / 2 * 2 == i) {
    return 1 + ConstexprLog2(i / 2);
  }
  __builtin_abort();
}

#endif  // __cplusplus

#endif  // MOTORS_UTIL_H_
