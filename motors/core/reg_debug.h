#ifndef MOTORS_CORE_REG_DEBUG_H_
#define MOTORS_CORE_REG_DEBUG_H_

#include <stdint.h>
#include <stddef.h>

// Cortex-M4 always has 32 registers according to its Technical Reference
// Manual.
#define ITM_NUM_STIM 32

// Cortex-M4 TRM is incomplete. See the Cortex-M3 one for all the registers
// here (and some forms of the M3 one are missing registers too?):
// https://developer.arm.com/docs/ddi0337/e/system-debug/itm/summary-and-description-of-the-itm-registers#BABBCEHF
struct ARM_ITM {
  volatile uint32_t STIM[ITM_NUM_STIM];
  volatile uint32_t unused1[896 - ITM_NUM_STIM];
  volatile uint32_t TER[(ITM_NUM_STIM + 31) / 32];
  volatile uint32_t unused2[16 - ((ITM_NUM_STIM + 31) / 32)];
  volatile uint32_t TPR;
  volatile uint32_t unused3[15];
  volatile uint32_t TCR;
  volatile uint32_t unused4[29];
  volatile uint32_t IWR;
  const volatile uint32_t IRR;
  volatile uint32_t IMC;
  volatile uint32_t unused5[43];
  volatile uint32_t LAR;
  const volatile uint32_t LSR;
  volatile uint32_t unused6[6];
  union {
    struct {
      const volatile uint32_t PID4;
      const volatile uint32_t PID5;
      const volatile uint32_t PID6;
      const volatile uint32_t PID7;
      const volatile uint32_t PID0;
      const volatile uint32_t PID1;
      const volatile uint32_t PID2;
      const volatile uint32_t PID3;
    };
    const volatile uint32_t PIDx[8];
  };
} __attribute__((aligned(0x1000)));
static_assert(offsetof(ARM_ITM, TER) == 0xE00, "padding is wrong");
static_assert(offsetof(ARM_ITM, TPR) == 0xE40, "padding is wrong");
static_assert(offsetof(ARM_ITM, TCR) == 0xE80, "padding is wrong");
static_assert(offsetof(ARM_ITM, IWR) == 0xEF8, "padding is wrong");
static_assert(offsetof(ARM_ITM, LAR) == 0xFB0, "padding is wrong");
static_assert(offsetof(ARM_ITM, PID4) == 0xFD0, "padding is wrong");
static_assert(sizeof(ARM_ITM) == 0x1000, "padding is wrong");
#define ITM (*(ARM_ITM *)0xE0000000)

#define V_TCR_TraceBusID(n) ((static_cast<uint32_t>(n) & UINT32_C(0x7F)) << 16)
#define V_TCR_GTSFREQ(n) ((static_cast<uint32_t>(n) & UINT32_C(3)) << 10)
#define V_TCR_TSPrescale(n) ((static_cast<uint32_t>(n) & UINT32_C(3)) << 8)
#define M_TCR_SWOENA (UINT32_C(1) << 4)
#define M_TCR_TXENA (UINT32_C(1) << 3)
#define M_TCR_SYNCENA (UINT32_C(1) << 2)
#define M_TCR_TSENA (UINT32_C(1) << 1)
#define M_TCR_ITMENA (UINT32_C(1) << 0)

struct ARM_TPIU {
  const volatile uint32_t SSPSR;
  volatile uint32_t CSPSR;
  volatile uint32_t unused1[2];
  volatile uint32_t ACPR;
  volatile uint32_t unused2[55];
  volatile uint32_t SPPR;
  volatile uint32_t unused3[131];
  const volatile uint32_t FFSR;
  volatile uint32_t FFCR;
  const volatile uint32_t FSCR;
  volatile uint32_t unused4[759];
  const volatile uint32_t TRIGGER;
  const volatile uint32_t FIFO_data0;
  const volatile uint32_t ITATBCTR2;
  const volatile uint32_t FIFO_data1;
  const volatile uint32_t ITATBCTR0;
  volatile uint32_t unused5;
  volatile uint32_t ITCTRL;
  volatile uint32_t unused6[39];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
  volatile uint32_t unsued7[8];
  const volatile uint32_t DEVID;
  const volatile uint32_t DEVTYPE;
  union {
    struct {
      const volatile uint32_t PID4;
      const volatile uint32_t PID5;
      const volatile uint32_t PID6;
      const volatile uint32_t PID7;
      const volatile uint32_t PID0;
      const volatile uint32_t PID1;
      const volatile uint32_t PID2;
      const volatile uint32_t PID3;
    };
    const volatile uint32_t PIDx[8];
  };
  const volatile uint32_t CID[4];
} __attribute__((aligned(0x1000)));
static_assert(offsetof(ARM_TPIU, ACPR) == 0x10, "padding is wrong");
static_assert(offsetof(ARM_TPIU, SPPR) == 0xF0, "padding is wrong");
static_assert(offsetof(ARM_TPIU, FFSR) == 0x300, "padding is wrong");
static_assert(offsetof(ARM_TPIU, TRIGGER) == 0xEE8, "padding is wrong");
static_assert(offsetof(ARM_TPIU, PID4) == 0xFD0, "padding is wrong");
static_assert(sizeof(ARM_TPIU) == 0x1000, "padding is wrong");
#define TPIU (*(ARM_TPIU *)0xE0040000)

#define M_FFCR_EnFCont (UINT32_C(1) << 1)
#define M_FFCR_TrigIn (UINT32_C(1) << 8)

struct ARM_DWT {
  struct Compare {
    volatile uint32_t COMP;
    volatile uint32_t MASK;
    volatile uint32_t FUNCTION;
    volatile uint32_t unused1;
  };

  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  const volatile uint32_t PCSR;
  Compare comp[4];
  volatile uint32_t unused1[988];
  union {
    struct {
      const volatile uint32_t PID4;
      const volatile uint32_t PID5;
      const volatile uint32_t PID6;
      const volatile uint32_t PID7;
      const volatile uint32_t PID0;
      const volatile uint32_t PID1;
      const volatile uint32_t PID2;
      const volatile uint32_t PID3;
    };
    const volatile uint32_t PIDx[8];
  };
  const volatile uint32_t CID[4];
} __attribute__((aligned(0x1000)));
static_assert(offsetof(ARM_DWT, PID4) == 0xFD0, "padding is wrong");
static_assert(sizeof(ARM_DWT) == 0x1000, "padding is wrong");
#define DWT (*(ARM_DWT *)0xE0001000)

#define G_DWT_CTRL_NUMCOMP(n) ((static_cast<uint32_t>(n) >> 28) & UINT32_C(0xF))
#define M_DWT_CTRL_NOTRCPKT (UINT32_C(1) << 27)
#define M_DWT_CTRL_NOEXTTRIG (UINT32_C(1) << 26)
#define M_DWT_CTRL_NOCYCCNT (UINT32_C(1) << 25)
#define M_DWT_CTRL_NOPRFCNT (UINT32_C(1) << 24)
#define M_DWT_CTRL_CYCEVTENA (UINT32_C(1) << 22)
#define M_DWT_CTRL_FOLDEVTENA (UINT32_C(1) << 21)
#define M_DWT_CTRL_LSUEVTENA (UINT32_C(1) << 20)
#define M_DWT_CTRL_SLEEPEVTENA (UINT32_C(1) << 19)
#define M_DWT_CTRL_EXCEVTENA (UINT32_C(1) << 18)
#define M_DWT_CTRL_CPIEVTENA (UINT32_C(1) << 17)
#define M_DWT_CTRL_EXCTRCENA (UINT32_C(1) << 16)
#define M_DWT_CTRL_PCSAMPLENA (UINT32_C(1) << 12)
#define V_DWT_CTRL_SYNCTAP(n) ((static_cast<uint32_t>(n) & UINT32_C(3)) << 10)
#define G_DWT_CTRL_SYNCTAP(n) ((static_cast<uint32_t>(n) >> 10) & UINT32_C(3))
#define M_DWT_CTRL_CYCTAP (UINT32_C(1) << 9)
#define V_DWT_CTRL_POSTINIT(n) ((static_cast<uint32_t>(n) & UINT32_C(0xF)) << 5))
#define G_DWT_CTRL_POSTINIT(n) ((static_cast<uint32_t>(n) >> 5) & UINT32_C(0xF))
#define V_DWT_CTRL_POSTPRESET(n) \
  ((static_cast<uint32_t>(n) & UINT32_C(0xF)) << 1)
#define G_DWT_CTRL_POSTPRESET(n) \
  ((static_cast<uint32_t>(n) >> 1) & UINT32_C(0xF))
#define M_DWT_CTRL_CYCCNTENA (UINT32_C(1) << 0)

#endif  // MOTORS_CORE_REG_DEBUG_H_
