#include "motors/core/itm.h"

#include "motors/core/kinetis.h"

namespace frc971 {
namespace motors {
namespace itm {
namespace {

// 480000 works for a bit but eventually errors out with an stlink-v2 and
// FRDM-K64F dev board. 200000 works for longer, but still fails eventually.
constexpr int kDesiredTraceFrequency = 192000;
constexpr int kTpiuAsynchronousClockFrequency = F_CPU;

}  // namespace

void Initialize() {
  ARM_DEMCR = ARM_DEMCR_TRCENA;

  // 0 is parallel mode (not supported on Kinetis-K22 at least).
  // 1 is asynchronous SWO with Manchester encoding ("medium speed").
  // 2 is asynchronous SWO with NRZ encoder ("low speed").
  TPIU.SPPR = 2;

  // Round up on the prescaler, to round down on the final frequency.
  TPIU.ACPR = ((kTpiuAsynchronousClockFrequency + kDesiredTraceFrequency - 1) /
               kDesiredTraceFrequency) -
              1;

  ITM.LAR = 0xC5ACCE55;

  ITM.TCR =
      V_TCR_TraceBusID(7) /* Whatever, as long as it's non-0 */ |
      V_TCR_GTSFREQ(0) /* No global timestamps */ |
      V_TCR_TSPrescale(0) /* No extra prescaling here */ |
      M_TCR_SWOENA /* Use the TPIU's clock */ |
      0 /* !M_TCR_TXENA to disable DWT events that we don't use */ |
      0 /* !M_TCR_SYNCENA to disable synchronization packet transmission */ |
      0 /* !M_TCR_TSENA to disable local timestamps */ | M_TCR_ITMENA;
  // Allow unprivileged access to the port for printing.
  ITM.TPR = 1;
  // Only enable trace port 0, because that's the one we use for printing.
  ITM.TER[0] = 1;

  // Disable everything we can here.
  DWT.CTRL = 0;

  // Indicate a trigger when the input trigger signal is asserted. Not sure if
  // this matters for anything for now, but it's what the examples all do.
  TPIU.FFCR = M_FFCR_TrigIn;

  ITM.LAR = 0;
}

}  // namespace itm
}  // namespace motors
}  // namespace frc971
