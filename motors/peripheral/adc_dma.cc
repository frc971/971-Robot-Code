#include "motors/peripheral/adc_dma.h"

#include <assert.h>

// Design notes:
// * Want to grab 3 differential-pair captures in rapid succession (aka 3 pairs
//   each containing 2 differential ADC input values)
// * Use hardware triggering to allow triggering captures on both ADCs at the
//   exact same time
// * Need to use both A and B channels within each ADC because writing to SC1n
//   while it's doing a capture (like to set up the next one) aborts the capture
// * Can't use alternate (non-PDB) ADC triggers, because we want to use both A
//   and B channels within each ADC, and only the PDB triggers know how to use
//   two pre-triggers to choose which ADC channel to trigger
// * Back-to-back connections aren't directly helpful because they're only
//   set up to go ADC0.A, ADC0.B, ADC1.A, ADC1.B, in a loop
// * Setup:
//   * Set ADC0 and ADC1 for hardware triggering
//   * Write to ADC0_SC1A and ADC1_SC1A (with CPU) with initial values
//   * PDB:
//     * One-shot mode
//     * A output bypassed (so it goes right after the trigger)
//       * Triggered by FTM trigger outputs (twice per cycle)
//     * B output in back-to-back mode
//     * Delays don't matter
//     * A doesn't re-trigger off of B, so PDB+ADC by themselves stop after
//       doing two samples
//     * FTM triggers it at the appropriate points in the cycle (using two
//       otherwise-unused FTM channels)
//   * DMA:
//     * One result triggers off of ADC.COCO (either one)
//       * SOFF moves between RA and RB
//       * 1 minor loop = reading Rn from one ADC
//       * Trigger other result channel or first reconfigure channel after
//         (link on both major and minor completion)
//       * SMOD wraps back to RB after RA
//       * One major iteration is all four samples
//       * Can't use SOFF and SMOD to read all results with one channel because
//         SOFF is too small
//     * Same idea for two reconfigure channels
//     * Use DREQ so disabled after doing all four for CPU to read results
//     * Configure to reset everything after the major iteration so CPU just
//       has to re-enable
// * Desired sequence:
//   1. Trigger ADC0.A and ADC1.A
//   2. [Reading ADC0.RB and ADC1.RB would be OK]
//   3. Wait for ADC0.A and ADC1.A to finish
//   4. Trigger ADC0.B and ADC0.B
//   5. Read ADC0.RA and ADC1.RA
//   6. Wait for ADC0.B and ADC1.B to finish
//   7. Trigger ADC0.A and ADC0.A
//   8. Read ADC0.RB and ADC1.RB
//   9. Go back to step #3

namespace frc971 {
namespace teensy {
namespace {

constexpr uint32_t pdb_sc(int pdb_input) {
  return V_PDB_LDMOD(0) /* Load immediately after LDOK */ |
         V_PDB_PRESCALER(0) /* Doesn't matter */ | V_PDB_TRGSEL(pdb_input) |
         M_PDB_PDBEN /* Enable it */ | V_PDB_MULT(0) /* Doesn't matter */ |
         M_PDB_LDOK /* Load new values now */;
}

}  // namespace

AdcDmaSampler::AdcDmaSampler(int counts_per_cycle) : counts_per_cycle_(counts_per_cycle) {
  for (int adc = 0; adc < 2; ++adc) {
    for (int i = 0; i < 2; ++i) {
      adc_sc1s_[adc][kNumberAdcSamples + i] = ADC_SC1_ADCH(0x1f);
    }
  }
}

void AdcDmaSampler::set_adc0_samples(
    const ::std::array<uint32_t, kNumberAdcSamples> &adc0_samples) {
  for (int i = 0; i < kNumberAdcSamples; ++i) {
    adc_sc1s_[0][i] = adc0_samples[i] | ADC_SC1_AIEN;
  }
}

void AdcDmaSampler::set_adc1_samples(
    const ::std::array<uint32_t, kNumberAdcSamples> &adc1_samples) {
  for (int i = 0; i < kNumberAdcSamples; ++i) {
    adc_sc1s_[1][i] = adc1_samples[i] | ADC_SC1_AIEN;
  }
}

void AdcDmaSampler::Initialize() {
  // TODO(Brian): Put this somewhere better?
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX | SIM_SCGC6_PDB;

  assert(ftm_delays_[0] != nullptr);
  assert(ftm_delays_[1] != nullptr);
  {
    // See "Figure 33-92. Conversion time equation" for details on this math.
    // All the math is in bus clock cycles, until being divided into FTM
    // clock cycles at the end.

    // Divider from bus clock to FTM clock.
    // TODO(Brian): Make it so this can actually change.
    const int ftm_clock_divider = 1;

    // Divider from bus clock to ADC clock.
    // TODO(Brian): Make sure this stays in sync with what adc.cc actually
    // configures.
    static constexpr int kAdcClockDivider = 4;

    static constexpr int kSfcAdder = 5 * kAdcClockDivider + 5;

    // 12-bit single-ended is only 20, but waiting a bit too long for some of
    // the samples doesn't hurt anything.
    static constexpr int kBct = 30 /* 13-bit differential */ * kAdcClockDivider;

    static constexpr int kLstAdder = 0 * kAdcClockDivider;

    static constexpr int kHscAdder = 2 * kAdcClockDivider;


    static constexpr int kConversionTime =
        kSfcAdder + 1 /* AverageNum */ * (kBct + kLstAdder + kHscAdder);

    // Sampling takes 8 ADCK cycles according to
    // "33.4.4.5 Sample time and total conversion time". This means we want 0
    // (the start of the cycle) to be 4 ADCK cycles into the second of our four
    // samples.
    const int delay_before_cycle =
        (kConversionTime -
         4 /* Clocks before middle of sample */ * kAdcClockDivider +
         ftm_clock_divider - 1) /
        ftm_clock_divider;
    const int ftm_delay =
        (kConversionTime * 2 /* 2 ADC samples */ + ftm_clock_divider - 1) /
        ftm_clock_divider;
    *ftm_delays_[0] = counts_per_cycle_ - delay_before_cycle;
    *ftm_delays_[1] = ftm_delay - delay_before_cycle;
  }

  InitializePdbChannel(&PDB0.CH[0]);
  InitializePdbChannel(&PDB0.CH[1]);
  PDB0.MOD = 1 /* Doesn't matter */;
  PDB0.SC = pdb_sc(pdb_input_);

  DMAMUX0.CHCFG[result_dma_channel(0)] = M_DMAMUX_ENBL | DMAMUX_SOURCE_ADC0;
  DMAMUX0.CHCFG[result_dma_channel(1)] = 0;
  DMAMUX0.CHCFG[reconfigure_dma_channel(0)] = 0;
  DMAMUX0.CHCFG[reconfigure_dma_channel(1)] = 0;

  static constexpr ssize_t kResultABOffset =
      static_cast<ssize_t>(offsetof(KINETIS_ADC_t, RB)) -
      static_cast<ssize_t>(offsetof(KINETIS_ADC_t, RA));
  static_assert(kResultABOffset > 0, "Offset is backwards");
  static constexpr ssize_t kResultOffsetBits =
      ConstexprLog2(kResultABOffset * 2);
  static constexpr ssize_t kSC1ABOffset =
      static_cast<ssize_t>(offsetof(KINETIS_ADC_t, SC1B)) -
      static_cast<ssize_t>(offsetof(KINETIS_ADC_t, SC1A));
  static_assert(kSC1ABOffset > 0, "Offset is backwards");
  static constexpr ssize_t kSC1OffsetBits = ConstexprLog2(kSC1ABOffset * 2);
  for (int adc = 0; adc < 2; ++adc) {
    // Make sure we can actually write to all the fields in the DMA channels.
    DMA.CDNE = result_dma_channel(adc);
    DMA.CDNE = reconfigure_dma_channel(adc);
    DMA.CERQ = result_dma_channel(adc);
    DMA.CERQ = reconfigure_dma_channel(adc);

    ADC(adc)
        ->SC2 |= ADC_SC2_ADTRG /* Use hardware triggering */ |
                 ADC_SC2_DMAEN /* Enable DMA triggers */;

    int next_result_channel, next_reconfigure_channel;
    if (adc == 0) {
      next_result_channel = result_dma_channel(1);
      next_reconfigure_channel = reconfigure_dma_channel(1);
    } else {
      next_result_channel = reconfigure_dma_channel(0);
#if 0
      // TODO(Brian): Use this once we're actually doing encoder captures.
      next_reconfigure_channel = encoder_value_dma_channel();
#else
      next_reconfigure_channel = -1;
#endif
    }
    result_dma(adc)->SOFF = kResultABOffset;
    reconfigure_dma(adc)->SOFF = sizeof(uint32_t);
    result_dma(adc)->SADDR = &ADC(adc)->RA;
    reconfigure_dma(adc)->SADDR = &adc_sc1s_[adc][2];
    result_dma(adc)->ATTR =
        V_TCD_SMOD(kResultOffsetBits) | V_TCD_SSIZE(TCD_SIZE_16BIT) |
        V_TCD_DMOD(0) /* No DMOD */ | V_TCD_DSIZE(TCD_SIZE_16BIT);
    reconfigure_dma(adc)->ATTR =
        V_TCD_SMOD(0) /* No SMOD */ | V_TCD_SSIZE(TCD_SIZE_32BIT) |
        V_TCD_DMOD(kSC1OffsetBits) | V_TCD_DSIZE(TCD_SIZE_32BIT);
    result_dma(adc)->NBYTES = sizeof(uint16_t);
    reconfigure_dma(adc)->NBYTES = sizeof(uint32_t);
    result_dma(adc)->SLAST = 0;
    reconfigure_dma(adc)->SLAST = -(kNumberAdcSamples * sizeof(uint32_t));
    result_dma(adc)->DADDR = &adc_results_[adc][0];
    reconfigure_dma(adc)->DADDR = &ADC(adc)->SC1A;
    result_dma(adc)->DOFF = sizeof(uint16_t);
    reconfigure_dma(adc)->DOFF = kSC1ABOffset;
    {
      uint16_t link = 0;
      if (next_result_channel != -1) {
        link = M_TCD_ELINK | V_TCD_LINKCH(next_result_channel);
      }
      result_dma(adc)->CITER = result_dma(adc)->BITER =
          link | 4 /* 4 minor iterations */;
    }
    {
      uint16_t link = 0;
      if (next_reconfigure_channel != -1) {
        link = M_TCD_ELINK | V_TCD_LINKCH(next_reconfigure_channel);
      }
      reconfigure_dma(adc)->CITER = reconfigure_dma(adc)->BITER =
          link | 4 /* 4 minor iterations */;
    }
    result_dma(adc)->DLASTSGA = -(kNumberAdcSamples * sizeof(uint16_t));
    reconfigure_dma(adc)->DLASTSGA = 0;
    {
      uint16_t link = 0;
      if (next_result_channel != -1) {
        link = V_TCD_MAJORLINKCH(next_result_channel) | M_TCD_MAJORELINK;
      }
      result_dma(adc)->CSR =
          V_TCD_BWC(0) /* No extra stalls */ | link |
          M_TCD_DREQ /* Disable requests after major iteration */;
    }
    {
      uint16_t link = 0;
      if (next_reconfigure_channel != -1) {
        link = V_TCD_MAJORLINKCH(next_reconfigure_channel) | M_TCD_MAJORELINK;
      }
      reconfigure_dma(adc)->CSR =
          V_TCD_BWC(0) /* No extra stalls */ | link |
          M_TCD_DREQ /* Disable requests after major iteration */ |
          M_TCD_INTMAJOR;
    }
  }
}

void AdcDmaSampler::Reset() {
  // Disable the PDB. This both prevents weird interference with what we're
  // trying to do and is part of the process to clear its internal
  // (unobservable) "locks". If we get out of sync, then the DMA won't read from
  // the ADC so the ADC's COCO will stay asserted and the PDB will stay "locked"
  // on that channel. Disabling then re-enabling the PDB is the easiest way to
  // clear that. See "37.4.1 PDB pre-trigger and trigger outputs" in the
  // reference manual for details.
  PDB0.SC = 0;

  // Set the initial ADC sample configs.
  for (int adc = 0; adc < 2; ++adc) {
    // Tell the DMA it should go again.
    DMA.CDNE = result_dma_channel(adc);
    DMA.CDNE = reconfigure_dma_channel(adc);

    for (int i = 0; i < 2; ++i) {
      ADC(adc)->SC1[i] = adc_sc1s_[adc][i];
    }
  }

  // Re-enable the first DMA channel (which is the only one triggered by
  // hardware, and disables itself once it's done).
  DMA.SERQ = result_dma_channel(0);

  // Now re-enable the PDB.
  PDB0.SC = pdb_sc(pdb_input_);
  // Both channels' S registers are now 0, empirically, regardless of what they
  // were before. Reference manual isn't super clear what's supposed to happen.
}

void AdcDmaSampler::InitializePdbChannel(KINETIS_PDB_CHANNEL_t *channel) {
  channel->C1 = V_PDB_BB(2) /* Back-to-back enabled for channel 1 */ |
                V_PDB_TOS(2) /* Bypass 0, and 1 doesn't matter (?) */ |
                V_PDB_EN(3) /* Enable our two */;
}

}  // namespace teensy
}  // namespace frc971
