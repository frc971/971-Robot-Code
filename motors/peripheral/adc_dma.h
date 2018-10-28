#ifndef MOTORS_PERIPHERAL_ADC_DMA_H_
#define MOTORS_PERIPHERAL_ADC_DMA_H_

#include <array>

#include <stdint.h>

#include "motors/core/kinetis.h"
#include "motors/peripheral/configuration.h"
#include "motors/util.h"

namespace frc971 {
namespace teensy {

// This class manages configuring the hardware to automatically capture various
// sensor values periodically with tight timing. Currently it's only 4 samples
// on each of ADC0 and ADC1.
// TODO(Brian): Capture the encoder value (and velocity?) too.
//
// Uses:
//   * Both PDBs
//   * Both ADCs
//   * ADC_RESULT_DMA_CHANNEL and ADC_RECONFIGURE_DMA_CHANNEL
class AdcDmaSampler {
 public:
  static constexpr int kNumberAdcSamples = 4;

  // Corresponding samples in adc0_samples and adc1_samples happen
  // simultaneously. Second sample happens at the cycle boundary. Elements
  // should include the appropriate ADCH and DIFF values.
  AdcDmaSampler(int counts_per_cycle);

  // Must be called before Initialize().
  void set_adc0_samples(
      const ::std::array<uint32_t, kNumberAdcSamples> &adc0_samples);
  void set_adc1_samples(
      const ::std::array<uint32_t, kNumberAdcSamples> &adc1_samples);
  // pdb_input is the trigger-in index for PDB0 to use the attached FTM. This
  // FTM must be set to do trigger-outs on exactly two of its channels, which
  // will be dedicated to this object. This FTM must also start counting at 0 at
  // the beginning of each cycle and have MOD set to match one cycle time.
  void set_pdb_input(int pdb_input) { pdb_input_ = pdb_input; }
  // ftm_delays is pointers to the two FTM.SCn registers which are set up to do
  // trigger-outs.
  void set_ftm_delays(const ::std::array<volatile uint32_t *, 2> &ftm_delays) {
    ftm_delays_ = ftm_delays;
  }

  AdcDmaSampler(const AdcDmaSampler &) = delete;
  AdcDmaSampler &operator=(const AdcDmaSampler &) = delete;

  void Initialize();

  // Checks if the current cycle has finished reading out results.
  // After this returns true, it is safe to read the results until Reset() is
  // called.
  bool CheckDone() {
    const uint32_t mask = 1 << reconfigure_dma_channel(1);
    if (!(DMA.INT & mask)) {
      return false;
    }
    DmaMemoryBarrier();
    DMA.INT = mask;
    (void)DMA.INT;
    return true;
  }

  // Must be called after CheckDone() returns true each time.
  void Reset();

  int16_t adc_result(int adc, int i) { return adc_results_[adc][i]; }

 private:
  void InitializePdbChannel(KINETIS_PDB_CHANNEL_t *channel);

  static constexpr int result_dma_channel(int adc) {
    if (adc == 0) {
      return ADC_RESULT_DMA_CHANNEL0;
    }
    return ADC_RESULT_DMA_CHANNEL1;
  }
  static KINETIS_TCD_t *result_dma(int adc) {
    return &DMA.TCD[result_dma_channel(adc)];
  }

  static constexpr int reconfigure_dma_channel(int adc) {
    if (adc == 0) {
      return ADC_RECONFIGURE_DMA_CHANNEL0;
    }
    return ADC_RECONFIGURE_DMA_CHANNEL1;
  }
  static KINETIS_TCD_t *reconfigure_dma(int adc) {
    return &DMA.TCD[reconfigure_dma_channel(adc)];
  }

  static constexpr int encoder_value_dma_channel() {
    return ENCODER_VALUE_DMA_CHANNEL;
  }

  const int counts_per_cycle_;

  ::std::array<::std::array<volatile uint32_t, kNumberAdcSamples + 2>, 2>
      adc_sc1s_{};
  ::std::array<::std::array<volatile uint16_t, kNumberAdcSamples>, 2>
      adc_results_{};

  int pdb_input_ = 0xF;
  ::std::array<volatile uint32_t *, 2> ftm_delays_{nullptr, nullptr};
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_ADC_DMA_H_
