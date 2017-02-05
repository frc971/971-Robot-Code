#include "frc971/wpilib/dma.h"

#include <string.h>

#include <algorithm>
#include <type_traits>

#include "DigitalSource.h"
#include "AnalogInput.h"
#include "Encoder.h"
#ifdef WPILIB2017
#include "HAL/HAL.h"
#endif

// Interface to the roboRIO FPGA's DMA features.

// Like tEncoder::tOutput with the bitfields reversed.
typedef union {
  struct {
    unsigned Direction: 1;
    signed Value: 31;
  };
  struct {
    unsigned value: 32;
  };
} t1Output;

static const int32_t kNumHeaders = 10;

static constexpr ssize_t kChannelSize[20] = {2, 2, 4, 4, 2, 2, 4, 4, 3, 3,
                                             2, 1, 4, 4, 4, 4, 4, 4, 4, 4};

#ifndef WPILIB2017
#define HAL_GetErrorMessage getHALErrorMessage
#endif

enum DMAOffsetConstants {
  kEnable_AI0_Low = 0,
  kEnable_AI0_High = 1,
  kEnable_AIAveraged0_Low = 2,
  kEnable_AIAveraged0_High = 3,
  kEnable_AI1_Low = 4,
  kEnable_AI1_High = 5,
  kEnable_AIAveraged1_Low = 6,
  kEnable_AIAveraged1_High = 7,
  kEnable_Accumulator0 = 8,
  kEnable_Accumulator1 = 9,
  kEnable_DI = 10,
  kEnable_AnalogTriggers = 11,
  kEnable_Counters_Low = 12,
  kEnable_Counters_High = 13,
  kEnable_CounterTimers_Low = 14,
  kEnable_CounterTimers_High = 15,
  kEnable_Encoders_Low = 16,
  kEnable_Encoders_High = 17,
  kEnable_EncoderTimers_Low = 18,
  kEnable_EncoderTimers_High = 19,
};

DMA::DMA() {
  tRioStatusCode status = 0;
  tdma_config_ = tDMA::create(&status);
  tdma_config_->writeConfig_ExternalClock(false, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }
  SetRate(1);
  SetPause(false);
}

DMA::~DMA() {
  tRioStatusCode status = 0;

  manager_->stop(&status);
  delete tdma_config_;
}

void DMA::SetPause(bool pause) {
  tRioStatusCode status = 0;
  tdma_config_->writeConfig_Pause(pause, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void DMA::SetRate(uint32_t cycles) {
  if (cycles < 1) {
    cycles = 1;
  }
  tRioStatusCode status = 0;
  tdma_config_->writeRate(cycles, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void DMA::Add(Encoder *encoder) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }
  const int index = encoder->GetFPGAIndex();

  if (index < 4) {
    // TODO(austin): Encoder uses a Counter for 1x or 2x; quad for 4x...
    tdma_config_->writeConfig_Enable_Encoders_Low(true, &status);
  } else if (index < 8) {
    // TODO(austin): Encoder uses a Counter for 1x or 2x; quad for 4x...
    tdma_config_->writeConfig_Enable_Encoders_High(true, &status);
  } else {
    wpi_setErrorWithContext(
        NiFpga_Status_InvalidParameter,
        "FPGA encoder index is not in the 4 that get logged.");
    return;
  }

  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void DMA::Add(DigitalSource * /*input*/) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }

  tdma_config_->writeConfig_Enable_DI(true, &status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void DMA::Add(AnalogInput *input) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }

  if (input->GetChannel() <= 3) {
    tdma_config_->writeConfig_Enable_AI0_Low(true, &status);
  } else {
    tdma_config_->writeConfig_Enable_AI0_High(true, &status);
  }
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
}

void DMA::SetExternalTrigger(DigitalSource *input, bool rising, bool falling) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::SetExternalTrigger() only works before DMA::Start()");
    return;
  }

  auto index =
      ::std::find(trigger_channels_.begin(), trigger_channels_.end(), false);
  if (index == trigger_channels_.end()) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
                            "DMA: No channels left");
    return;
  }
  *index = true;

  const int channel_index = ::std::distance(trigger_channels_.begin(), index);

  const bool is_external_clock =
      tdma_config_->readConfig_ExternalClock(&status);
  if (status == 0) {
    if (!is_external_clock) {
      tdma_config_->writeConfig_ExternalClock(true, &status);
      wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
      if (status != 0) {
        return;
      }
    }
  } else {
    wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
    return;
  }

  nFPGA::nRoboRIO_FPGANamespace::tDMA::tExternalTriggers new_trigger;

  new_trigger.FallingEdge = falling;
  new_trigger.RisingEdge = rising;
  new_trigger.ExternalClockSource_AnalogTrigger = false;
  unsigned char module = 0;
  uint32_t channel =
#ifdef WPILIB2017
      input->GetChannel();
#else
      input->GetChannelForRouting();
#endif
  if (channel >= kNumHeaders) {
    module = 1;
    channel -= kNumHeaders;
  } else {
    module = 0;
  }

  new_trigger.ExternalClockSource_Module = module;
  new_trigger.ExternalClockSource_Channel = channel;

// Configures the trigger to be external, not off the FPGA clock.
  tdma_config_->writeExternalTriggers(channel_index / 4, channel_index % 4,
                                      new_trigger, &status);
  if (status != 0) {
    wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
    return;
  }
}

DMA::ReadStatus DMA::Read(DMASample *sample, uint32_t timeout_ms,
                          size_t *remaining_out) {
  tRioStatusCode status = 0;
  size_t remainingBytes = 0;
  *remaining_out = 0;

  if (!manager_.get()) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Read() only works after DMA::Start()");
    return STATUS_ERROR;
  }

  sample->dma_ = this;
  manager_->read(sample->read_buffer_, capture_size_, timeout_ms,
                 &remainingBytes, &status);

  // TODO(jerry): Do this only if status == 0?
  *remaining_out = remainingBytes / capture_size_;

  // TODO(austin): Check that *remainingBytes % capture_size_ == 0 and deal
  // with it if it isn't.  Probably meant that we overflowed?
  if (status == 0) {
    return STATUS_OK;
  } else if (status == NiFpga_Status_FifoTimeout) {
    return STATUS_TIMEOUT;
  } else {
    wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
    return STATUS_ERROR;
  }
}

const char *DMA::NameOfReadStatus(ReadStatus s) {
  switch (s) {
    case STATUS_OK:      return "OK";
    case STATUS_TIMEOUT: return "TIMEOUT";
    case STATUS_ERROR:   return "ERROR";
    default:             return "(bad ReadStatus code)";
  }
}

void DMA::Start(size_t queue_depth) {
  tRioStatusCode status = 0;
  tconfig_ = tdma_config_->readConfig(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }

  {
    size_t accum_size = 0;
#define SET_SIZE(bit)                      \
  if (tconfig_.bit) {                      \
    channel_offsets_[k##bit] = accum_size; \
    accum_size += kChannelSize[k##bit];    \
  } else {                                 \
    channel_offsets_[k##bit] = -1;         \
  }

    SET_SIZE(Enable_AI0_Low);
    SET_SIZE(Enable_AI0_High);
    SET_SIZE(Enable_AIAveraged0_Low);
    SET_SIZE(Enable_AIAveraged0_High);
    SET_SIZE(Enable_AI1_Low);
    SET_SIZE(Enable_AI1_High);
    SET_SIZE(Enable_AIAveraged1_Low);
    SET_SIZE(Enable_AIAveraged1_High);
    SET_SIZE(Enable_Accumulator0);
    SET_SIZE(Enable_Accumulator1);
    SET_SIZE(Enable_DI);
    SET_SIZE(Enable_AnalogTriggers);
    SET_SIZE(Enable_Counters_Low);
    SET_SIZE(Enable_Counters_High);
    SET_SIZE(Enable_CounterTimers_Low);
    SET_SIZE(Enable_CounterTimers_High);
    SET_SIZE(Enable_Encoders_Low);
    SET_SIZE(Enable_Encoders_High);
    SET_SIZE(Enable_EncoderTimers_Low);
    SET_SIZE(Enable_EncoderTimers_High);
#undef SET_SIZE
    capture_size_ = accum_size + 1;
  }

  manager_.reset(
      new nFPGA::tDMAManager(0, queue_depth * capture_size_, &status));

  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }
  // Start, stop, start to clear the buffer.
  manager_->start(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }
  manager_->stop(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }
  manager_->start(&status);
  wpi_setErrorWithContext(status, HAL_GetErrorMessage(status));
  if (status != 0) {
    return;
  }

}

static_assert(::std::is_pod<DMASample>::value, "DMASample needs to be POD");

ssize_t DMASample::offset(int index) const { return dma_->channel_offsets_[index]; }

uint32_t DMASample::GetTime() const {
  return read_buffer_[dma_->capture_size_ - 1];
}

double DMASample::GetTimestamp() const {
  return static_cast<double>(GetTime()) * 0.000001;
}

bool DMASample::Get(DigitalSource *input) const {
  if (offset(kEnable_DI) == -1) {
    wpi_setStaticErrorWithContext(
        dma_, NiFpga_Status_ResourceNotFound,
        HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
    return false;
  }
  const uint32_t channel =
#ifdef WPILIB2017
      input->GetChannel();
#else
      input->GetChannelForRouting();
#endif
  if (channel < kNumHeaders) {
    return (read_buffer_[offset(kEnable_DI)] >> channel) & 0x1;
  } else {
    return (read_buffer_[offset(kEnable_DI)] >> (channel + 6)) & 0x1;
  }
}

int32_t DMASample::GetRaw(Encoder *input) const {
  int index = input->GetFPGAIndex();
  uint32_t dmaWord = 0;
  if (index < 4) {
    if (offset(kEnable_Encoders_Low) == -1) {
      wpi_setStaticErrorWithContext(
          dma_, NiFpga_Status_ResourceNotFound,
          HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
      return -1;
    }
    dmaWord = read_buffer_[offset(kEnable_Encoders_Low) + index];
  } else if (index < 8) {
    if (offset(kEnable_Encoders_High) == -1) {
      wpi_setStaticErrorWithContext(
          dma_, NiFpga_Status_ResourceNotFound,
          HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
      return -1;
    }
    dmaWord = read_buffer_[offset(kEnable_Encoders_High) + (index - 4)];
  } else {
    wpi_setStaticErrorWithContext(
        dma_, NiFpga_Status_ResourceNotFound,
        HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
    return 0;
  }

  int32_t result = 0;

  // Extract the 31-bit signed tEncoder::tOutput Value using a struct with the
  // reverse packed field order of tOutput. This gets Value from the high
  // order 31 bits of output on little-endian ARM using gcc. This works
  // even though C/C++ doesn't guarantee bitfield order.
  t1Output output;

  output.value = dmaWord;
  result = output.Value;

  return result;
}

int32_t DMASample::Get(Encoder *input) const {
  int32_t raw = GetRaw(input);

  return raw / input->GetEncodingScale();
}

uint16_t DMASample::GetValue(AnalogInput *input) const {
  uint32_t channel = input->GetChannel();
  uint32_t dmaWord;
  if (channel < 4) {
    if (offset(kEnable_AI0_Low) == -1) {
      wpi_setStaticErrorWithContext(
          dma_, NiFpga_Status_ResourceNotFound,
          HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
      return 0xffff;
    }
    dmaWord = read_buffer_[offset(kEnable_AI0_Low) + channel / 2];
  } else if (channel < 8) {
    if (offset(kEnable_AI0_High) == -1) {
      wpi_setStaticErrorWithContext(
          dma_, NiFpga_Status_ResourceNotFound,
          HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
      return 0xffff;
    }
    dmaWord = read_buffer_[offset(kEnable_AI0_High) + (channel - 4) / 2];
  } else {
    wpi_setStaticErrorWithContext(
        dma_, NiFpga_Status_ResourceNotFound,
        HAL_GetErrorMessage(NiFpga_Status_ResourceNotFound));
    return 0xffff;
  }
  if (channel % 2) {
    return (dmaWord >> 16) & 0xffff;
  } else {
    return dmaWord & 0xffff;
  }
  return static_cast<int16_t>(dmaWord);
}

float DMASample::GetVoltage(AnalogInput *input) const {
  uint16_t value = GetValue(input);
  if (value == 0xffff) return 0.0;
  uint32_t lsb_weight = input->GetLSBWeight();
  int32_t offset = input->GetOffset();
  float voltage = lsb_weight * 1.0e-9 * value - offset * 1.0e-9;
  return voltage;
}
