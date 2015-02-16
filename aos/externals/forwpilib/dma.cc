#include "dma.h"

#include <string.h>

#include <algorithm>
#include <type_traits>

#include "DigitalSource.h"
#include "AnalogInput.h"
#include "Encoder.h"


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

static const uint32_t kNumHeaders = 10;

static constexpr ssize_t kChannelSize[18] = {2, 2, 4, 4, 2, 2, 4, 4, 3,
                                             3, 2, 1, 4, 4, 4, 4, 4, 4};

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
  kEnable_Encoders = 16,
  kEnable_EncoderTimers = 17,
};

DMA::DMA() {
  tRioStatusCode status = 0;
  tdma_config_ = tDMA::create(&status);
  tdma_config_->writeConfig_ExternalClock(false, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  NiFpga_WriteU32(0x10000, 0x1832c, 0x0);
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
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
}

void DMA::SetRate(uint32_t cycles) {
  if (cycles < 1) {
    cycles = 1;
  }
  tRioStatusCode status = 0;
  tdma_config_->writeRate(cycles, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
}

void DMA::Add(Encoder *encoder) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }
  if (encoder->GetFPGAIndex() >= 4) {
    wpi_setErrorWithContext(
        NiFpga_Status_InvalidParameter,
        "FPGA encoder index is not in the 4 that get logged.");
  }

  // TODO(austin): Encoder uses a Counter for 1x or 2x; quad for 4x...
  tdma_config_->writeConfig_Enable_Encoders(true, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
}

void DMA::Add(DigitalSource * /*input*/) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }

  tdma_config_->writeConfig_Enable_DI(true, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
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
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
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

  const int channel_index = index - trigger_channels_.begin();
  /*
  printf(
      "Allocating trigger on index %d, routing module %d, routing channel %d, "
      "is analog %d\n",
      channel_index, input->GetModuleForRouting(),
      input->GetChannelForRouting(), input->GetAnalogTriggerForRouting());
  */

  const bool is_external_clock =
      tdma_config_->readConfig_ExternalClock(&status);
  if (status == 0) {
    if (!is_external_clock) {
      tdma_config_->writeConfig_ExternalClock(true, &status);
      wpi_setErrorWithContext(status, getHALErrorMessage(status));
      if (status != 0) {
        return;
      }
    }
  } else {
    wpi_setErrorWithContext(status, getHALErrorMessage(status));
    return;
  }

  nFPGA::nFRC_2015_1_0_A::tDMA::tExternalTriggers new_trigger;

  new_trigger.FallingEdge = falling;
  new_trigger.RisingEdge = rising;
  new_trigger.ExternalClockSource_AnalogTrigger =
      input->GetAnalogTriggerForRouting();
  new_trigger.ExternalClockSource_AnalogTrigger = false;
  new_trigger.ExternalClockSource_Module = input->GetModuleForRouting();
  new_trigger.ExternalClockSource_Channel = input->GetChannelForRouting();

  // Configures the trigger to be external, not off the FPGA clock.
  /*
  tdma_config_->writeExternalTriggers(channel_index, new_trigger, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  */

  uint32_t current_triggers;
  tRioStatusCode register_status = NiFpga_ReadU32(0x10000, 0x1832c, &current_triggers);
  if (register_status != 0) {
    wpi_setErrorWithContext(register_status, getHALErrorMessage(status));
    return;
  }
  current_triggers = (current_triggers & ~(0xff << (channel_index * 8))) |
                     (new_trigger.value << (channel_index * 8));
  register_status = NiFpga_WriteU32(0x10000, 0x1832c, current_triggers);
  if (register_status != 0) {
    wpi_setErrorWithContext(register_status, getHALErrorMessage(status));
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
  // memset(&sample->read_buffer_, 0, sizeof(sample->read_buffer_));
  manager_->read(sample->read_buffer_, capture_size_, timeout_ms,
                 &remainingBytes, &status);

  if (0) { // DEBUG
    printf("buf[] = ");
    for (size_t i = 0;
         i < sizeof(sample->read_buffer_) / sizeof(sample->read_buffer_[0]);
         ++i) {
      if (i != 0) {
        printf(" ");
      }
      printf("0x%.8x", sample->read_buffer_[i]);
    }
    printf("\n");
  }

  // TODO(jerry): Do this only if status == 0?
  *remaining_out = remainingBytes / capture_size_;

  if (0) { // DEBUG
    printf("Remaining samples = %d\n", *remaining_out);
  }

  // TODO(austin): Check that *remainingBytes % capture_size_ == 0 and deal
  // with it if it isn't.  Probably meant that we overflowed?
  if (status == 0) {
    return STATUS_OK;
  } else if (status == NiFpga_Status_FifoTimeout) {
    return STATUS_TIMEOUT;
  } else {
    wpi_setErrorWithContext(status, getHALErrorMessage(status));
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
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
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
    SET_SIZE(Enable_Encoders);
    SET_SIZE(Enable_EncoderTimers);
#undef SET_SIZE
    capture_size_ = accum_size + 1;
  }

  manager_.reset(
      new nFPGA::tDMAManager(0, queue_depth * capture_size_, &status));

  if (0) {
    for (int i = 0; i < 4; ++i) {
      tRioStatusCode status = 0;
      auto x = tdma_config_->readExternalTriggers(i, &status);
      printf(
          "index %d, FallingEdge: %d, RisingEdge: %d, "
          "ExternalClockSource_AnalogTrigger: %d, ExternalClockSource_Module: "
          "%d, ExternalClockSource_Channel: %d\n",
          i, x.FallingEdge, x.RisingEdge, x.ExternalClockSource_AnalogTrigger,
          x.ExternalClockSource_Module, x.ExternalClockSource_Channel);
      if (status != 0) {
        wpi_setErrorWithContext(status, getHALErrorMessage(status));
      }
    }
  }

  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  // Start, stop, start to clear the buffer.
  manager_->start(&status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  manager_->stop(&status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  manager_->start(&status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }

}

static_assert(::std::is_pod<DMASample>::value, "DMASample needs to be POD");

ssize_t DMASample::offset(int index) const { return dma_->channel_offsets_[index]; }

double DMASample::GetTimestamp() const {
  return static_cast<double>(read_buffer_[dma_->capture_size_ - 1]) * 0.000001;
}

bool DMASample::Get(DigitalSource *input) const {
  if (offset(kEnable_DI) == -1) {
    wpi_setStaticErrorWithContext(dma_,
        NiFpga_Status_ResourceNotFound,
        getHALErrorMessage(NiFpga_Status_ResourceNotFound));
    return false;
  }
  if (input->GetChannelForRouting() < kNumHeaders) {
    return (read_buffer_[offset(kEnable_DI)] >>
            input->GetChannelForRouting()) &
           0x1;
  } else {
    return (read_buffer_[offset(kEnable_DI)] >>
            (input->GetChannelForRouting() + 6)) &
           0x1;
  }
}

int32_t DMASample::GetRaw(Encoder *input) const {
  if (offset(kEnable_Encoders) == -1) {
    wpi_setStaticErrorWithContext(dma_,
        NiFpga_Status_ResourceNotFound,
        getHALErrorMessage(NiFpga_Status_ResourceNotFound));
    return -1;
  }

  if (encoder->GetFPGAIndex() >= 4) {
    wpi_setStaticErrorWithContext(dma_,
        NiFpga_Status_ResourceNotFound,
        getHALErrorMessage(NiFpga_Status_ResourceNotFound));
  }

  uint32_t dmaWord =
      read_buffer_[offset(kEnable_Encoders) + input->GetFPGAIndex()];
  int32_t result = 0;

  if (1) {
    // Extract the 31-bit signed tEncoder::tOutput Value using a struct with the
    // reverse packed field order of tOutput. This gets Value from the high
    // order 31 bits of output on little-endian ARM using gcc. This works
    // even though C/C++ doesn't guarantee bitfield order.
    t1Output output;

    output.value = dmaWord;
    result = output.Value;
  } else if (1) {
    // Extract the 31-bit signed tEncoder::tOutput Value using right-shift.
    // This works even though C/C++ doesn't guarantee whether signed >> does
    // arithmetic or logical shift. (dmaWord / 2) would fix that but it rounds.
    result = static_cast<int32_t>(dmaWord) >> 1;
  }
#if 0  // This approach was recommended but it doesn't return the right value.
  else {
    // Byte-reverse the DMA word (big-endian value from the FPGA) then extract
    // the 31-bit tEncoder::tOutput. This does not return the right Value.
    tEncoder::tOutput encoderData;

    encoderData.value = __builtin_bswap32(dmaWord);
    result = encoderData.Value;
  }
#endif

  return result;
}

int32_t DMASample::Get(Encoder *input) const {
  int32_t raw = GetRaw(input);

  return raw / input->GetEncodingScale();
}

uint16_t DMASample::GetValue(AnalogInput *input) const {
  if (offset(kEnable_Encoders) == -1) {
    wpi_setStaticErrorWithContext(dma_,
        NiFpga_Status_ResourceNotFound,
        getHALErrorMessage(NiFpga_Status_ResourceNotFound));
    return 0xffff;
  }

  uint32_t dmaWord;
  uint32_t channel = input->GetChannel();
  if (input->GetChannel() <= 3) {
    dmaWord = read_buffer_[offset(kEnable_AI0_Low) + channel / 2];
  } else {
    dmaWord = read_buffer_[offset(kEnable_AI0_High) + (channel - 4) / 2];
  }
  if (channel % 1) {
    return (dmaWord >> 16) & 0xffff;
  } else {
    return (dmaWord) & 0xffff;
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
