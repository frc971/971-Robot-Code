#include "dma.h"

#include <algorithm>

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
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
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

void DMA::Add(Encoder * /*encoder*/) {
  tRioStatusCode status = 0;

  if (manager_) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Add() only works before DMA::Start()");
    return;
  }

  fprintf(stderr, "DMA::Add(Encoder*) needs re-testing. aborting\n");
  abort();

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

  tdma_config_->writeConfig_ExternalClock(true, &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }

  // Configures the trigger to be external, not off the FPGA clock.
  tdma_config_->writeExternalTriggers_ExternalClockSource_Channel(
      channel_index, input->GetChannelForRouting(), &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }

  tdma_config_->writeExternalTriggers_ExternalClockSource_Module(
      channel_index, input->GetModuleForRouting(), &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  tdma_config_->writeExternalTriggers_ExternalClockSource_AnalogTrigger(
      channel_index, input->GetAnalogTriggerForRouting(), &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  tdma_config_->writeExternalTriggers_RisingEdge(channel_index, rising,
                                                 &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
  tdma_config_->writeExternalTriggers_FallingEdge(channel_index, falling,
                                                  &status);
  wpi_setErrorWithContext(status, getHALErrorMessage(status));
  if (status != 0) {
    return;
  }
}

DMA::ReadStatus DMA::Read(DMASample *sample, uint32_t timeout_ms,
                          size_t *remaining) {
  tRioStatusCode status = 0;
  size_t remainingBytes = 0;
  *remaining = 0;

  if (!manager_.get()) {
    wpi_setErrorWithContext(NiFpga_Status_InvalidParameter,
        "DMA::Read() only works after DMA::Start()");
    return STATUS_ERROR;
  }

  // memset(&sample->read_buffer_, 0, sizeof(read_buffer_));
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
  *remaining = remainingBytes / capture_size_;
  sample->dma_ = this;

  if (0) { // DEBUG
    printf("Remaining samples = %d\n", *remaining);
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

  manager_.reset(new nFPGA::tDMAManager(0, queue_depth * capture_size_, &status));
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
    // arithmetic or logical shift. (dmaWord / 2) is not a great alternative
    // since it rounds.
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

  // TODO(austin): Really bad...  DecodingScaleFactor?
  return raw / 4.0;
}
