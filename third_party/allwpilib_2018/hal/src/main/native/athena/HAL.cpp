/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "HAL/HAL.h"

#include <signal.h>  // linux for kill
#include <sys/prctl.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <thread>

#include <FRC_NetworkCommunication/FRCComm.h>
#include <FRC_NetworkCommunication/LoadOut.h>
#include <llvm/raw_ostream.h>
#include <support/mutex.h>
#include <support/timestamp.h>

#include "HAL/ChipObject.h"
#include "HAL/DriverStation.h"
#include "HAL/Errors.h"
#include "HAL/Notifier.h"
#include "HAL/handles/HandlesInternal.h"
#include "HALInitializer.h"
#include "ctre/ctre.h"

using namespace hal;

static std::unique_ptr<tGlobal> global;
static std::unique_ptr<tSysWatchdog> watchdog;

using namespace hal;

namespace hal {
namespace init {
void InitializeHAL() {
  InitializeHandlesInternal();
  InitializeAccelerometer();
  InitializeAnalogAccumulator();
  InitializeAnalogInput();
  InitializeAnalogInternal();
  InitializeAnalogOutput();
  InitializeAnalogTrigger();
  InitializeCAN();
  InitializeCompressor();
  InitializeConstants();
  InitializeCounter();
  InitializeDigitalInternal();
  InitializeDIO();
  InitializeEncoder();
  InitializeFPGAEncoder();
  InitializeFRCDriverStation();
  InitializeI2C();
  InitialzeInterrupts();
  InitializeNotifier();
  InitializePCMInternal();
  InitializePDP();
  InitializePorts();
  InitializePower();
  InitializePWM();
  InitializeRelay();
  InitializeSolenoid();
  InitializeSPI();
  InitializeThreads();
}
}  // namespace init
}  // namespace hal

extern "C" {

HAL_PortHandle HAL_GetPort(int32_t channel) {
  // Dont allow a number that wouldn't fit in a uint8_t
  if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
  return createPortHandle(channel, 1);
}

/**
 * @deprecated Uses module numbers
 */
HAL_PortHandle HAL_GetPortWithModule(int32_t module, int32_t channel) {
  // Dont allow a number that wouldn't fit in a uint8_t
  if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
  if (module < 0 || module >= 255) return HAL_kInvalidHandle;
  return createPortHandle(channel, module);
}

const char* HAL_GetErrorMessage(int32_t code) {
  switch (code) {
    case 0:
      return "";
    case CTR_RxTimeout:
      return CTR_RxTimeout_MESSAGE;
    case CTR_TxTimeout:
      return CTR_TxTimeout_MESSAGE;
    case CTR_InvalidParamValue:
      return CTR_InvalidParamValue_MESSAGE;
    case CTR_UnexpectedArbId:
      return CTR_UnexpectedArbId_MESSAGE;
    case CTR_TxFailed:
      return CTR_TxFailed_MESSAGE;
    case CTR_SigNotUpdated:
      return CTR_SigNotUpdated_MESSAGE;
    case NiFpga_Status_FifoTimeout:
      return NiFpga_Status_FifoTimeout_MESSAGE;
    case NiFpga_Status_TransferAborted:
      return NiFpga_Status_TransferAborted_MESSAGE;
    case NiFpga_Status_MemoryFull:
      return NiFpga_Status_MemoryFull_MESSAGE;
    case NiFpga_Status_SoftwareFault:
      return NiFpga_Status_SoftwareFault_MESSAGE;
    case NiFpga_Status_InvalidParameter:
      return NiFpga_Status_InvalidParameter_MESSAGE;
    case NiFpga_Status_ResourceNotFound:
      return NiFpga_Status_ResourceNotFound_MESSAGE;
    case NiFpga_Status_ResourceNotInitialized:
      return NiFpga_Status_ResourceNotInitialized_MESSAGE;
    case NiFpga_Status_HardwareFault:
      return NiFpga_Status_HardwareFault_MESSAGE;
    case NiFpga_Status_IrqTimeout:
      return NiFpga_Status_IrqTimeout_MESSAGE;
    case SAMPLE_RATE_TOO_HIGH:
      return SAMPLE_RATE_TOO_HIGH_MESSAGE;
    case VOLTAGE_OUT_OF_RANGE:
      return VOLTAGE_OUT_OF_RANGE_MESSAGE;
    case LOOP_TIMING_ERROR:
      return LOOP_TIMING_ERROR_MESSAGE;
    case SPI_WRITE_NO_MOSI:
      return SPI_WRITE_NO_MOSI_MESSAGE;
    case SPI_READ_NO_MISO:
      return SPI_READ_NO_MISO_MESSAGE;
    case SPI_READ_NO_DATA:
      return SPI_READ_NO_DATA_MESSAGE;
    case INCOMPATIBLE_STATE:
      return INCOMPATIBLE_STATE_MESSAGE;
    case NO_AVAILABLE_RESOURCES:
      return NO_AVAILABLE_RESOURCES_MESSAGE;
    case RESOURCE_IS_ALLOCATED:
      return RESOURCE_IS_ALLOCATED_MESSAGE;
    case RESOURCE_OUT_OF_RANGE:
      return RESOURCE_OUT_OF_RANGE_MESSAGE;
    case HAL_INVALID_ACCUMULATOR_CHANNEL:
      return HAL_INVALID_ACCUMULATOR_CHANNEL_MESSAGE;
    case HAL_HANDLE_ERROR:
      return HAL_HANDLE_ERROR_MESSAGE;
    case NULL_PARAMETER:
      return NULL_PARAMETER_MESSAGE;
    case ANALOG_TRIGGER_LIMIT_ORDER_ERROR:
      return ANALOG_TRIGGER_LIMIT_ORDER_ERROR_MESSAGE;
    case ANALOG_TRIGGER_PULSE_OUTPUT_ERROR:
      return ANALOG_TRIGGER_PULSE_OUTPUT_ERROR_MESSAGE;
    case PARAMETER_OUT_OF_RANGE:
      return PARAMETER_OUT_OF_RANGE_MESSAGE;
    case HAL_COUNTER_NOT_SUPPORTED:
      return HAL_COUNTER_NOT_SUPPORTED_MESSAGE;
    case HAL_ERR_CANSessionMux_InvalidBuffer:
      return ERR_CANSessionMux_InvalidBuffer_MESSAGE;
    case HAL_ERR_CANSessionMux_MessageNotFound:
      return ERR_CANSessionMux_MessageNotFound_MESSAGE;
    case HAL_WARN_CANSessionMux_NoToken:
      return WARN_CANSessionMux_NoToken_MESSAGE;
    case HAL_ERR_CANSessionMux_NotAllowed:
      return ERR_CANSessionMux_NotAllowed_MESSAGE;
    case HAL_ERR_CANSessionMux_NotInitialized:
      return ERR_CANSessionMux_NotInitialized_MESSAGE;
    case HAL_PWM_SCALE_ERROR:
      return HAL_PWM_SCALE_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_NOT_FOUND:
      return HAL_SERIAL_PORT_NOT_FOUND_MESSAGE;
    case HAL_THREAD_PRIORITY_ERROR:
      return HAL_THREAD_PRIORITY_ERROR_MESSAGE;
    case HAL_THREAD_PRIORITY_RANGE_ERROR:
      return HAL_THREAD_PRIORITY_RANGE_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_OPEN_ERROR:
      return HAL_SERIAL_PORT_OPEN_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_ERROR:
      return HAL_SERIAL_PORT_ERROR_MESSAGE;
    default:
      return "Unknown error status";
  }
}

/**
 * Returns the runtime type of this HAL
 */
HAL_RuntimeType HAL_GetRuntimeType(void) { return HAL_Athena; }

/**
 * Return the FPGA Version number.
 * For now, expect this to be competition year.
 * @return FPGA Version number.
 */
int32_t HAL_GetFPGAVersion(int32_t* status) {
  if (!global) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return 0;
  }
  return global->readVersion(status);
}

/**
 * Return the FPGA Revision number.
 * The format of the revision is 3 numbers.
 * The 12 most significant bits are the Major Revision.
 * the next 8 bits are the Minor Revision.
 * The 12 least significant bits are the Build Number.
 * @return FPGA Revision number.
 */
int64_t HAL_GetFPGARevision(int32_t* status) {
  if (!global) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return 0;
  }
  return global->readRevision(status);
}

/**
 * Read the microsecond-resolution timer on the FPGA.
 *
 * @return The current time in microseconds according to the FPGA (since FPGA
 * reset).
 */
uint64_t HAL_GetFPGATime(int32_t* status) {
  if (!global) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return 0;
  }

  uint64_t upper1 = global->readLocalTimeUpper(status);
  uint32_t lower = global->readLocalTime(status);
  uint64_t upper2 = global->readLocalTimeUpper(status);
  if (*status != 0) return 0;
  if (upper1 != upper2) {
    // Rolled over between the lower call, reread lower
    lower = global->readLocalTime(status);
    if (*status != 0) return 0;
  }
  return (upper2 << 32) + lower;
}

/**
 * Convert a freestanding lower half to a 64 bit FPGA timestamp
 *
 * Note: This is making the assumption that the timestamp being converted is
 * always in the past.  If you call this with a future timestamp, it probably
 * will make it in the past.  If you wait over 70 minutes between capturing the
 * bottom 32 bits of the timestamp and expanding it, you will be off by
 * multiples of 1<<32 microseconds.
 *
 * @return The current time in microseconds according to the FPGA (since FPGA
 * reset) as a 64 bit number.
 */
uint64_t HAL_ExpandFPGATime(uint32_t unexpanded_lower, int32_t* status) {
  // Capture the current FPGA time.  This will give us the upper half of the
  // clock.
  uint64_t fpga_time = HAL_GetFPGATime(status);
  if (*status != 0) return 0;

  // Now, we need to detect the case where the lower bits rolled over after we
  // sampled.  In that case, the upper bits will be 1 bigger than they should
  // be.

  // Break it into lower and upper portions.
  uint32_t lower = fpga_time & ((uint64_t)0xffffffff);
  uint64_t upper = (fpga_time >> 32) & 0xffffffff;

  // The time was sampled *before* the current time, so roll it back.
  if (lower < unexpanded_lower) {
    --upper;
  }

  return (upper << 32) + static_cast<uint64_t>(unexpanded_lower);
}

/**
 * Get the state of the "USER" button on the roboRIO
 * @return true if the button is currently pressed down
 */
HAL_Bool HAL_GetFPGAButton(int32_t* status) {
  if (!global) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return false;
  }
  return global->readUserButton(status);
}

HAL_Bool HAL_GetSystemActive(int32_t* status) {
  if (!watchdog) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return false;
  }
  return watchdog->readStatus_SystemActive(status);
}

HAL_Bool HAL_GetBrownedOut(int32_t* status) {
  if (!watchdog) {
    *status = NiFpga_Status_ResourceNotInitialized;
    return false;
  }
  return !(watchdog->readStatus_PowerAlive(status));
}

void HAL_BaseInitialize(int32_t* status) {
  static std::atomic_bool initialized{false};
  static wpi::mutex initializeMutex;
  // Initial check, as if it's true initialization has finished
  if (initialized) return;

  std::lock_guard<wpi::mutex> lock(initializeMutex);
  // Second check in case another thread was waiting
  if (initialized) return;
  // image 4; Fixes errors caused by multiple processes. Talk to NI about this
  nFPGA::nRoboRIO_FPGANamespace::g_currentTargetClass =
      nLoadOut::kTargetClass_RoboRIO;

  global.reset(tGlobal::create(status));
  watchdog.reset(tSysWatchdog::create(status));
  initialized = true;
}

static bool killExistingProgram(int timeout, int mode) {
  // Kill any previous robot programs
  std::fstream fs;
  // By making this both in/out, it won't give us an error if it doesnt exist
  fs.open("/var/lock/frc.pid", std::fstream::in | std::fstream::out);
  if (fs.bad()) return false;

  pid_t pid = 0;
  if (!fs.eof() && !fs.fail()) {
    fs >> pid;
    // see if the pid is around, but we don't want to mess with init id=1, or
    // ourselves
    if (pid >= 2 && kill(pid, 0) == 0 && pid != getpid()) {
      llvm::outs() << "Killing previously running FRC program...\n";
      kill(pid, SIGTERM);  // try to kill it
      std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
      if (kill(pid, 0) == 0) {
        // still not successfull
        if (mode == 0) {
          llvm::outs() << "FRC pid " << pid << " did not die within " << timeout
                       << "ms. Aborting\n";
          return 0;              // just fail
        } else if (mode == 1) {  // kill -9 it
          kill(pid, SIGKILL);
        } else {
          llvm::outs() << "WARNING: FRC pid " << pid << " did not die within "
                       << timeout << "ms.\n";
        }
      }
    }
  }
  fs.close();
  // we will re-open it write only to truncate the file
  fs.open("/var/lock/frc.pid", std::fstream::out | std::fstream::trunc);
  fs.seekp(0);
  pid = getpid();
  fs << pid << std::endl;
  fs.close();
  return true;
}

/**
 * Call this to start up HAL. This is required for robot programs.
 */
HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode) {
  static std::atomic_bool initialized{false};
  static wpi::mutex initializeMutex;
  // Initial check, as if it's true initialization has finished
  if (initialized) return true;

  std::lock_guard<wpi::mutex> lock(initializeMutex);
  // Second check in case another thread was waiting
  if (initialized) return true;

  hal::init::InitializeHAL();

  setlinebuf(stdin);
  setlinebuf(stdout);
  llvm::outs().SetUnbuffered();

  prctl(PR_SET_PDEATHSIG, SIGTERM);

  // Return false if program failed to kill an existing program
  if (!killExistingProgram(timeout, mode)) {
    return false;
  }

  FRC_NetworkCommunication_Reserve(nullptr);

  std::atexit([]() {
    // Unregister our new data condition variable.
    setNewDataSem(nullptr);
  });

  int32_t status = 0;
  HAL_BaseInitialize(&status);
  if (status != 0) return false;

  HAL_InitializeDriverStation();

  // Set WPI_Now to use FPGA timestamp
  wpi::SetNowImpl([]() -> uint64_t {
    int32_t status = 0;
    uint64_t rv = HAL_GetFPGATime(&status);
    if (status != 0) {
      llvm::errs()
          << "Call to HAL_GetFPGATime failed."
          << "Initialization might have failed. Time will not be correct\n";
      llvm::errs().flush();
      return 0u;
    }
    return rv;
  });

  initialized = true;
  return true;
}

int64_t HAL_Report(int32_t resource, int32_t instanceNumber, int32_t context,
                   const char* feature) {
  if (feature == nullptr) {
    feature = "";
  }

  return FRC_NetworkCommunication_nUsageReporting_report(
      resource, instanceNumber, context, feature);
}

// TODO: HACKS
// No need for header definitions, as we should not run from user code.
void NumericArrayResize(void) {}
void RTSetCleanupProc(void) {}
void EDVR_CreateReference(void) {}

}  // extern "C"
