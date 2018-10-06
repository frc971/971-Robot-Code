#ifndef MOTORS_CORE_SEMIHOSTING_H_
#define MOTORS_CORE_SEMIHOSTING_H_

// This file defines interfaces to the ARM semihosting functions.

#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace motors {
namespace semihosting {

inline uint32_t integer_operation(const uint32_t operation, void *const block) {
  register uint32_t operation_register asm("r0") = operation;
  register void *block_register asm("r1") = block;
  // Clobbering all the other registers here doesn't help the strange ways this
  // affects other code.
  __asm__ __volatile__("bkpt #0xab"
                       : "+r"(operation_register)
                       : "r"(block_register)
                       : "memory", "cc");
  return operation_register;
}

struct Close {
  // A handle for an open file.
  uint32_t handle;

  // Returns 0 if it succeeds, or -1 if not.
  int Execute() {
    return integer_operation(0x02, this);
  }
};

struct Errno {
  // Returns the current errno value.
  int Execute() {
    return integer_operation(0x13, nullptr);
  }
};

struct FileLength {
  // A handle for a previously opened seekable file.
  uint32_t handle;

  // Returns the current length of the file or -1.
  int Execute() {
    return integer_operation(0x0C, this);
  }
};

struct GetCommandLine {
  // Where the result is stored.
  // The data is guaranteed to be null-terminated, but it's unclear what happens
  // if the provided buffer is too short.
  char *buffer;
  uint32_t length;

  GetCommandLine() = default;
  GetCommandLine(gsl::span<char> buffer_in) {
    buffer = buffer_in.data();
    length = buffer_in.size();
  }

  // Returns 0 if it succeeds, or -1 if not.
  int Execute() {
    return integer_operation(0x15, this);
  }
};

struct IsTty {
  // A handle for a previously opened file.
  uint32_t handle;

  // Returns 0 if it's a file, 1 if it's an interactive device, or something
  // else otherwise.
  int Execute() {
    return integer_operation(0x09, this);
  }
};

struct Open {
  // The names are basically fopen arguments. + is translated to P.
  enum class Mode : uint32_t {
    kR = 0,
    kRB = 1,
    kRP = 2,
    kRPB = 3,
    kW = 4,
    kWB = 5,
    kWP = 6,
    kWPB = 7,
    kA = 8,
    kAB = 9,
    kAP = 10,
    kAPB = 11,
  };

  // A null-terminated file or device name.
  char *name;
  Mode mode;
  // Does not include the terminating null which must still be present.
  int name_length;

  // Returns a non-zero file handle if it succeeds, or -1 if not.
  int Execute() {
    return integer_operation(0x01, this);
  }
};

struct Read {
  // A handle for a previously opened file.
  uint32_t handle;
  char *buffer;
  uint32_t size;

  Read() = default;
  Read(uint32_t handle_in, gsl::span<char> buffer_in) {
    handle = handle_in;
    buffer = buffer_in.data();
    size = buffer_in.size();
  }

  // Returns the result. If this is empty, that means the file is at EOF.
  gsl::span<const char> Execute() {
    const uint32_t not_read = integer_operation(0x06, this);
    return gsl::span<const char>(buffer, size - not_read);
  }
};

struct ReadCharacter {
  char Execute() {
    return integer_operation(0x07, nullptr);
  }
};

struct Seek {
  // A handle for a previously opened file.
  uint32_t handle;
  // The absolute byte position to move to.
  uint32_t position;

  // Returns 0 if it succeeds, or a negative value if not.
  int Execute() {
    return integer_operation(0x0A, this);
  }
};

struct RealtimeTime {
  // Returns the number of seconds since 00:00 January 1, 1970.
  uint32_t Execute() {
    return integer_operation(0x11, nullptr);
  }
};

struct Write {
  // A handle for a previously opened file.
  // 1 and 2 for stdout/stderr seem to work too.
  uint32_t handle;
  const char *buffer;
  uint32_t length;

  Write() = default;
  Write(uint32_t handle_in, gsl::span<const char> buffer_in) {
    handle = handle_in;
    buffer = buffer_in.data();
    length = buffer_in.size();
  }

  // Returns 0 if it succeeds, or the number of bytes NOT written.
  int Execute() {
    return integer_operation(0x05, this);
  }
};

struct WriteDebugCharacter {
  const char *character;

  void Execute() {
    integer_operation(0x03, this);
  }
};

struct WriteDebug {
  // string must be null-terminated.
  void Execute(const char *string) {
    integer_operation(0x04, const_cast<char *>(string));
  }
};

}  // namespace semihosting
}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_CORE_SEMIHOSTING_H_
