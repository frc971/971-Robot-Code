#ifndef AOS_IPC_LIB_SHM_BASE_H_
#define AOS_IPC_LIB_SHM_BASE_H_

#include <string_view>

#include "absl/flags/declare.h"

ABSL_DECLARE_FLAG(std::string, shm_base);

namespace aos::testing {
void SetShmBase(const std::string_view base);
}

#endif  // AOS_IPC_LIB_SHM_BASE_H_
