#ifndef AOS_IPC_LIB_SHM_BASE_H_
#define AOS_IPC_LIB_SHM_BASE_H_

#include "gflags/gflags.h"

DECLARE_string(shm_base);

namespace aos::testing {
void SetShmBase(const std::string_view base);
}
#endif  // AOS_IPC_LIB_SHM_BASE_H_
