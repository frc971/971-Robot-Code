#ifndef AOS_IPC_LIB_MEMORY_ESTIMATION_H_
#define AOS_IPC_LIB_MEMORY_ESTIMATION_H_

#include "aos/configuration.h"

namespace aos::ipc_lib {
// Returns the total shared memory that will be used by the specified config on
// the specified node, in bytes.
size_t TotalSharedMemoryUsage(const aos::Configuration *config,
                              const aos::Node *node);
}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_MEMORY_ESTIMATION_H_
