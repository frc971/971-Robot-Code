#ifndef _AOS_CORE_H_
#define _AOS_CORE_H_

#ifdef __VXWORKS__
#include "WPILib/Task.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __VXWORKS__
#include "aos/atom_code/ipc_lib/resource.h"
#include "aos/atom_code/ipc_lib/shared_mem.h"
#include "aos/atom_code/ipc_lib/queue.h"
#ifdef __cplusplus
}
#include "aos/atom_code/init.h"
extern "C" {
#endif

// A macro that runs a control loop using AOS_RUN on linux, and lets the user
// register the control loop on the cRIO.
#define AOS_RUN_LOOP AOS_RUN

// A macro that will create an instance of the given class using
// its default constructor and call Run() on it on the cRIO or the atom.
// It will generate a main on the atom and a function that the build system links
// in a call to on the cRIO.
// NOTE: No ; after this macro, and it should be used at the file scope (NO NAMESPACES!).
#define AOS_RUN(classname) int main() { \
  aos::Init(); \
  classname looper; /* on the stack to avoid eigen alignment issue */ \
  looper.Run(); \
  aos::Cleanup(); \
}
// Same as AOS_RUN, except uses aos::init_nrt instead of aos::init.
#define AOS_RUN_NRT(classname) int main() { \
  aos::InitNRT(); \
  classname looper; /* on the stack to avoid eigen alignment issue */ \
  looper.Run(); \
  aos::Cleanup(); \
}
// Same as AOS_RUN, except passes args to the constructor.
#define AOS_RUN_ARGS(classname, args...) int () { \
  aos::Init(); \
  classname looper(args); /* on the stack to avoid eigen alignment issue */ \
  looper.Run(); \
  aos::Cleanup(); \
}

#else // ifndef __VXWORKS__

// The cRIO doesn't need to run the Run method.  The cRIO main should register
// all loops to get run.
#define AOS_RUN_LOOP(classname) /* nop */

#define AOS_RUN(classname) extern "C" void AOS_INITNAME() { \
  classname *looper = new classname(); /* dynamically allocated because most (all?) Run functions store references */ \
  looper->Run(); \
}
// Same as AOS_RUN, except it runs in a new Task (named name and at priority).
// Calls both the constructor and Run in the new Task.
#define AOS_RUN_FORK(classname, name, priority) \
static void aos_init_(void *) { \
  classname *looper = new classname(); /* dynamically allocated because most (all?) Run functions store references */ \
  looper->Run(); \
} \
extern "C" void AOS_INITNAME() { \
  (new Task(name, reinterpret_cast<FUNCPTR>(aos_init_), priority))->Start(); \
}

#endif // ifndef __VXWORKS__

#ifdef __cplusplus
}
#endif

#include "aos/common/logging/logging.h"

#endif
