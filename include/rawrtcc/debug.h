// Module level overwrites global level if present
#ifdef RAWRTC_DEBUG_MODULE_LEVEL
#    define DEBUG_LEVEL 0
#    if RAWRTC_DEBUG_MODULE_LEVEL == 1
#        define DEBUG_LEVEL 1
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 2
#        define DEBUG_LEVEL 2
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 3
#        define DEBUG_LEVEL 3
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 4
#        define DEBUG_LEVEL 4
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 5
#        define DEBUG_LEVEL 5
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 6
#        define DEBUG_LEVEL 6
#    elif RAWRTC_DEBUG_MODULE_LEVEL == 7
#        define DEBUG_LEVEL 7
#    endif
#else
#    ifndef RAWRTC_DEBUG_LEVEL
#        pragma message "RAWRTC_DEBUG_LEVEL is not defined!"
#    endif
#    define DEBUG_LEVEL RAWRTC_DEBUG_LEVEL
#endif

#include <re_dbg.h>

// Undef for next module
#undef RAWRTC_DEBUG_MODULE_LEVEL