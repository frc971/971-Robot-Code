#include <rawrtcc/code.h>
#include <rawrtcc/utils.h>
#include <re.h>
#include <stdarg.h>  // va_*

/*
 * Translate a rawrtc return code to a string.
 */
char const* rawrtc_code_to_str(enum rawrtc_code const code) {
    switch (code) {
        case RAWRTC_CODE_UNKNOWN_ERROR:
            return "unknown error";
        case RAWRTC_CODE_NOT_IMPLEMENTED:
            return "not implemented";
        case RAWRTC_CODE_SUCCESS:
            return "success";
        case RAWRTC_CODE_INITIALISE_FAIL:
            return "failed to initialise";
        case RAWRTC_CODE_INVALID_ARGUMENT:
            return "invalid argument";
        case RAWRTC_CODE_NO_MEMORY:
            return "no memory";
        case RAWRTC_CODE_INVALID_STATE:
            return "invalid state";
        case RAWRTC_CODE_UNSUPPORTED_PROTOCOL:
            return "unsupported protocol";
        case RAWRTC_CODE_UNSUPPORTED_ALGORITHM:
            return "unsupported algorithm";
        case RAWRTC_CODE_NO_VALUE:
            return "no value";
        case RAWRTC_CODE_NO_SOCKET:
            return "no socket";
        case RAWRTC_CODE_INVALID_CERTIFICATE:
            return "invalid certificate";
        case RAWRTC_CODE_INVALID_FINGERPRINT:
            return "invalid fingerprint";
        case RAWRTC_CODE_INSUFFICIENT_SPACE:
            return "insufficient space";
        case RAWRTC_CODE_STILL_IN_USE:
            return "still in use";
        case RAWRTC_CODE_INVALID_MESSAGE:
            return "invalid message";
        case RAWRTC_CODE_MESSAGE_TOO_LONG:
            return "message too long";
        case RAWRTC_CODE_TRY_AGAIN_LATER:
            return "try again later";
        case RAWRTC_CODE_STOP_ITERATION:
            return "stop iteration";
        case RAWRTC_CODE_NOT_PERMITTED:
            return "not permitted";
        case RAWRTC_CODE_EXTERNAL_ERROR:
            return "external callback error-ed";
        default:
            return "(no error translation)";
    }
}

/*
 * Translate an re error to a rawrtc code.
 * TODO: Add codes from trice_lcand_add
 */
enum rawrtc_code rawrtc_error_to_code(int const code) {
    switch (code) {
        case 0:
            return RAWRTC_CODE_SUCCESS;
        case EAGAIN:
#if (EAGAIN != EWOULDBLOCK)
        case EWOULDBLOCK:
#endif
            return RAWRTC_CODE_TRY_AGAIN_LATER;
        case EAUTH:
            return RAWRTC_CODE_INVALID_CERTIFICATE;
        case EBADMSG:
            return RAWRTC_CODE_INVALID_MESSAGE;
        case EINVAL:
            return RAWRTC_CODE_INVALID_ARGUMENT;
        case EMSGSIZE:
            return RAWRTC_CODE_MESSAGE_TOO_LONG;
        case ENOMEM:
            return RAWRTC_CODE_NO_MEMORY;
        case EPERM:
            return RAWRTC_CODE_NOT_PERMITTED;
        default:
            return RAWRTC_CODE_UNKNOWN_ERROR;
    }
}

/*
 * Duplicate a string.
 * `*destinationp` will be set to a copy of `source` and must be
 * unreferenced.
 */
enum rawrtc_code rawrtc_strdup(char** const destinationp, char const* const source) {
    int err = str_dup(destinationp, source);
    return rawrtc_error_to_code(err);
}

/*
 * Print a formatted string to a dynamically allocated buffer.
 * `*destinationp` must be unreferenced.
 */
enum rawrtc_code rawrtc_sdprintf(char** const destinationp, char* const formatter, ...) {
    int err;
    va_list args;
    va_start(args, formatter);
    err = re_vsdprintf(destinationp, formatter, args);
    va_end(args);
    return rawrtc_error_to_code(err);
}
