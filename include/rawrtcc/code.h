#pragma once

/// Return codes.
///
/// To make it easy to test for errors, the *success* return code's
/// value will always be `0`. Therefore, you can test for errors
/// in the following way:
///
///     enum rawrtc_code const error = rawrtc_some_function();
///     if (error) {
///        // Handle the error...
///     }
///
/// **Important**: Add translations for new return codes in
/// `utils/utils.c`!
enum rawrtc_code {
    /// An unknown (or non-translatable) error occurred.
    RAWRTC_CODE_UNKNOWN_ERROR = -2,
    /// The necessary functionality has not been implemented.
    RAWRTC_CODE_NOT_IMPLEMENTED = -1,
    /// Success! Nothing went wrong - you're fine to proceed.
    RAWRTC_CODE_SUCCESS = 0,
    /// Initialisation failed.
    RAWRTC_CODE_INITIALISE_FAIL,
    /// Invalid argument.
    RAWRTC_CODE_INVALID_ARGUMENT,
    /// Memory could not be allocated.
    RAWRTC_CODE_NO_MEMORY,
    /// Invalid state.
    RAWRTC_CODE_INVALID_STATE,
    /// Unsupported protocol.
    RAWRTC_CODE_UNSUPPORTED_PROTOCOL,
    /// Unsupported algorithm.
    RAWRTC_CODE_UNSUPPORTED_ALGORITHM,
    /// No value has been set.
    /// @note This is often used for functions that change the value of
    ///       a variable declared outside of the function to indicate
    ///       that no change occurred.
    RAWRTC_CODE_NO_VALUE,
    /// Socket could not be found.
    RAWRTC_CODE_NO_SOCKET,
    /// Invalid certificate.
    RAWRTC_CODE_INVALID_CERTIFICATE,
    /// Invalid fingerprint.
    RAWRTC_CODE_INVALID_FINGERPRINT,
    /// Insufficient space.
    RAWRTC_CODE_INSUFFICIENT_SPACE,
    /// Target is still being used.
    RAWRTC_CODE_STILL_IN_USE,
    /// Invalid message.
    RAWRTC_CODE_INVALID_MESSAGE,
    /// Message is too long.
    RAWRTC_CODE_MESSAGE_TOO_LONG,
    /// Try again later.
    /// @note This is semantically equivalent to `EAGAIN` and
    ///       `EWOULDBLOCK`.
    RAWRTC_CODE_TRY_AGAIN_LATER,
    /// Stopped iterating (early).
    RAWRTC_CODE_STOP_ITERATION,
    /// Operation not permitted.
    RAWRTC_CODE_NOT_PERMITTED,
    /// An external function returned an error.
    RAWRTC_CODE_EXTERNAL_ERROR,
};
