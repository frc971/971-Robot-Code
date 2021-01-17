#include <rawrtcdc/data_channel.h>

/*
 * Get the corresponding name for a data channel state.
 */
char const* rawrtc_data_channel_state_to_name(enum rawrtc_data_channel_state const state) {
    switch (state) {
        case RAWRTC_DATA_CHANNEL_STATE_CONNECTING:
            return "connecting";
        case RAWRTC_DATA_CHANNEL_STATE_OPEN:
            return "open";
        case RAWRTC_DATA_CHANNEL_STATE_CLOSING:
            return "closing";
        case RAWRTC_DATA_CHANNEL_STATE_CLOSED:
            return "closed";
        default:
            return "???";
    }
}
