#pragma once
#include <rawrtcdc/data_channel.h>
#include <rawrtcc/code.h>
#include <re.h>

/**
 * Data channel parameters.
 */
struct rawrtc_data_channel_parameters {
    char* label;  // copied
    enum rawrtc_data_channel_type channel_type;
    uint32_t reliability_parameter;  // contains either max_packet_lifetime or max_retransmit
    char* protocol;  // copied
    bool negotiated;
    uint16_t id;
};

enum rawrtc_code rawrtc_data_channel_parameters_create_internal(
    struct rawrtc_data_channel_parameters** const parametersp,  // de-referenced
    char* const label,  // referenced, nullable
    enum rawrtc_data_channel_type const channel_type,
    uint32_t const reliability_parameter,
    char* const protocol,  // referenced, nullable
    bool const negotiated,
    uint16_t const id);
