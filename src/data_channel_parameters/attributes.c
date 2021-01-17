#include "parameters.h"
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/data_channel_parameters.h>
#include <rawrtcc/code.h>
#include <re.h>

/*
 * Get the label from the data channel parameters.
 * `*labelp` will be set to a copy of the parameter's label and must be
 * unreferenced.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case no label has been set.
 * Otherwise, `RAWRTC_CODE_SUCCESS` will be returned and `*parameters*
 * must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_label(
    char** const labelp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!labelp || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    if (parameters->label) {
        *labelp = mem_ref(parameters->label);
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Get the channel type from the data channel parameters.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_channel_type(
    enum rawrtc_data_channel_type* const channel_typep,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!channel_typep || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    *channel_typep = parameters->channel_type;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the reliability parameter from the data channel parameters.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case the channel type is
 * `RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_*`. Otherwise,
 * `RAWRTC_CODE_SUCCESS` will be returned.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_reliability_parameter(
    uint32_t* const reliability_parameterp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!reliability_parameterp || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    switch (parameters->channel_type) {
        case RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_ORDERED:
        case RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_UNORDERED:
            return RAWRTC_CODE_NO_VALUE;
        default:
            *reliability_parameterp = parameters->reliability_parameter;
            return RAWRTC_CODE_SUCCESS;
    }
}

/*
 * Get the protocol from the data channel parameters.
 * `*protocolp` will be set to a copy of the parameter's protocol and
 * must be unreferenced.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case no protocol has been set.
 * Otherwise, `RAWRTC_CODE_SUCCESS` will be returned and `*protocolp*
 * must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_protocol(
    char** const protocolp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!protocolp || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    if (parameters->protocol) {
        *protocolp = mem_ref(parameters->protocol);
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Get the 'negotiated' flag from the data channel parameters.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_negotiated(
    bool* const negotiatedp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!negotiatedp || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    *negotiatedp = parameters->negotiated;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the negotiated id from the data channel parameters.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case the 'negotiated' flag is set
 * `false`. Otherwise, `RAWRTC_CODE_SUCCESS` will be returned.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_id(
    uint16_t* const idp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters) {
    // Check arguments
    if (!idp || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    if (parameters->negotiated) {
        *idp = parameters->id;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}
