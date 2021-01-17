# RAWRTCDC

[![CircleCI build status][circleci-badge]][circleci-url]
[![Travis CI build status][travis-ci-badge]][travis-ci-url]
[![Join our chat on Gitter][gitter-icon]][gitter]

A standalone [WebRTC][w3c-webrtc] and [ORTC][w3c-ortc] data channel
implementation.

## Features

The following list represents all features that are planned for RAWRTCDC.
Features with a check mark are already implemented.

* [x] C-API based on the [W3C CG ORTC API][w3c-ortc]
* [x] SCTP-based data channels [[draft-ietf-rtcweb-data-channel-13]][sctp-dc]
  - [x] DCEP implementation [[draft-ietf-rtcweb-data-protocol-09]][dcep]
  - [x] Support for arbitrarily sized messages
  - [ ] Support for SCTP ndata [RFC 8260][sctp-ndata] (partially implemented,
        see [#14][#14])
  - [ ] Streaming mode (partially implemented, see [#16][#16])
  - [x] Hardware CRC32-C checksum offloading (requires SSE4.2)

## FAQ

1. *Who should use this?*

   If you have built a WebRTC stack and...
   
   * you want data channel support but you don't have it so far, or
   * you don't want to maintain your own data channel implementation, or
   * your data channel implementation is lacking some features,
   
   then you should consider integrating this library. :tada:

2. *But I also need ICE/DTLS!*

   Check out [RAWRTC][rawrtc].

3. *How does it work?*

   Basically, you pass in SCTP packets and you get out SCTP packets.
   Put that on top of your DTLS transport and you're ready to go.
   
   See the [*Getting Started*](#getting-started) section on how to set it up.

4. *Can I use it in an event loop?*

   Yes.

5. *Can I use it in a threaded environment?*

   Yes. Just make sure you're always calling it from the same thread the event
   loop is running on, or either [lock/unlock the event loop thread][re-lock]
   or [use the message queues provided by re][re-mqueue] to exchange data with
   the event loop thread. However, it is important that you only run one *re*
   event loop in one thread.

6. *Does it create threads?*

   No.

7. *Is this a custom SCTP implementation?*

   No, it uses [usrsctp](https://github.com/sctplab/usrsctp) underneath but
   handles all the nitty-gritty details for you.

## Prerequisites

The following tools are required:

* [git][git]
* [ninja][ninja] >= 1.5
* [meson][meson] >= 0.46.0
* Optional: pkg-config (`pkgconf` for newer FreeBSD versions)

## Build

```bash
cd <path-to-rawrtcdc>
mkdir build
meson build
cd build
ninja
```

## Getting Started

Now that you've built the library, let's get started integrating this library
into your stack.

### Initialise

Before doing anything, initialise the library:

```c
#include <rawrtcc.h>
#include <rawrtcdc.h>

[...]

enum rawrtc_code error = rawrtcdc_init(init_re, timer_handler);
if (error) {
    your_log_function("Initialisation failed: %s", rawrtc_code_to_str(error));
}
```

In the following code examples, the error handling will be omitted. But you of
course still need to handle it in your code.

Unless you're initialising [re][re] yourselves, the `init_re` parameter to
`rawrtcdc_init` should be `true`. The second is a pointer to a timer handler.

The timer handler function works in the following way (see comments inline):

```c
enum rawrtc_code timer_handler(bool const on, uint_fast16_t const interval) {
    if (on) {
        // Start a timer that calls `rawrtcdc_timer_tick` every `interval`
        // milliseconds.
    } else {
        // Stop the timer.
    }

    // Indicate success. In case something fails for you, you can also
    // backpropagate an appropriate error code here.
    return RAWRTC_CODE_SUCCESS;
}
```

### Create an SCTP transport

Before you can create data channels, you will need to create an SCTP transport:

```c
// Create SCTP transport context
struct rawrtc_sctp_transport_context context = {
    .role_getter = dtls_role_getter,
    .state_getter = dtls_transport_state_getter,
    .outbound_handler = sctp_transport_outbound_handler,
    .detach_handler = sctp_transport_detach_handler,
    .destroyed_handler = sctp_transport_destroy,
    .trace_packets = false,
    .arg = your_reference,
};

// Create SCTP transport
struct rawrtc_sctp_transport transport;
error = rawrtc_sctp_transport_create_from_external(
    &transport, &context, local_sctp_port,
    data_channel_handler, state_change_handler, arg);
if (error) {
    your_log_function("Creating SCTP transport failed: %s",
                      rawrtc_code_to_str(error));
    goto out;
}

// Attach your DTLS transport here
```

After the transport has been created successfully, `transport` will point to
some dynamically allocated memory which is reference counted (and the reference
counter value will be `1` after the function returned). If you want to increase
the reference, call `mem_ref(transport)`. If you need to decrease the
reference, call `mem_deref(transport)`. Once the counter value reaches `0`, it
will run a destructor function and free the memory. However, you should
normally stop the transport with `rawrtc_sctp_transport_stop` in a more
graceful manner before doing so. We're pointing this out here since basically
everything in this library that allocates dynamic memory works that way.

Furthermore, from this moment on your DTLS transport should feed SCTP packets
into the SCTP transport by calling
`rawrtc_sctp_transport_feed_inbound(transport, buffer, ecn_bits)`. Check the
[header file][sctp_transport.h] for details on the parameters.

You're probably already wondering what the SCTP transport context is all about.
Basically, it contains pointers to some handler functions you will need to
define. The only exception is the `arg` field which let's you pass an arbitrary
pointer to the various handler functions. So, let's go through them:

```c
enum rawrtc_code dtls_role_getter(
    enum rawrtc_external_dtls_role* const rolep, void* const arg) {
    // Set the local role of your DTLS transport
    *rolep = your_dtls_transport.local_role;
    return RAWRTC_CODE_SUCCESS;
}

enum rawrtc_code dtls_transport_state_getter(
    enum rawrtc_external_dtls_transport_state* const statep, void* const arg) {
    // Set the state of your DTLS transport
    *statep = your_dtls_transport.state;
    return RAWRTC_CODE_SUCCESS;
}
```

These DTLS handler functions were fairly straightforward. Now to the SCTP
handler that hands back outbound SCTP packets. These packets will need to be
fed into the DTLS transport as application data and sent to the other peer:

```c
enum rawrtc_code sctp_transport_outbound_handler(
    struct mbuf* const buffer, uint8_t const tos, uint8_t const set_df,
    void* const arg) {
    // Feed the data to the DTLS transport
    your_dtls_transport_send(buffer, tos, set_df);
    return RAWRTC_CODE_SUCCESS;
}
```

The `struct buffer` and its functions are documented [here][re-mbuf]. As a rule
of thumb, you should call `mbuf_buf(buffer)` to get a pointer to the current
position and `mbuf_get_left(buffer)` to get the amount of bytes left in the
buffer.
Be aware `buffer` in this case is not dynamically allocated and shall not be
referenced. This has been done for optimisation purposes.
Check the [header file][external.h] for further details on the various
parameters passed to this handler function.

The following handler is merely a way to let you know that you should not feed
any data to the SCTP transport anymore:

```c
void sctp_transport_detach_handler(void* const arg) {
    // Detach from DTLS transport
    your_dtls_transport.stop_feeding_data = true;
}
```

The last handler function we need to talk about is a way to tell you that the
SCTP transport's reference count has been decreased to `0` and its about to be
free'd. Be aware that you may not call any SCTP transport or data channel
functions once this handler is being called.

```c
void sctp_transport_destroy(void* const arg) {
    // Your cleanup code here
}
```

The `trace_packets` attribute allows you to enable writing SCTP packets to a
trace file. The name of that file is randomly generated and will be placed in
the current working directory.

That's all for the SCTP transport.

### Create a Data Channel

The data channel API is very similar to the one used in WebRTC and ORTC, so we
will not go into detail for them. Here's a quick example on how to create a
data channel with the following properties:

* label: `meow`
* protocol: `v1.cat-noises`
* reliable
* unordered
* pre-negotiated
* stream id is fixed to `42`

```c
// Create data channel parameters
struct rawrtc_data_channel_parameters parameters;
error = rawrtc_data_channel_parameters_create(
    &parameters, "meow", RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_UNORDERED, 0,
    "v1.cat-noises", true, 42);

// Create the data channel, using the transport and the parameters
struct rawrtc_data_channel channel;
error = rawrtc_data_channel_create(
    &channel, transport, parameters,
    open_handler, buffered_amount_low_handler, error_handler,
    close_handler, message_handler, pointer_passed_to_handlers);

mem_deref(parameters);
```

Instead of adding handler functions on creation, you can also pass `NULL` and
register handler functions later.

For further explanation on the various parameters, check the
[header files][headers].

Once the SCTP transport goes into the *connected* state, the created channels
will open. If you see this happening, this is a good indication that you've set
everything up correctly. :clap:

### Exit

Once your code exits, you should call `rawrtcdc_close(close_re)`. If the
`close_re` parameter is `true`, [re][re] will be closed as well.

### Any Questions?

![Draw The Rest Of The Owl Meme][owl-meme]

Do you feel like this now? If yes, please join our [gitter chat][gitter], so we
can help you and work out what's missing in this little tutorial.

## Contributing

When creating a pull request, it is recommended to run `format-all.sh` to
apply a consistent code style.



[circleci-badge]: https://circleci.com/gh/rawrtc/rawrtc-data-channel.svg?style=shield
[circleci-url]: https://circleci.com/gh/rawrtc/rawrtc-data-channel
[travis-ci-badge]: https://travis-ci.org/rawrtc/rawrtc-data-channel.svg?branch=master
[travis-ci-url]: https://travis-ci.org/rawrtc/rawrtc-data-channel

[gitter]: https://gitter.im/rawrtc/Lobby
[gitter-icon]: https://badges.gitter.im/rawrtc/Lobby.svg

[w3c-webrtc]: https://www.w3.org/TR/webrtc/
[w3c-ortc]: https://draft.ortc.org
[dcep]: https://tools.ietf.org/html/draft-ietf-rtcweb-data-protocol-09
[sctp-dc]: https://tools.ietf.org/html/draft-ietf-rtcweb-data-channel-13
[#14]: https://github.com/rawrtc/rawrtc-data-channel/issues/14
[#16]: https://github.com/rawrtc/rawrtc-data-channel/issues/16
[sctp-ndata]: https://tools.ietf.org/html/rfc8260

[rawrtc]: https://github.com/rawrtc/rawrtc

[git]: https://git-scm.com
[meson]: https://mesonbuild.com
[ninja]: https://ninja-build.org

[re-lock]: http://www.creytiv.com/doxygen/re-dox/html/re__main_8h.html#ad335fcaa56e36b39cb1192af1a6b9904
[re-mqueue]: http://www.creytiv.com/doxygen/re-dox/html/re__mqueue_8h.html
[usrsctp-neat-issue-12]: https://github.com/NEAT-project/usrsctp-neat/issues/12
[sctp_transport.h]: include/rawrtcdc/sctp_transport.h
[external.h]: include/rawrtcdc/external.h
[headers]: include/rawrtcdc
[re]: https://github.com/creytiv/re
[re-mbuf]: http://www.creytiv.com/doxygen/re-dox/html/re__mbuf_8h.html
[owl-meme]: ../assets/draw-the-rest-of-the-owl.jpg?raw=true
