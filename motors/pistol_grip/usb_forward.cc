// This file is designed to be compiled for Windows to run on the driver's
// station. It forwards UDP packets to the pistol grip controller over USB.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>

#ifdef __WINNT__
#include <winsock2.h>

#define PRId8 "hhd"
#define PRIx8 "hhx"
#else
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include <string>

#include "libusb-1.0/libusb.h"

#define CHECK_LIBUSB(expression)                                   \
  ({                                                               \
    const int result = (expression);                               \
    if (result < 0) {                                              \
      fprintf(stderr, #expression " failed with %d: %s\n", result, \
              libusb_error_name(result));                          \
      abort();                                                     \
    }                                                              \
    result;                                                        \
  })

#define CHECK(expression)                       \
  do {                                          \
    if (!(expression)) {                        \
      fprintf(stderr, #expression " failed\n"); \
      abort();                                  \
    }                                           \
  } while (false)

#define PCHECK(expression)                                         \
  ({                                                               \
    const int result = (expression);                               \
    if (result == -1) {                                            \
      fprintf(stderr, #expression " failed with %d: %s\n", result, \
              strerror(result));                                   \
      abort();                                                     \
    }                                                              \
    result;                                                        \
  })

namespace {

void OpenDevice(libusb_device_handle **handle) {
  libusb_device **devices;
  const ssize_t count = CHECK_LIBUSB(libusb_get_device_list(nullptr, &devices));
  for (ssize_t i = 0; i < count; ++i) {
    libusb_device_descriptor device_descriptor;
    libusb_device *const device = devices[i];
    CHECK_LIBUSB(libusb_get_device_descriptor(device, &device_descriptor));
    if (device_descriptor.idVendor == 0x16C0 &&
        device_descriptor.idProduct == 0x0491) {
      // TODO(Brian): Check for the right interface too, instead of relying on
      // us choosing different PIDs for each kind of Teensy.
      fprintf(stderr, "Found device at %" PRId8 ":%" PRId8 "\n",
              libusb_get_bus_number(device), libusb_get_port_number(device));
      CHECK_LIBUSB(libusb_open(device, handle));
      libusb_free_device_list(devices, true);
      return;
    }
  }
  fprintf(stderr, "Could not find device\n");
  abort();
}

void FindEndpoint(libusb_device_handle *handle, uint8_t *endpoint) {
  libusb_device *const device = libusb_get_device(handle);

  libusb_config_descriptor *config_descriptor;
  CHECK_LIBUSB(libusb_get_config_descriptor(device, 0, &config_descriptor));

  for (uint8_t i = 0; i < config_descriptor->bNumInterfaces; ++i) {
    const libusb_interface *const interface = &config_descriptor->interface[i];
    CHECK(interface->num_altsetting == 1);
    const libusb_interface_descriptor *const interface_descriptor =
        &interface->altsetting[0];
    if (interface_descriptor->bInterfaceClass == 0xFF &&
        interface_descriptor->bInterfaceSubClass == 0x97 &&
        interface_descriptor->bInterfaceProtocol == 0x97) {
      CHECK(interface_descriptor->bNumEndpoints == 1);
      const libusb_endpoint_descriptor *const endpoint_descriptor =
          &interface_descriptor->endpoint[0];
      CHECK_LIBUSB(libusb_claim_interface(
          handle, interface_descriptor->bInterfaceNumber));
      fprintf(stderr, "Found endpoint 0x%" PRIx8 "\n",
              endpoint_descriptor->bEndpointAddress);
      *endpoint = endpoint_descriptor->bEndpointAddress;
      libusb_free_config_descriptor(config_descriptor);
      return;
    }
  }

  fprintf(stderr, "Could not find interface\n");
  abort();
}

constexpr uint16_t port_number() { return 10971; }

}  // namespace

int main(int argc, char ** /*argv*/) {
  CHECK_LIBUSB(libusb_init(nullptr));
  libusb_set_debug(nullptr, LIBUSB_LOG_LEVEL_INFO);

  if (argc > 1) {
    libusb_set_debug(nullptr, LIBUSB_LOG_LEVEL_DEBUG);
  }

#ifdef __WINNT__
  {
    WSADATA wsa;
    PCHECK(WSAStartup(MAKEWORD(2, 2), &wsa));
  }
#endif

  libusb_device_handle *handle;
  OpenDevice(&handle);
  uint8_t endpoint;
  FindEndpoint(handle, &endpoint);

  const int socket_fd = PCHECK(socket(AF_INET, SOCK_DGRAM, 0));
  {
    sockaddr_in bind_address;
    memset(&bind_address, 0, sizeof(bind_address));
    bind_address.sin_family = AF_INET;
    bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_address.sin_port = htons(port_number());
    PCHECK(bind(socket_fd, reinterpret_cast<sockaddr *>(&bind_address),
                sizeof(bind_address)));
  }

  while (true) {
    char buffer[64];
#ifdef __WINNT__
    // TODO(Brian): Figure out how to detect too-large UDP packets.
    static constexpr int kFlags = 0;
#else
    static constexpr int kFlags = MSG_TRUNC;
#endif
    const int length = PCHECK(recv(socket_fd, buffer, sizeof(buffer), kFlags));
    if (static_cast<size_t>(length) > sizeof(buffer)) {
      fprintf(stderr, "Too-long packet of %d: ignoring\n", length);
      continue;
    }

    int transferred;
    CHECK_LIBUSB(libusb_interrupt_transfer(
        handle, endpoint, reinterpret_cast<unsigned char *>(buffer), length,
        &transferred, 100));
    if (transferred != length) {
      fprintf(stderr, "Transferred %d instead of %d\n", transferred, length);
    }
  }
}
