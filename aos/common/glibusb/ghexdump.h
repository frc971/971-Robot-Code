// Copyright 2012 Google Inc. All Rights Reserved.

#ifndef _GLIBUSB_GHEXDUMP_H_
#define _GLIBUSB_GHEXDUMP_H_

#include <stddef.h>
#include <string>

namespace glibusb {

std::string Dump(const void *buf, size_t n);

}  // namespace glibusb

#endif  // _GLIBUSB_GHEXDUMP_H_
