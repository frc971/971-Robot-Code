#!/usr/bin/env python

# This file is a modified version of the gen_embedded script included in the
# scripts directory of seasocks (version 1.1.2). It has been modified to
# recursively find the web files itself, which was originally done by piping
# in the results from a "find" shell command.

# The embedded files includes only those that are required for the server to run
# (including 404 files, a default index page, favicon, etc.)

import os, os.path, sys

o = open('embedded.h', 'w')

web = []
for root, dirs, files in os.walk("./web", topdown=False):
  for name in files:
    web.append(os.path.join(root, name))
  for name in dirs:
    web.append(os.path.join(root, name))

o.write("""
#include "internal/Embedded.h"

#include <string>
#include <unordered_map>

namespace {

std::unordered_map<std::string, EmbeddedContent> embedded = {
""")

for f in web:
  bytes = open(f, 'rb').read()
  o.write('{"/%s", {' % os.path.basename(f))
  o.write('"' + "".join(['\\x%02x' % ord(x) for x in bytes]) + '"')
  o.write(',%d }},' % len(bytes))

o.write("""
};

} // namespace

const EmbeddedContent* findEmbeddedContent(const std::string& name) {
  auto found = embedded.find(name);
  if (found == embedded.end()) {
    return NULL;
  }
  return &found->second;
}
""")

o.close()
