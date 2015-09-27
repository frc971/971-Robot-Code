#!/usr/bin/env python3

# This file is a modified version of the gen_embedded script included in the
# scripts directory of seasocks (version 1.1.2). It has been modified to
# recursively find the web files itself, which was originally done by piping
# in the results from a "find" shell command.

# The embedded files includes only those that are required for the server to run
# (including 404 files, a default index page, favicon, etc.)

import os, os.path, sys

output = sys.argv[1].replace('"', '')

if not os.path.exists(os.path.dirname(output)):
  os.makedirs(os.path.dirname(output))

if len(sys.argv) >= 3:
  web = sys.argv[2:]
else:
  web = []
  for root, dirs, files in os.walk("./www_defaults", topdown=False):
    for name in files + dirs:
      web.append(os.path.join(root, name))

with open(output, 'w') as o:
  o.write("""
#include "internal/Embedded.h"

#include <string>
#include <unordered_map>

namespace {

std::unordered_map<std::string, EmbeddedContent> embedded = {
""")

  for f in web:
    with open(f, 'rb') as file:
      bytes = file.read()
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
