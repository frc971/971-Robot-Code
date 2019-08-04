#!/bin/bash
# This script forces generation of all the necessary cache files from emscripten.
export EMCC_FORCE_STDLIBS=1
# Run with WEBGL2 enabled and not, as the compiler will only generate one of the
# webgl libraries at once.
tools/cpp/emscripten/emcc.sh -o foo.html.tar tools/cpp/emscripten/foo.o -s 'USE_WEBGL2=1' -no-canonical-prefixes
tools/cpp/emscripten/emcc.sh -o foo.html.tar tools/cpp/emscripten/foo.o -s 'USE_WEBGL2=0' -no-canonical-prefixes
for OUTPUT in $@
do
  if [ ! -f ${OUTPUT} ]; then
    cp -f tmp/emscripten_cache/asmjs/$(basename ${OUTPUT}) ${OUTPUT}
  fi
done
