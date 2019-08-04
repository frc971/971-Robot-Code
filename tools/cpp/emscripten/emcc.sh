#!/bin/bash
# Originally sourced from https://github.com/ribrdb/rules_emscripten
set -euo pipefail
EM_CONFIG="LLVM_ROOT='$(pwd -P)/external/emscripten_clang';"
EM_CONFIG+="EMSCRIPTEN_NATIVE_OPTIMIZER='external/emscripten_clang/optimizer';"
EM_CONFIG+="BINARYEN_ROOT='external/emscripten_clang/binaryen';"
EM_CONFIG+="NODE_JS='$(pwd -P)/external/nodejs/bin/node';"
EM_CONFIG+="EMSCRIPTEN_ROOT='external/emscripten_toolchain';"
EM_CONFIG+="SPIDERMONKEY_ENGINE='';"
EM_CONFIG+="V8_ENGINE='';"
EM_CONFIG+="TEMP_DIR='tmp';"
EM_CONFIG+="COMPILER_ENGINE=NODE_JS;"
EM_CONFIG+="JS_ENGINES=[NODE_JS];"
export EM_CONFIG

export EM_EXCLUSIVE_CACHE_ACCESS=1
export EMCC_SKIP_SANITY_CHECK=1
# export EMCC_DEBUG=2
export EMCC_WASM_BACKEND=0
export EMMAKEN_NO_SDK=1

mkdir -p "tmp/emscripten_cache"
export EM_CACHE="$(pwd -P)/tmp/emscripten_cache"
export EMCC_TEMP_DIR="$(pwd -P)/tmp"

# Prepare the cache content so emscripten doesn't try to rebuild it all the time
cache_source=tools/cpp/emscripten/emscripten_cache
# TODO(james): How do I avoid hardcoding this path? This is needed to make
# gencache.sh work properly and to have it put the files in the correct spot.
if [ -d bazel-out/host/bin/tools/cpp/emscripten/emscripten_cache ]; then
  cache_source=bazel-out/host/bin/tools/cpp/emscripten/emscripten_cache
elif [ -d external/rules_emscripten/toolchain/emscripten_cache ]; then
  cache_source=external/rules_emscripten/toolchain/emscripten_cache
fi
(
  cd tmp/emscripten_cache;
  for n in "../../$cache_source"/*;do
    ln -f -s "$n"
  done
)

argv=("$@")
tarfile=
# Find the -o option, and strip the .tar from it.
for (( i=0; i<$#; i++ )); do
  if [[ "x${argv[i]}" == x-o ]]; then
    arg=${argv[$((i+1))]}
    if [[ "x$arg" == x*.tar ]];then
        tarfile="$(cd $(dirname "$arg"); pwd -P)/$(basename "$arg")"
        emfile="$(dirname "$arg")/$(basename $arg .tar)"
        basearg="$(basename "$(basename "$(basename "$emfile" .js)" .html)" .wasm)"
        baseout="$(dirname "$arg")/$basearg"
        argv[$((i+1))]="$emfile"
    fi
  fi
done
python external/emscripten_toolchain/emcc.py "${argv[@]}"
# Now create the tarfile
shopt -s extglob
if [ "x$tarfile" != x ]; then
  outdir="$(dirname "$baseout")"
  outbase="$(basename "$baseout")"
  (
      cd "$outdir";
      tar cf "$tarfile" "$outbase."?(html|js|wasm|mem|data|worker.js)
  )
fi
