#!/bin/sh
# Builds the OnSpeed core for the browser demo (needs zig: pip install ziglang).
set -e
here=$(cd "$(dirname "$0")" && pwd)
core=$here/../firmware/components/onspeed_core
python3 -m ziglang c++ -target wasm32-wasi -Os -s -fno-exceptions -fno-rtti -mexec-model=reactor -Wall \
  -I"$core/include" "$here/wasm_api.cpp" "$core/onspeed_core.cpp" "$core/tone_synth.cpp" "$core/g5.cpp" \
  -o "$here/onspeed_core.wasm"
