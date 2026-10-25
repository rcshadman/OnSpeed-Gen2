#!/bin/sh
# Host tests for components/onspeed_core/canfix.{h,cpp}
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
CORE=$HERE/../../firmware/components/onspeed_core
WORK=${TMPDIR:-/tmp}/canfix_xcheck.$$
echo "== build (g++ -Wall -Wextra -Werror, ASan+UBSan)"
g++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
    -I"$CORE/include" "$HERE/test_canfix.cpp" "$CORE/canfix.cpp" "$CORE/g5.cpp" -o "$HERE/canfix_test"
echo "== C++ unit tests"
"$HERE/canfix_test"
echo "== cross-check vs python-canfix"
python3 -c "import canfix" 2>/dev/null || python3 -m pip install -q python-canfix
python3 "$HERE/crosscheck.py" "$HERE/canfix_test" "$WORK"
rm -rf "$WORK"
if [ -f "$HERE/../../firmware/build/compile_commands.json" ]; then
  echo "== xtensa-esp32s3 syntax check (ESP-IDF flags)"
  python3 "$HERE/xtensa_syntax_check.py"
fi
echo "ALL PASS"
