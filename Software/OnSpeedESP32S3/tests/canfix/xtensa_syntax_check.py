#!/usr/bin/env python3
"""Optional: syntax-check canfix.cpp and main_twai_example.cpp with the project's
xtensa-esp32s3 g++ flags (from onspeed-s3/build/compile_commands.json)."""
import glob, json, shlex, subprocess, sys
SIM = "/home/claude/sim"
d = json.load(open(f"{SIM}/onspeed-s3/build/compile_commands.json"))
e = [e for e in d if "main/" in e["file"] and e["file"].endswith(".cpp")][0]
out, skip = [], False
for a in shlex.split(e["command"]):
    if skip: skip = False; continue
    if a in ("-o", "-c", "-MT", "-MF"): skip = True; continue
    if a == "-MD": continue
    out.append(a)
inc = [f"-I{SIM}/onspeed-s3/components/onspeed_core/include"]
for comp in ["driver/twai", "esp_driver_gpio", "esp_timer", "esp_pm", "esp_ringbuf"]:
    inc += ["-I" + p for p in glob.glob(f"{SIM}/esp-idf/components/{comp}/include")]
rc = 0
for src in [f"{SIM}/onspeed-s3/components/onspeed_core/canfix.cpp", f"{SIM}/hosttest/canfix/main_twai_example.cpp"]:
    r = subprocess.run(out + inc + ["-fsyntax-only", "-Wall", "-Werror", src], capture_output=True, text=True, cwd=e["directory"])
    print(("OK   " if r.returncode == 0 else "FAIL ") + src); sys.stdout.write(r.stderr); rc |= r.returncode
sys.exit(rc)
