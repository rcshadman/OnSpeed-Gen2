#!/bin/sh
# Full board build: placement -> ground fan-out -> autoroute -> clean-up -> pours, stitching, silk, DRC.
set -e
cd "$(dirname "$0")"
python3 make_pcb.py
python3 route.py --prepare-only
FANOUT_ONLY=1 python3 finish.py out/onspeed_planes.kicad_pcb out/onspeed_fanout.kicad_pcb
ROUTE_GND=1 python3 route.py ${PASSES:-80}
python3 ses_import.py out/onspeed_fanout.kicad_pcb out/onspeed_gnd.ses out/onspeed_routed.kicad_pcb
rm -f out/onspeed_fixed.kicad_pcb
# KiCad 7 sometimes segfaults at interpreter exit after a successful save; the output file is the check
for i in 1 2 3; do python3 fixups.py out/onspeed_routed.kicad_pcb out/onspeed_fixed.kicad_pcb && break; done; test -f out/onspeed_fixed.kicad_pcb
python3 finish.py out/onspeed_fixed.kicad_pcb out/onspeed_final.kicad_pcb
