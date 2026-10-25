# Board generator

Everything in `../kicad` and `../fab` is generated from `design.py` by these scripts (KiCad 7.0 Python, Freerouting 1.9.0 under xvfb).

    python3 make_sch.py            # schematic from design.py
    python3 check_sch.py           # netlist vs design.py
    ./build_board.sh               # place, fan out ground, autoroute, clean up, pours, stitching, silk, DRC
    python3 fab.py out/onspeed_final.kicad_pcb   # Gerbers, JLC BOM/CPL, STEP, PDFs
    python3 glb.py out/fab/onspeed_board.step out/fab/onspeed_board.glb out/onspeed_final.kicad_pcb

Freerouting 1.9.0 goes in `tools/freerouting-1.9.0.jar` (2.1.0 ignores its pass limit in batch mode).
