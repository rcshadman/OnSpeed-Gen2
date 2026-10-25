# OnSpeed S3 board, rev A: order guide

This folder holds everything a board house needs to build the board fully assembled. Nothing has been ordered.
Read the review checklist at the end before you pay.

## What is in the folder

| File | What it is |
|---|---|
| `fab/onspeed_gerbers.zip` | Copper (4 layers), solder mask, silkscreen, paste, outline and drill. Upload this as the PCB. |
| `fab/onspeed_bom_jlc.csv` | Parts list in JLCPCB's format (Comment, Designator, Footprint, LCSC Part #). |
| `fab/onspeed_cpl_jlc.csv` | Pick-and-place positions and rotations in JLCPCB's format. |
| `fab/onspeed_board.step`, `fab/onspeed_board.glb` | 3D model of the assembled board (STEP for CAD, GLB for the web viewer). |
| `fab/assembly_top.pdf`, `fab/copper_*.pdf` | Assembly drawing and one page per copper layer. |
| `kicad/` | KiCad 7 project: schematic, board, design rules and the DRC report. |
| `enclosure/` | 3D-printable case (`.scad` source, base and lid `.stl`). |
| `review/schematic_review.md` | Independent datasheet review; the must-fix items are already applied. |

## Board facts

- 100 × 80 mm, 4 layers, 1.6 mm FR-4. JLC stack-up JLC04161H-7628.
- Layer use: top for parts and signals. Inner 1 is a solid ground plane. Inner 2 carries signals with a 3.3 V pour. The bottom has signals and a ground pour.
- Tracks are 0.2 mm (signals), 0.3 mm (sensor and audio supplies) and 0.5 mm (power), with 0.2 mm clearance throughout. Vias are 0.6/0.3 mm.
- The LM5164 and ESP32 footprints have 0.2 mm thermal vias, which JLC accepts on 4-layer boards.
- About 1,400 track segments and 420 vias. Every ground pin has its own via to the ground plane.
- There is a via ring around the board edge every 2.5 mm, and the inner layers are stitched every 5 mm.
- No copper sits under the ESP32 antenna.
- Design-rule check: 0 errors and 0 unconnected items. The only remaining messages say the checker had no KiCad library table configured, which does not affect the board.

## Ordering at JLCPCB (recommended)

1. Go to jlcpcb.com, choose **Order now**, and upload `fab/onspeed_gerbers.zip`.
2. PCB options:
   - Layers 4, thickness 1.6 mm, stack-up JLC04161H-7628, impedance control not needed.
   - Surface finish **ENIG**. This is worth it for the 0.5 mm-pitch IMU and the flat USB-C pads.
   - Mask colour: any. Green is cheapest and fastest.
   - Mark on PCB: **Remove mark**, or "specify a location". There is no reserved spot, so removing it is simplest.
   - Via covering: tented. Leave everything else at default.
   - Quantity: 5, the minimum.
3. Turn on **PCB Assembly**:
   - Choose **Standard**, top side only, quantity 2 or 5.
   - Tooling holes: "Added by JLCPCB".
   - Confirm the parts placement: Yes.
4. Upload `fab/onspeed_bom_jlc.csv` and `fab/onspeed_cpl_jlc.csv`.
5. Parts page:
   - Every line matches an LCSC part except the three pressure sensors.
   - **U5, U6: TE MS4525DO-DS5AI001DP** and **U7: Honeywell SSCSRNN1.6BA7A3**. Use **Global Sourcing / Parts Pre-order** for these, or mark them "do not place" and fit your existing Gen2 sensors yourself. Their footprints match the Gen2 parts.
   - JP1 (CAN termination) is a solder jumper. It has no part, so leave it alone. Bridge it with solder only on the board at the end of the CAN bus.
6. Placement preview (important):
   - Check that the ESP32 module's antenna points to the board's left edge.
   - Check that pin 1 of the IMU (U10), DAC (U11), RS-232 (U12), CAN (U13) and buck (U1) matches the dot on the drawing.
   - Check that the cathode band of D1, D2, D3 and D19 matches the silkscreen bar.
   - Check that the bulk capacitor C1 has its stripe on the minus side.
   - If anything is rotated, correct it in the preview. I applied JLC's usual rotation offsets, but their library occasionally differs.
7. Through-hole parts are hand-soldered by JLC: the two D-subs (J2, J4) and the three sensors. Accept the THT assembly option.

PCBWay works the same way if you prefer: upload the same Gerbers and send the BOM and CPL with a turnkey assembly request.

## Cost estimate (5 boards, USD)

**This is an estimate, not a quote.** JLC's quote is only produced when you upload the files. Prices move with stock and exchange rates.

| Item | For 5 boards | Per board |
|---|---|---|
| 4-layer PCB, ENIG | $25–40 | $5–8 |
| Standard assembly: setup, stencil, about 40 extended-part fees, THT soldering | $90–140 | $18–28 |
| Parts on the LCSC list (ESP32-S3 module, DAC, IMU, connectors and so on) | $110–160 | $22–32 |
| Pressure sensors, MS4525DO × 2 + SSC × 1 (Global Sourcing) | about $1,170 | about $234 |
| Shipping (DHL or FedEx to Australia) | $30–45 | $6–9 |
| **Total with sensors** | **about $1,425–1,555** | **about $285–310** |
| Total if you fit your own Gen2 sensors | about $255–385 | about $51–77 |

The case is extra if you have it printed. In MJF PA12 (JLC3DP or similar), the base and lid cost about $15–30 per set.

## Review checklist before you pay

1. The DB-15 pinout must match your aircraft harness and the Gen2 box. The pin table is in `kicad/onspeed.kicad_sch`, block 4.
   - Pin 1 is aircraft power, 14 is ground and 15 is audio ground.
   - Pin 11 now supplies about 2.95 V, down from 3.3 V, because of the added reverse-protection diode. The volume knob reading shifts slightly, and firmware can scale for it.
2. Check the placement preview as described in step 6.
3. Check that the sensor prices and lead times on the Global Sourcing page are acceptable, or choose to fit your own.
4. The JST-GH I2C expansion connector has no ESD protection. Treat it as an inside-the-box connector.
5. CAN termination stays open unless this box is at the end of the CAN-FiX bus.

## First power-up (when the boards arrive)

1. Use a bench supply at 12 V with a 150 mA current limit. Idle current should be roughly 60–120 mA.
2. Check the test points: VIN, then 5.0 V, then 3.3 V. The green PWR LED should be on.
3. Plug in USB-C and flash the firmware from the `onspeed-esp32s3` branch. The board also runs from USB alone.
4. Before conformal coating, check that the sensors read zero airspeed and the right static pressure. Mask the sensor ports and connectors when you coat.
