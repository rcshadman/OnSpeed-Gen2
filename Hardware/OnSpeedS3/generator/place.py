"""Board placement. KiCad coordinates in mm: x 0..100 left to right, y 0 = front edge (pitot/static
tubes, SD slot), y 80 = rear edge (DB-15, USB-C, DE-9). Parts are found by symbol and nets, so the
placement survives reference renumbering in design.py.

Layout rules followed here:
- the left and right 2.5 mm strips stay clear for the enclosure card rails;
- nothing sits under the D-sub housings (y > 66 between their flanges);
- decoupling capacitors sit on the pin they serve, MLCCs are kept away from board edges, D-sub
  flanges and mounting holes (flex cracks under vibration);
- the IMU is near the board centre, away from the inductor and the connectors' mating forces.
"""
import design as D

# board-edge geometry of the connectors (CONNFLY DS1037 ...KT74: pin rows 2.84 apart, pin 9 row 9.40 mm
# from the flange, so the first row is 12.24 mm from the board edge)
DSUB_Y = 80 - 12.24
DB15_X = 87.0   # pin 1
DE9_X = 13.0    # pin 1

P = []  # (symbol, nets multiset, x, y, rot, side)


def at(sym, nets, x, y, rot=0, side="F"):
    P.append((sym, None if nets is None else tuple(sorted(nets)), x, y, rot, side))


# ---------------------------------------------------------------- connectors
at("DA15_Receptacle_MountingHoles", ("GND", "VIN_RAW", "RS232_EFIS_IN", "X_SW", "RS232_GPS_IN", "X_LED", "X_AUD_L", "X_FLAP",
   "X_AUD_R", "X_DISP", "X_VOL", "X_3V3", "RS232_EFIS_OUT", "X_BOOM", "GND", "AUDIO_GND"), DB15_X, DSUB_Y)
at("DE9_Plug_MountingHoles", ("GND", "CANL", "GND", "GND", "CANH", "GND"), DE9_X, DSUB_Y)
at("USB_C_Receptacle_USB2.0_16P", None, 45.0, 76.4)

# ---------------------------------------------------------------- front: pressure sensors, SD, expansion
at("MS4525DO", ("GND", "+5V_S", "I2CA5_SDA", "I2CA5_SCL"), 35.0, 8.5, 270)
at("MS4525DO", ("GND", "+5V_S", "I2CB5_SDA", "I2CB5_SCL"), 12.0, 8.5, 270)
at("SSC_SIP4", ("GND", "+3V3_S", "I2CA_SDA", "I2CA_SCL"), 22.5, 4.86, 90)
at("PCA9306", ("GND", "+3V3_S", "I2CA_SCL", "I2CA_SDA", "I2CA5_SDA", "I2CA5_SCL", "I2CA_VREF2", "I2CA_VREF2"), 35.0, 17.6, 0)
at("PCA9306", ("GND", "+3V3_S", "I2CB_SCL", "I2CB_SDA", "I2CB5_SDA", "I2CB5_SCL", "I2CB_VREF2", "I2CB_VREF2"), 12.0, 17.6, 0)
for bus, x0 in (("A", 29.6), ("B", 6.0)):
    xs = [x0 + i * 1.75 for i in range(6)] + [x0 + 6 * 1.75 + 0.5, x0 + 7 * 1.75 + 1.0]
    parts = [("R", ("+5V_S", f"I2C{bus}5_SDA")), ("R", ("+5V_S", f"I2C{bus}5_SCL")), ("R", ("+5V_S", f"I2C{bus}_VREF2")),
             ("C", (f"I2C{bus}_VREF2", "GND")), ("R", ("+3V3_S", f"I2C{bus}_SDA")), ("R", ("+3V3_S", f"I2C{bus}_SCL")),
             ("C", ("+5V_S", "GND")), ("C", ("+3V3_S", "GND"))]
    for xx, (s, n) in zip(xs, parts):
        at(s, n, xx, 21.4, 90)
at("C", ("+3V3_S", "GND"), 22.5, 13.0, 0)                # SSC
at("FerriteBead_Small", ("+5V", "+5V_S"), 22.5, 16.2, 0)
at("C", ("+5V_S", "GND"), 22.5, 19.4, 0)                 # 10 uF
at("LSM6DSM", None, 50.0, 30.0, 0)
at("C", ("+3V3_S", "GND"), 47.0, 26.8, 0)
at("C", ("+3V3_S", "GND"), 53.0, 26.8, 0)
at("C", ("+3V3_S", "GND"), 53.0, 33.2, 0)                # 1 uF
at("Conn_01x04", ("+3V3_S", "I2CB_SCL", "I2CB_SDA", "GND"), 52.0, 4.6, 180)
at("Micro_SD_Card_Det2", None, 84.0, 6.4, 180)
for i, n in enumerate((("SD_CLK", "SD_CLK_C"), ("+3V3", "SD_CMD"), ("+3V3", "SD_D0"), ("+3V3", "SD_D1"), ("+3V3", "SD_D2"),
                       ("+3V3", "SD_D3"), ("+3V3", "SD_DET"))):
    at("R", n, 76.6 + i * 1.75, 15.4, 90)
at("C", ("+3V3", "GND"), 90.0, 15.4, 90)                 # 10 uF
at("C", ("+3V3", "GND"), 91.9, 15.4, 90)

# ---------------------------------------------------------------- ESP32-S3 module, antenna to the left
at("ESP32-S3-WROOM-1", None, 16.0, 42.0, 90)
at("C", ("+3V3", "GND"), 11.2, 53.9, 0)                  # 22 uF at pin 2
at("C", ("+3V3", "GND"), 14.6, 53.6, 0)
at("C", ("+3V3", "GND"), 17.8, 53.6, 0)
at("R", ("+3V3", "EN"), 21.0, 53.6, 0)
at("C", ("EN", "GND"), 24.2, 53.6, 0)
at("SW_Push", ("EN", "GND"), 33.4, 46.0, 90)
at("SW_Push", ("BOOT", "GND"), 33.4, 38.6, 90)
at("R", ("+3V3", "BOOT"), 31.6, 34.4, 0)
at("R", ("PGOOD", "+3V3"), 31.6, 32.6, 0)
at("C", ("VIN_SENSE", "GND"), 31.6, 50.6, 0)
at("BAT54S", ("GND", "+3V3", "FLAP_ADC"), 11.0, 29.4, 0)
at("BAT54S", ("GND", "+3V3", "VOL_ADC"), 15.4, 29.4, 0)
at("C", ("FLAP_ADC", "GND"), 11.0, 26.4, 0)
at("C", ("VOL_ADC", "GND"), 15.4, 26.4, 0)
at("R", ("STAT_LED", "LED_STAT"), 20.4, 29.4, 0)
at("LED", ("GND", "LED_STAT"), 23.6, 29.4, 180)
at("R", ("+3V3", "LED_PWR"), 20.4, 26.6, 0)
at("LED", ("GND", "LED_PWR"), 23.6, 26.6, 180)
at("BAT54S", ("GND", "+3V3", "SW_IN"), 12.6, 57.4, 0)
at("BAT54S", ("GND", "+3V3", "BOOM_RX"), 17.0, 57.4, 0)
for i, n in enumerate(("VIN", "+5V", "+3V3", "GND", "GND", "EN", "BOOT", "U0TXD", "U0RXD")):
    at("TestPoint", (n,), 35.0 + i * 2.6, 40.0, 0, "B")

# ---------------------------------------------------------------- CAN
at("TJA1051T-3", None, 24.0, 59.6, 0)
at("NUP2105L", ("CANH", "CANL", "GND"), 17.0, 62.6, 0)
at("C", ("+5V", "GND"), 29.8, 57.4, 0)
at("C", ("+5V", "GND"), 29.8, 59.6, 0)                   # 4.7 uF
at("C", ("+3V3", "GND"), 29.8, 61.8, 0)
at("R", ("CAN_S", "GND"), 20.8, 55.8, 0)
at("SolderJumper_2_Open", ("CANH", "CAN_TH"), 22.4, 64.2, 0)
at("R", ("CAN_TH", "CAN_MID"), 25.6, 64.2, 90)
at("R", ("CAN_TH", "CAN_MID"), 27.2, 64.2, 90)
at("R", ("CAN_MID", "CANL"), 28.8, 64.2, 90)
at("R", ("CAN_MID", "CANL"), 30.4, 64.2, 90)
at("C", ("CAN_MID", "GND"), 32.4, 64.2, 90)

# ---------------------------------------------------------------- USB-C and 3.3 V regulator
at("USBLC6-2SC6", None, 45.0, 68.4, 0)
at("R", ("USB_CC1", "GND"), 40.6, 68.4, 90)
at("R", ("USB_CC2", "GND"), 49.4, 68.4, 90)
at("C", ("VBUS", "GND"), 52.2, 70.4, 90)
at("SS14", ("+5V", "VBUS"), 52.0, 64.0, 0)
at("AMS1117-3.3", None, 45.0, 57.0, 0)
at("C", ("+5V", "GND"), 51.4, 57.0, 90)                  # 10 uF in
at("C", ("+3V3", "GND"), 38.8, 55.6, 90)                 # 22 uF out
at("C", ("+3V3", "GND"), 38.8, 59.0, 90)
at("FerriteBead_Small", ("+3V3", "+3V3_S"), 45.0, 50.8, 0)
at("C", ("+3V3_S", "GND"), 45.0, 48.6, 0)                # 10 uF
at("C", ("+3V3_S", "GND"), 45.0, 46.6, 0)

# ---------------------------------------------------------------- DB-15 line protection comb
# one column per connector line, ESD diode at the connector, then the series part, then the shunts
COL = {"X_AUD_R": 66.0, "X_FLAP": 68.4, "X_AUD_L": 70.8, "X_BOOM": 73.2, "X_LED": 75.6, "X_3V3": 78.2, "X_SW": 80.8,
       "X_VOL": 83.2, "X_DISP": 85.6}
ESD_Y, S0 = 63.0, 59.6
for net, x in COL.items():
    at("D_TVS", (net, "GND"), x, ESD_Y, 90)
stack = {
    "X_AUD_R": [("R", ("AUDR_P", "X_AUD_R")), ("R", ("AUDR_P", "GND")), ("R", ("AUDR_F", "AUDR_P")), ("C", ("AUDR_F", "GND")),
                ("R", ("DAC_R", "AUDR_F"))],
    "X_AUD_L": [("R", ("AUDL_P", "X_AUD_L")), ("R", ("AUDL_P", "GND")), ("R", ("AUDL_F", "AUDL_P")), ("C", ("AUDL_F", "GND")),
                ("R", ("DAC_L", "AUDL_F"))],
    "X_FLAP": [("R", ("X_FLAP", "FLAP_ADC")), ("R", ("FLAP_ADC", "GND"))],
    "X_BOOM": [("R", ("X_BOOM", "BOOM_RX"))],
    "X_LED": [("R", ("LED_OUT", "X_LED"))],
    "X_SW": [("R", ("X_SW", "SW_IN")), ("R", ("+3V3", "SW_IN")), ("C", ("SW_IN", "GND"))],
    "X_VOL": [("R", ("X_VOL", "VOL_ADC")), ("R", ("+3V3", "VOL_ADC")), ("R", ("VOL_ADC", "GND"))],
    "X_DISP": [("R", ("DISP_TX", "X_DISP"))],
}
for net, items in stack.items():
    for i, (s, n) in enumerate(items):
        at(s, n, COL[net], S0 - i * 3.0, 90)
at("SS14", ("X_3V3", "3V3_OUT_F"), COL["X_3V3"], 57.6, 90)
at("Polyfuse", ("+3V3", "3V3_OUT_F"), COL["X_3V3"], 52.4, 90)
at("R", ("AUDIO_GND", "GND"), 63.0, 63.0, 90)

# ---------------------------------------------------------------- audio DAC
at("PCM5102A", None, 68.0, 41.0, 0)
for i, n in enumerate((("DAC_CAPP", "DAC_CAPM"), ("DAC_VNEG", "GND"), ("+3V3_A", "GND"), ("+3V3_A", "GND"), ("+3V3_A", "GND"),
                       ("+3V3_A", "GND"))):
    at("C", n, 61.8, 37.4 + i * 2.1, 0)
at("FerriteBead_Small", ("+3V3", "+3V3_A"), 61.8, 50.2, 0)
at("C", ("+3V3_A", "GND"), 61.8, 33.8, 0)                # rail 10 uF
for i, n in enumerate((("DAC_LDOO", "GND"), ("+3V3", "GND"), ("+3V3", "GND"), ("DAC_XSMT", "GND"))):
    at("C" if n[0] != "DAC_XSMT" else "R", n, 74.2, 37.6 + i * 1.8, 0)

# ---------------------------------------------------------------- RS-232
at("MAX3232", None, 80.5, 45.0, 0)
for i, n in enumerate((("RS_C1P", "RS_C1N"), ("RS_C2P", "RS_C2N"), ("RS_VP", "GND"), ("RS_VN", "GND"), ("+3V3", "GND"))):
    at("C", n, 87.4, 44.2 + i * 1.8, 0)

# ---------------------------------------------------------------- power input and buck
at("Polyfuse", ("VIN_RAW", "VIN_F"), 91.3, 61.6, 0)
at("SS210", ("VIN_P", "VIN_F"), 91.0, 54.6, 90)
at("D_Zener", ("VIN_P", "GND"), 95.6, 54.6, 90)
at("C", ("VIN_P", "GND"), 88.0, 54.6, 90)                # 100 nF 100 V
at("FerriteBead_Small", ("VIN_P", "VIN"), 91.0, 49.2, 90)
at("C_Polarized", ("VIN", "GND"), 93.3, 38.15, 90)
at("LM5164DDA", None, 82.0, 31.0, 180)
at("C", ("VIN", "GND"), 87.45, 30.4, 270)                 # 2.2 uF 100 V
at("C", ("VIN", "GND"), 90.85, 30.4, 270)
at("C", ("VIN", "GND"), 84.0, 35.6, 0)                   # 100 nF 100 V
at("L", ("BUCK_SW", "+5V"), 74.0, 31.6, 0)
at("C", ("+5V", "GND"), 68.6, 29.6, 90)
at("C", ("+5V", "GND"), 68.6, 33.6, 90)
at("C", ("+5V", "GND"), 66.4, 31.6, 90)
at("C", ("BUCK_BST", "BUCK_SW"), 80.6, 35.4, 0)
for i, (s, n) in enumerate((("R", ("+5V", "BUCK_FB")), ("R", ("BUCK_FB", "GND")), ("C", ("BUCK_RA", "BUCK_FB")),
                            ("R", ("BUCK_SW", "BUCK_RA")), ("C", ("BUCK_RA", "+5V")))):
    at(s, n, 70.6 + i * 2.0, 25.6, 90)
for i, n in enumerate((("VIN", "BUCK_EN"), ("BUCK_EN", "GND"), ("BUCK_RON", "GND"))):
    at("R", n, 83.0 + i * 2.0, 25.6, 90)
at("R", ("VIN", "VIN_SENSE"), 92.0, 25.6, 90)
at("R", ("VIN_SENSE", "GND"), 94.0, 25.6, 90)

# ---------------------------------------------------------------- mechanical
for x, y in ((4.5, 27.5), (4.5, 59.0), (95.5, 4.5), (95.8, 47.0)):
    at("MountingHole_Pad", ("GND",), x, y)
for x, y in ((45.6, 2.5), (97.5, 19.5), (2.0, 64.0)):
    at("Fiducial", (), x, y)


def resolve():
    """Map each placement row to a part reference."""
    used, out, errors = set(), {}, []
    for sym, nets, x, y, rot, side in P:
        cands = [p for p in D.parts if p.sym == sym and p.ref not in used and not p.ref.startswith("#")
                 and (nets is None or tuple(sorted(n for n in p.nets.values() if n)) == nets)]
        if not cands:
            errors.append(f"no part for {sym} {nets}")
            continue
        p = cands[0]
        used.add(p.ref)
        out[p.ref] = (x, y, rot, side)
    missing = [p.ref for p in D.parts if p.footprint and not p.ref.startswith("#") and p.ref not in used]
    return out, errors, missing


if __name__ == "__main__":
    out, errors, missing = resolve()
    for e in errors:
        print("ERROR", e)
    print("unplaced:", missing)
