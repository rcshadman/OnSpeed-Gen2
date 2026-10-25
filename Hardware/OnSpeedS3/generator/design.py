"""OnSpeed ESP32-S3 board: complete circuit, as data.

One source of truth: the schematic, the PCB nets, the BOM and the placement file are all generated
from the component list built here. Positions are schematic coordinates in mm on an A1 sheet; the
board placement lives in place.py.

Design notes that matter for review are kept next to the block they belong to.
"""
from sch import Symbol
from boxsym import box_symbol

# ---------------------------------------------------------------- part catalogue
# Passives are 0603 unless noted. LCSC numbers are JLC "basic" parts where one exists; parts.json in
# the shared folder records how each number was checked.
RES = {  # 0603 1% thick film
    "0": "C21189", "33": "C23140", "120": "C22787", "100": "C22775", "150": "C22808", "470": "C23179",
    "560": "C23204", "1k": "C21190", "2k": "C22975", "4.7k": "C23162", "5.1k": "C23186", "5.6k": "C23189",
    "8.2k": "C25981", "10k": "C25804", "20k": "C4184", "22k": "C31850", "31.6k": "C25967", "100k": "C25803", "150k": "C22807",
    "200k": "C25811", "330k": "C23137",
}
CAP = {  # value -> (LCSC, footprint, rating)
    "100n": ("C14663", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "100n/100V": ("C28233", "Capacitor_SMD:C_0805_2012Metric", "100V X7R"),
    "10n": ("C57112", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "4.7n": ("C53987", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "2.2n": ("C1604", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "1u": ("C15849", "Capacitor_SMD:C_0603_1608Metric", "16V X7R"),
    "2.2u": ("C23630", "Capacitor_SMD:C_0603_1608Metric", "16V X5R"),
    "4.7u": ("C1779", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "220p": ("C1805", "Capacitor_SMD:C_0805_2012Metric", "50V C0G"),
    "10u": ("C15850", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "22u": ("C45783", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "2.2u/100V": ("C86054", "Capacitor_SMD:C_1210_3225Metric", "100V X7S"),
}
R0603 = "Resistor_SMD:R_0603_1608Metric"

# Parts whose final choice depends on stock and footprint checks. Each entry is (value, footprint,
# LCSC, MPN). The parts agent's results replace the placeholders.
PART = {
    "esp": ("ESP32-S3-WROOM-1-N8", "OnSpeed:ESP32-S3-WROOM-1_BodyCourtyard", "C2913198", "ESP32-S3-WROOM-1-N8"),
    "buck": ("LM5164DDAR", "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.95x4.9mm_Mask2.71x3.4mm_ThermalVias", "C477928", "LM5164DDAR"),
    "ind": ("33uH 1.8A shielded", "Inductor_SMD:L_Changjiang_FNR6045S", "C168083", "FNR6045S330MT"),
    "ldo": ("LDL1117S33R", "Package_TO_SOT_SMD:SOT-223-3_TabPin2", "C435835", "LDL1117S33R"),
    "tvs_in": ("SMBJ58A", "Diode_SMD:D_SMB", "C10226", "SMBJ58A"),
    "rev": ("SS210", "Diode_SMD:D_SMA", "C14996", "SS210"),
    "ss14": ("SS14", "Diode_SMD:D_SMA", "C2480", "SS14"),
    "ptc_in": ("0.75A 60V PTC", "Fuse:Fuse_2920_7451Metric", "C210842", "MF-SM075/60-2"),
    "ptc_3v3": ("0.1A 15V PTC", "Fuse:Fuse_0603_1608Metric", "C883088", "BSMD0603-010-15V"),
    "fb_in": ("220R@100MHz 2A", "Inductor_SMD:L_0805_2012Metric", "C85840", "BLM21PG221SN1D"),
    "fb": ("600R@100MHz 1.3A", "Inductor_SMD:L_0603_1608Metric", "C85833", "BLM18KG601SN1D"),
    "bulk": ("22uF 100V", "Capacitor_SMD:CP_Elec_8x10", "C116424", "CS2A220M-CRF10"),
    "esd15": ("15V bidir ESD", "Diode_SMD:D_SOD-323", "C2985482", "PESD15VL1BA"),
    "esd24": ("24V bidir ESD", "Diode_SMD:D_SOD-323", "C314263", "PESD24VL1BA"),
    "pca9306": ("PCA9306DCUR", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm", "C33196", "PCA9306DCUR"),
    "bat54s": ("BAT54S", "Package_TO_SOT_SMD:SOT-23", "C19726", "BAT54SLT1G"),
    "usblc6": ("USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6", "C7519", "USBLC6-2SC6"),
    "usbc": ("USB-C 16P", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", "C165948", "TYPE-C-31-M-12"),
    "db15": ("DA-15 female", "OnSpeed:DSUB-15_Female_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm_Holes33.32mm", "C77835", "DS1037-15FNAKT74-0CC"),
    "de9": ("DE-9 male", "OnSpeed:DSUB-9_Male_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm_Holes24.99mm", "C75750", "DS1037-09MNAKT74-0CC"),
    "dac": ("PCM5102APWR", "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm", "C107671", "PCM5102APWR"),
    "rs232": ("SP3232EEN-L/TR", "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm", "C9378", "SP3232EEN-L/TR"),
    "can": ("TJA1051T/3/1J", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", "C38695", "TJA1051T/3/1J"),
    "canesd": ("PESD2CAN", "Package_TO_SOT_SMD:SOT-23", "C2687131", "PESD2CAN"),
    "imu": ("ISM330DHCX", "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y", "C2655101", "ISM330DHCXTR"),
    "ms4525": ("MS4525DO-DS5AI001DP", "OnSpeed:MS4525DO_DIP8_500mil", "", "MS4525DO-DS5AI001DP"),
    "ssc": ("SSCSRNN1.6BA7A3", "OnSpeed:Honeywell_SSC_SIP4_RN", "", "SSCSRNN1.6BA7A3"),
    "sd": ("microSD push-pull", "Connector_Card:microSD_HC_Molex_104031-0811", "C585350", "104031-0811"),
    "sw": ("Tactile 4.2x3.2mm", "Button_Switch_SMD:SW_SPST_PTS810", "C221895", "PTS810SJG250SMTRLFS"),
    "jstgh": ("JST-GH 4P", "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal", "C189895", "SM04B-GHS-TB(LF)(SN)"),
    "led_g": ("Green", "LED_SMD:LED_0603_1608Metric", "C72043", "19-217/GHC-YR1S2/3T"),
    "led_b": ("Blue", "LED_SMD:LED_0603_1608Metric", "C72041", "19-217/BHC-ZL1M2RY/3T"),
}
DS = {
    "esp": "https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf",
    "buck": "https://www.ti.com/lit/ds/symlink/lm5164.pdf",
    "dac": "https://www.ti.com/lit/ds/symlink/pcm5102a.pdf",
    "imu": "https://www.st.com/resource/en/datasheet/ism330dhcx.pdf",
    "can": "https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf",
    "ms4525": "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS4525DO&DocType=Data+Sheet&DocLang=English",
    "ssc": "https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/board-mount-pressure-sensors/trustability-ssc-series/documents/sps-siot-trustability-ssc-series-standard-accuracy-board-mount-pressure-sensors-50099533-a-en-ciid-151134.pdf",
}

parts = []
notes = []  # (text, x, y, size, bold)
frames = []  # (x1, y1, x2, y2, title)
_n = {}


def nxt(prefix):
    _n[prefix] = _n.get(prefix, 0) + 1
    return f"{prefix}{_n[prefix]}"


def add(s):
    parts.append(s)
    return s


class Block:
    """A titled frame on the sheet. Small parts are placed on a grid of horizontal cells."""

    def __init__(self, title, x, y, w, h, cell=(48.26, 8.89), top=12.7):
        self.x, self.y, self.w, self.h = x, y, w, h
        self.cell = cell
        self.cx, self.cy = x + 22.86, y + top
        frames.append((x, y, x + w, y + h, title))

    def at(self, dx, dy):
        return (self.x + dx, self.y + dy)

    def start_grid(self, dy):
        self.cx, self.cy = self.x + 22.86, self.y + dy

    def slot(self):
        p = (self.cx, self.cy)
        self.cx += self.cell[0]
        if self.cx > self.x + self.w - 20:
            self.cx = self.x + 22.86
            self.cy += self.cell[1]
        return p

    def note(self, txt, dx, dy, size=1.6):
        notes.append((txt, self.x + dx, self.y + dy, size, False))


def part(key, ref_prefix, libname, sym, nets, at, rot=0, custom=None, extra=None):
    value, fp, lcsc, mpn = PART[key]
    f = {"LCSC": lcsc, "MPN": mpn}
    if key in DS:
        f["Datasheet"] = DS[key]
    if extra:
        f.update(extra)
    return add(Symbol(nxt(ref_prefix), libname, sym, value, fp, f, nets, at, rot, custom=custom))


def R(val, a, b, blk, note=""):
    f = {"LCSC": RES[val]}
    if note:
        f["Note"] = note
    return add(Symbol(nxt("R"), "Device", "R", val, R0603, f, {"1": a, "2": b}, blk.slot(), 90))


def C(val, a, b, blk):
    lcsc, fp, rating = CAP[val]
    v = val.split("/")[0] + "F"
    return add(Symbol(nxt("C"), "Device", "C", v, fp, {"LCSC": lcsc, "Rating": rating}, {"1": a, "2": b}, blk.slot(), 90))


def two(key, prefix, libname, sym, n1, n2, blk, rot=90):
    """Any two-pin part from the PART table."""
    return part(key, prefix, libname, sym, {"1": n1, "2": n2}, blk.slot(), rot)


def flag(net, blk):
    n = len([p for p in parts if p.ref.startswith('#FLG')]) + 1
    return add(Symbol(f"#FLG0{n:02d}", "power", "PWR_FLAG", "PWR_FLAG", "", {}, {"1": net}, blk.slot(), 0, in_bom=False))


def esd(net, blk, key="esd15"):
    """Bidirectional ESD diode from a connector line to ground, placed at the connector."""
    return part(key, "D", "Device", "D_TVS", {"1": net, "2": "GND"}, blk.slot(), 0)


def clamp(net, blk):
    """BAT54S rail clamp: COM on the signal, one diode to GND, one to +3V3."""
    return part("bat54s", "D", "Diode", "BAT54S", {"1": "GND", "2": "+3V3", "3": net}, blk.slot(), 0)


# ---------------------------------------------------------------- custom symbols
SYM_MS4525 = box_symbol("MS4525DO", left=[("2", "VDD", "power_in"), ("1", "GND", "power_in"), None, ("5", "INT/SS", "output")],
                        right=[("3", "SDA", "bidirectional"), ("4", "SCL", "input"), None, ("6", "NC", "no_connect"),
                               ("7", "NC", "no_connect"), ("8", "NC", "no_connect")], ref="U")
SYM_SSC = box_symbol("SSC_SIP4", left=[("2", "VSUPPLY", "power_in"), ("1", "GND", "power_in")],
                     right=[("3", "SDA", "bidirectional"), ("4", "SCL", "input")], ref="U")

# ================================================================= 1. power input and 5 V buck
b = Block("1  Aircraft power in, protection, 5 V buck (LM5164)", 20, 20, 400, 150)
J1_NETS = {}  # filled by the connector block; pin 1 is VIN_RAW
part("ptc_in", "F", "Device", "Polyfuse", {"1": "VIN_RAW", "2": "VIN_F"}, b.at(30, 25), 90)
part("rev", "D", "Diode", "SS210", {"1": "VIN_P", "2": "VIN_F"}, b.at(80, 25), 180)
part("tvs_in", "D", "Device", "D_Zener", {"1": "VIN_P", "2": "GND"}, b.at(120, 40), 90)
part("fb_in", "FB", "Device", "FerriteBead_Small", {"1": "VIN_P", "2": "VIN"}, b.at(150, 25), 90)
part("bulk", "C", "Device", "C_Polarized", {"1": "VIN", "2": "GND"}, b.at(180, 40), 0, extra={"Rating": "100V, damps the input filter; above the 93.6 V TVS clamp"})
part("buck", "U", "Regulator_Switching", "LM5164DDA",
     {"1": "GND", "2": "VIN", "3": "BUCK_EN", "4": "BUCK_RON", "5": "BUCK_FB", "6": "PGOOD", "7": "BUCK_BST",
      "8": "BUCK_SW", "9": "GND"}, b.at(270, 45))
part("ind", "L", "Device", "L", {"1": "BUCK_SW", "2": "+5V"}, b.at(345, 30), 90)
b.start_grid(85)
C("100n/100V", "VIN_P", "GND", b)
C("2.2u/100V", "VIN", "GND", b)
C("2.2u/100V", "VIN", "GND", b)
C("100n/100V", "VIN", "GND", b)
R("330k", "VIN", "BUCK_EN", b, "UVLO: on 6.45 V, off 6.0 V (rides through cranking dips)")
R("100k", "BUCK_EN", "GND", b, "UVLO bottom")
R("31.6k", "BUCK_RON", "GND", b, "fsw = 5 x 2500 / 31.6 = 396 kHz")
R("100k", "+5V", "BUCK_FB", b, "Vout = 1.2 V x (1 + 100k/31.6k) = 5.0 V")
R("31.6k", "BUCK_FB", "GND", b)
R("100k", "BUCK_SW", "BUCK_RA", b, "Type 3 ripple injection Ra: >= 12 mV at FB from 6.45 V up")
C("2.2n", "BUCK_RA", "+5V", b)  # Ca, 50 V part: it sees the full SW swing
C("220p", "BUCK_RA", "BUCK_FB", b)  # Cb, C0G
C("2.2n", "BUCK_BST", "BUCK_SW", b)  # CBST per datasheet
C("22u", "+5V", "GND", b)
C("22u", "+5V", "GND", b)
C("100n", "+5V", "GND", b)
R("150k", "VIN", "VIN_SENSE", b, "VIN monitor: 36 V -> 1.87 V, 58 V -> 3.0 V")
R("8.2k", "VIN_SENSE", "GND", b)
C("100n", "VIN_SENSE", "GND", b)
R("100k", "PGOOD", "+3V3", b)
flag("VIN_RAW", b); flag("VIN", b); flag("GND", b); flag("+5V", b)
b.note("Order of protection: PTC (over-current) -> SS210 (reverse polarity) -> SMBJ58A (clamps 93.6 V, under the LM5164 100 V rating, so load dump and 28 V surges ride through)"
       " -> ferrite + 47 uF damped filter -> LM5164 (100 V rated).", 5, 145)

# ================================================================= 2. USB-C, 3.3 V LDO and rails
b = Block("2  USB-C (power + flashing), 3.3 V LDO, filtered rails", 430, 20, 390, 150)
part("usbc", "J", "Connector", "USB_C_Receptacle_USB2.0_16P",
     {"A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
      "A5": "USB_CC1", "B5": "USB_CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DM", "B7": "USB_DM",
      "A8": None, "B8": None, "S1": "GND"}, b.at(45, 60))
part("usblc6", "U", "Power_Protection", "USBLC6-2SC6",
     {"1": "USB_DM", "6": "USB_DM", "3": "USB_DP", "4": "USB_DP", "2": "GND", "5": "VBUS"}, b.at(130, 50))
part("ss14", "D", "Diode", "SS14", {"1": "+5V", "2": "VBUS"}, b.at(185, 30), 180)
part("ldo", "U", "Regulator_Linear", "AMS1117-3.3", {"1": "GND", "2": "+3V3", "3": "+5V"}, b.at(260, 40))
part("fb", "FB", "Device", "FerriteBead_Small", {"1": "+3V3", "2": "+3V3_S"}, b.at(330, 30), 90)
part("fb", "FB", "Device", "FerriteBead_Small", {"1": "+3V3", "2": "+3V3_A"}, b.at(330, 55), 90)
b.start_grid(100)
R("5.1k", "USB_CC1", "GND", b)
R("5.1k", "USB_CC2", "GND", b)
C("4.7u", "VBUS", "GND", b)
C("10u", "+5V", "GND", b)
C("22u", "+3V3", "GND", b)
C("100n", "+3V3", "GND", b)
C("10u", "+3V3_S", "GND", b)
C("100n", "+3V3_S", "GND", b)
C("10u", "+3V3_A", "GND", b)
R("470", "+3V3", "LED_PWR", b, "power LED (InGaN, Vf about 2.8 V), about 1 mA")
part("led_g", "D", "Device", "LED", {"1": "GND", "2": "LED_PWR"}, b.slot(), 180)
flag("+3V3", b); flag("+3V3_S", b); flag("+3V3_A", b); flag("VBUS", b)
b.note("USB VBUS feeds +5V through SS14, so the board runs and flashes from a laptop with no aircraft power."
       " With both present the buck (5.0 V) carries the load.", 5, 145)

# ================================================================= 3. ESP32-S3 module
b = Block("3  ESP32-S3-WROOM-1-N8", 20, 180, 400, 190)
ESP = {
    "1": "GND", "40": "GND", "41": "GND", "2": "+3V3", "3": "EN",
    "4": "CAN_RX", "5": "SW_IN", "6": "LED_OUT", "7": "BOOM_RX", "8": "EFIS_RX", "9": "GPS_RX",
    "10": "I2CB_SDA", "11": "I2CB_SCL", "12": "I2CA_SDA", "13": "USB_DM", "14": "USB_DP", "15": "VIN_SENSE",
    "16": None, "17": "I2CA_SCL", "18": "DISP_TX", "19": "EFIS_TX", "20": "I2S_BCK", "21": "I2S_LRCK",
    "22": "I2S_DIN", "23": "DAC_XSMT", "24": "CAN_TX", "25": "PGOOD", "26": None, "27": "BOOT",
    "28": "IMU_INT1", "29": "SD_DET", "30": "CAN_S", "31": "STAT_LED", "32": "SD_CLK", "33": "SD_CMD",
    "34": "SD_D0", "35": "SD_D3", "36": "U0RXD", "37": "U0TXD", "38": "VOL_ADC", "39": "FLAP_ADC",
}
part("esp", "U", "RF_Module", "ESP32-S3-WROOM-1", ESP, b.at(70, 80))
part("sw", "SW", "Switch", "SW_Push", {"1": "EN", "2": "GND"}, b.at(170, 30), 0, extra={"Note": "RESET"})
part("sw", "SW", "Switch", "SW_Push", {"1": "BOOT", "2": "GND"}, b.at(170, 50), 0, extra={"Note": "BOOT"})
b2 = Block("", 240, 180, 180, 190)  # grid inside the right half of block 3 (no frame title)
frames.pop()
b2.start_grid(20)
C("22u", "+3V3", "GND", b2)
C("100n", "+3V3", "GND", b2)
C("100n", "+3V3", "GND", b2)
R("10k", "+3V3", "EN", b2, "EN pull-up")
C("1u", "EN", "GND", b2)  # 10 ms reset delay
R("10k", "+3V3", "BOOT", b2)
R("470", "STAT_LED", "LED_STAT", b2)
part("led_b", "D", "Device", "LED", {"1": "GND", "2": "LED_STAT"}, b2.slot(), 180)
for net in ("VIN", "+5V", "+3V3", "GND", "GND", "EN", "BOOT", "U0TXD", "U0RXD"):
    add(Symbol(nxt("TP"), "Connector", "TestPoint", net, "TestPoint:TestPoint_Pad_D1.5mm", {}, {"1": net}, b2.slot(), 0, in_bom=False))
b.note("Strapping: IO0 BOOT button + 10k up; IO3 carries the VIN monitor (strap only used if the JTAG eFuse is set);"
       " IO45/IO46 left open (internal pull-downs: 3.3 V flash, normal boot).", 5, 185)

# ================================================================= 4. DB-15 harness connector and line conditioning
b = Block("4  DB-15 harness (Gen2 pinout) and line protection", 430, 180, 390, 190, cell=(48.26, 8.89))
part("db15", "J", "Connector", "DA15_Receptacle_MountingHoles",
     {"0": "GND", "1": "VIN_RAW", "2": "RS232_EFIS_IN", "3": "X_SW", "4": "RS232_GPS_IN", "5": "X_LED", "6": "X_AUD_L",
      "7": "X_FLAP", "8": "X_AUD_R", "9": "X_DISP", "10": "X_VOL", "11": "X_3V3", "12": "RS232_EFIS_OUT",
      "13": "X_BOOM", "14": "GND", "15": "AUDIO_GND"}, b.at(30, 70))
b.start_grid(12.7)
b.cx = b.x + 80
b.x0 = b.x
b.x += 57  # grid to the right of the connector
for net in ("X_SW", "X_LED", "X_AUD_L", "X_FLAP", "X_AUD_R", "X_DISP", "X_VOL", "X_3V3", "X_BOOM"):
    esd(net, b, "esd24" if net == "X_FLAP" else "esd15")  # flap may sit at 14 V, so it gets the 24 V part
# panel switch: 1k series, 20k pull-up, 10 nF, rail clamp
R("1k", "X_SW", "SW_IN", b); R("20k", "+3V3", "SW_IN", b); C("10n", "SW_IN", "GND", b); clamp("SW_IN", b)
# AOA LED drive
R("150", "LED_OUT", "X_LED", b)
# flap: Gen2 divider 22k / 5.6k (14 V -> 2.84 V)
R("22k", "X_FLAP", "FLAP_ADC", b); R("5.6k", "FLAP_ADC", "GND", b); C("100n", "FLAP_ADC", "GND", b); clamp("FLAP_ADC", b)
# volume: Gen2 network
R("22k", "X_VOL", "VOL_ADC", b); R("22k", "+3V3", "VOL_ADC", b); R("5.6k", "VOL_ADC", "GND", b); C("100n", "VOL_ADC", "GND", b)
clamp("VOL_ADC", b)
# display TX
R("150", "DISP_TX", "X_DISP", b)
# boom RX: 1k series + clamp makes it 5 V tolerant
R("1k", "X_BOOM", "BOOM_RX", b); clamp("BOOM_RX", b)
# 3.3 V out: PTC + Schottky so a miswired 12 V on pin 11 cannot back-feed the 3.3 V rail
part("ptc_3v3", "F", "Device", "Polyfuse", {"1": "+3V3", "2": "3V3_OUT_F"}, b.slot(), 90)
part("ss14", "D", "Diode", "SS14", {"1": "X_3V3", "2": "3V3_OUT_F"}, b.slot(), 180)
# audio return: tied to the ground plane at one point; can be cut and replaced by 10R for a hum loop
R("0", "AUDIO_GND", "GND", b, "audio ground link, see note")
b.note("Every external line has a 15 V bidirectional ESD diode at the connector (flap may see 14 V; audio swings both ways)."
       " RS-232 lines rely on the SP3232E's own +/-15 kV protection.", -52, 185)

# ================================================================= 5. pressure sensors and IMU
b = Block("5  Pressure sensors (Gen2 positions) and ISM330DHCX IMU", 20, 380, 400, 190)
# The MS4525DO-DS5 is calibrated at 5.0 V and TE states the calibration only holds at that supply, so the
# two differential sensors run from a filtered 5 V rail. A PCA9306 per sensor translates its I2C to 3.3 V.
part("ms4525", "U", "OnSpeed", "MS4525DO", {"1": "GND", "2": "+5V_S", "3": "I2CA5_SDA", "4": "I2CA5_SCL", "5": None,
     "6": None, "7": None, "8": None}, b.at(45, 35), custom=SYM_MS4525,
     extra={"Note": "Pfwd, bus A, 0x28, 5 V. Not stocked at JLC: global sourcing or hand fit"})
part("ms4525", "U", "OnSpeed", "MS4525DO", {"1": "GND", "2": "+5V_S", "3": "I2CB5_SDA", "4": "I2CB5_SCL", "5": None,
     "6": None, "7": None, "8": None}, b.at(45, 85), custom=SYM_MS4525, extra={"Note": "P45, bus B, 0x28, 5 V"})
part("ssc", "U", "OnSpeed", "SSC_SIP4", {"1": "GND", "2": "+3V3_S", "3": "I2CA_SDA", "4": "I2CA_SCL"}, b.at(45, 130),
     custom=SYM_SSC, extra={"Note": "static, bus A, 0x78, 3.3 V part (A3 suffix)"})
for bus in ("A", "B"):
    part("pca9306", "U", "Interface", "PCA9306",
         {"1": "GND", "2": "+3V3_S", "3": f"I2C{bus}_SCL", "4": f"I2C{bus}_SDA", "5": f"I2C{bus}5_SDA", "6": f"I2C{bus}5_SCL",
          "7": f"I2C{bus}_VREF2", "8": f"I2C{bus}_VREF2"}, b.at(120, 25 if bus == "A" else 95))
part("imu", "U", "Sensor_Motion", "LSM6DSM",
     {"1": "GND", "2": "GND", "3": "GND", "4": "IMU_INT1", "5": "+3V3_S", "6": "GND", "7": "GND", "8": "+3V3_S",
      "9": None, "10": None, "11": None, "12": "+3V3_S", "13": "I2CB_SCL", "14": "I2CB_SDA"}, b.at(120, 150),
     extra={"Note": "ISM330DHCX (pin-compatible with the LSM6DSM symbol), SA0 low = 0x6A, CS high = I2C"})
part("jstgh", "J", "Connector_Generic", "Conn_01x04", {"1": "+3V3_S", "2": "I2CB_SCL", "3": "I2CB_SDA", "4": "GND"},
     b.at(45, 165), extra={"Note": "I2C expansion, Pixhawk order, 3.3 V"})
part("fb", "FB", "Device", "FerriteBead_Small", {"1": "+5V", "2": "+5V_S"}, b.at(175, 25), 90)
b.start_grid(55)
b.x += 175
b.cell = (48.26, 7.62)
C("10u", "+5V_S", "GND", b); C("100n", "+5V_S", "GND", b); C("100n", "+5V_S", "GND", b)  # sensor rail, one 100n per MS4525
C("100n", "+3V3_S", "GND", b); C("100n", "+3V3_S", "GND", b); C("100n", "+3V3_S", "GND", b)  # SSC, two PCA9306 VREF1
C("100n", "+3V3_S", "GND", b); C("100n", "+3V3_S", "GND", b); C("1u", "+3V3_S", "GND", b)  # IMU VDD, VDDIO
for bus in ("A", "B"):
    R("200k", "+5V_S", f"I2C{bus}_VREF2", b, "PCA9306 VREF2/EN bias")
    C("100n", f"I2C{bus}_VREF2", "GND", b)
    R("4.7k", "+3V3_S", f"I2C{bus}_SDA", b); R("4.7k", "+3V3_S", f"I2C{bus}_SCL", b)
    R("4.7k", "+5V_S", f"I2C{bus}5_SDA", b); R("4.7k", "+5V_S", f"I2C{bus}5_SCL", b)
flag("+5V_S", b)
b.x -= 175
b.note("Two MS4525DO share address 0x28, so each sits on its own bus (as the firmware expects).", 5, 185)

# ================================================================= 6. audio DAC and line-level pad
b = Block("6  Audio: PCM5102A and 600 ohm intercom pad", 430, 380, 390, 190)
part("dac", "U", "Audio", "PCM5102A",
     {"1": "+3V3_A", "2": "DAC_CAPP", "3": "GND", "4": "DAC_CAPM", "5": "DAC_VNEG", "6": "DAC_L", "7": "DAC_R",
      "8": "+3V3_A", "9": "GND", "10": "GND", "11": "GND", "12": "GND", "13": "I2S_BCK", "14": "I2S_DIN", "15": "I2S_LRCK",
      "16": "GND", "17": "DAC_XSMT", "18": "DAC_LDOO", "19": "GND", "20": "+3V3"}, b.at(45, 70))
b.start_grid(12.7)
b.x += 100
C("2.2u", "DAC_CAPP", "DAC_CAPM", b); C("2.2u", "DAC_VNEG", "GND", b); C("100n", "DAC_LDOO", "GND", b)
C("10u", "+3V3_A", "GND", b); C("100n", "+3V3_A", "GND", b); C("100n", "+3V3_A", "GND", b)
C("10u", "+3V3_A", "GND", b); C("100n", "+3V3", "GND", b); C("10u", "+3V3", "GND", b); R("10k", "DAC_XSMT", "GND", b, "keeps the DAC muted until firmware is up")
for ch in ("L", "R"):
    R("470", f"DAC_{ch}", f"AUD{ch}_F", b)
    C("2.2n", f"AUD{ch}_F", "GND", b)
    R("560", f"AUD{ch}_F", f"AUD{ch}_P", b)
    R("470", f"AUD{ch}_P", "GND", b)
    R("100", f"AUD{ch}_P", f"X_AUD_{ch}", b)
b.x -= 100
b.note("Pad: 2.1 Vrms DAC -> about 0.39 Vrms into a 600 ohm panel input, source impedance about 420 ohm."
       " DAC load stays above 1 kohm. PCM5102A outputs are ground-centred, so no DC blocking is needed.", 5, 185)

# ================================================================= 7. RS-232
b = Block("7  RS-232 (EFIS in/out, GPS in): SP3232E", 830, 20, 260, 110)
part("rs232", "U", "Interface_UART", "MAX3232",
     {"1": "RS_C1P", "2": "RS_VP", "3": "RS_C1N", "4": "RS_C2P", "5": "RS_C2N", "6": "RS_VN", "7": None,
      "8": "RS232_GPS_IN", "9": "GPS_RX", "10": "GND", "11": "EFIS_TX", "12": "EFIS_RX", "13": "RS232_EFIS_IN",
      "14": "RS232_EFIS_OUT", "15": "GND", "16": "+3V3"}, b.at(50, 55))
b.start_grid(15)
b.x += 105
C("100n", "RS_C1P", "RS_C1N", b); C("100n", "RS_C2P", "RS_C2N", b); C("100n", "RS_VP", "GND", b)
C("100n", "RS_VN", "GND", b); C("100n", "+3V3", "GND", b)
b.x -= 105
b.note("Unused T2IN tied low so it cannot float.", 5, 105)

# ================================================================= 8. CAN-FiX
b = Block("8  CAN-FiX (MakerPlane): TJA1051T/3, DE-9 male", 830, 140, 260, 110)
part("can", "U", "Interface_CAN_LIN", "TJA1051T-3",
     {"1": "CAN_TX", "2": "GND", "3": "+5V", "4": "CAN_RX", "5": "+3V3", "6": "CANL", "7": "CANH", "8": "CAN_S"}, b.at(40, 45))
part("canesd", "D", "Power_Protection", "NUP2105L", {"1": "CANH", "2": "CANL", "3": "GND"}, b.at(100, 45))
part("de9", "J", "Connector", "DE9_Plug_MountingHoles",
     {"0": "GND", "1": None, "2": "CANL", "3": "GND", "4": None, "5": "GND", "6": None, "7": "CANH", "8": "GND", "9": None},
     b.at(225, 50))
add(Symbol(nxt("JP"), "Jumper", "SolderJumper_2_Open", "CAN term", "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
           {"Note": "bridge on the last node of the bus"}, {"1": "CANH", "2": "CAN_TH"}, b.at(140, 20), 0, in_bom=False))
b.start_grid(75)
R("120", "CAN_TH", "CAN_MID", b); R("120", "CAN_TH", "CAN_MID", b); R("120", "CAN_MID", "CANL", b); R("120", "CAN_MID", "CANL", b)  # 2 x 60 R split termination
C("4.7n", "CAN_MID", "GND", b)
C("100n", "+5V", "GND", b); C("4.7u", "+5V", "GND", b); C("100n", "+3V3", "GND", b)
R("10k", "CAN_S", "GND", b, "S low = normal mode; firmware can set listen-only")
b.note("DE-9: 2 CAN-L, 7 CAN-H, 3 CAN GND, 5 GND, 8 shield; pin 1 bus power not used.", 5, 105)

# ================================================================= 9. microSD logging
b = Block("9  microSD logging (1-bit SDMMC)", 830, 260, 260, 110)
part("sd", "J", "Connector", "Micro_SD_Card_Det2",
     {"1": "SD_D2", "2": "SD_D3", "3": "SD_CMD", "4": "+3V3", "5": "SD_CLK_C", "6": "GND", "7": "SD_D0", "8": "SD_D1",
      "9": "SD_DET", "10": "GND", "11": "GND"}, b.at(40, 50))
b.start_grid(15)
b.x += 95
R("33", "SD_CLK", "SD_CLK_C", b, "edge-rate damping")
for net in ("SD_CMD", "SD_D0", "SD_D1", "SD_D2", "SD_D3", "SD_DET"):
    R("10k", "+3V3", net, b)
C("10u", "+3V3", "GND", b); C("100n", "+3V3", "GND", b)
b.x -= 95

# ================================================================= 10. mechanical
b = Block("10  Mechanical: grounded M3 holes, fiducials", 830, 380, 260, 30)
for _ in range(4):
    add(Symbol(nxt("H"), "Mechanical", "MountingHole_Pad", "M3", "MountingHole:MountingHole_3.2mm_M3_Pad_Via", {},
               {"1": "GND"}, b.slot(), 0, in_bom=False))
for _ in range(3):
    add(Symbol(nxt("FID"), "Mechanical", "Fiducial", "Fiducial", "Fiducial:Fiducial_1mm_Mask2mm", {}, {}, b.slot(), 0,
               in_bom=False))
