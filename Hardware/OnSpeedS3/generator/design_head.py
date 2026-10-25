"""OnSpeed ESP32-S3 board: complete circuit, as data.

One source of truth: the schematic, the PCB nets, the BOM and the placement file are all generated
from the component list built here. Positions (x, y) are schematic coordinates in mm on an A1 sheet.
"""
from sch import Symbol
from boxsym import box_symbol

# ---------------------------------------------------------------- part catalogue
# value -> (LCSC, footprint). Passives are 0603 unless noted; JLC "basic" parts where possible.
RES = {  # 0603 1% thick film
    "33": "C23140", "100": "C22775", "150": "C22808", "470": "C23179", "560": "C23204", "1k": "C21190",
    "2k": "C22975", "3.3k": "C22978", "4.7k": "C23162", "5.1k": "C23186", "5.6k": "C23189", "10k": "C25804",
    "20k": "C4184", "22k": "C31850", "62": "C23205", "1M": "C22935", "0": "C21189", "100k": "C25803",
    "49.9k": "C23184", "8.2k": "C25981", "47k": "C25819",
}
CAP = {  # value -> (LCSC, footprint, rating)
    "100n": ("C14663", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "10n": ("C57112", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "4.7n": ("C53987", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "2.2n": ("C1604", "Capacitor_SMD:C_0603_1608Metric", "50V X7R"),
    "1u": ("C15849", "Capacitor_SMD:C_0603_1608Metric", "16V X7R"),
    "2.2u": ("C23630", "Capacitor_SMD:C_0603_1608Metric", "16V X5R"),
    "4.7u": ("C19666", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "10u": ("C15850", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "22u": ("C45783", "Capacitor_SMD:C_0805_2012Metric", "25V X5R"),
    "4.7u/50V": ("C2775055", "Capacitor_SMD:C_1206_3216Metric", "50V X7R"),
}
R0603 = "Resistor_SMD:R_0603_1608Metric"
DS = {
    "esp": "https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf",
    "pcm": "https://www.ti.com/lit/ds/symlink/pcm5102a.pdf",
    "ism": "https://www.st.com/resource/en/datasheet/ism330dhcx.pdf",
    "ms45": "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS4525DO&DocType=Data+Sheet&DocLang=English",
    "ssc": "https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/board-mount-pressure-sensors/trustability-ssc-series/documents/sps-siot-trustability-ssc-series-standard-accuracy-board-mount-pressure-sensors-50099533-a-en-ciid-151134.pdf",
}

