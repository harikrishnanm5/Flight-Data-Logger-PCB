"""
Master Hardware Design Specification for Crash-Safe UAV Flight Data Logger PCB.
Single Source of Truth shared by KiCad 6.0 Schematic & 4-Layer PCB generators.

Board: 50.0 mm x 50.0 mm, 4-Layer (F.Cu, In1.Cu=GND, In2.Cu=+3V3/PWR, B.Cu)
Mounting: Standard 30.5 mm x 30.5 mm M3 UAV stack (3.2 mm drill, 6.4 mm pad)
"""

BOARD_W = 50.0
BOARD_H = 50.0
CORNER_R = 2.5

# Components list:
# (ref, value, sym_lib, sym_name, fp_lib, fp_name, side, x, y, rot, mpn, desc, pin_nets)
# Note: side is 'F' (Top) or 'B' (Bottom). THT parts are mounted on 'F' and pierce all layers.

COMPONENTS = [
    # --- Mechanical Mounting Holes (30.5mm x 30.5mm UAV FC stack, M3) ---
    ("H1", "M3_GND", "Mechanical", "MountingHole_Pad", "MountingHole", "MountingHole_3.2mm_M3_Pad_TopBottom", "F", 9.75, 9.75, 0, "M3_HOLE", "M3 Drone Stack Hole NW", {"1": "GND"}),
    ("H2", "M3_GND", "Mechanical", "MountingHole_Pad", "MountingHole", "MountingHole_3.2mm_M3_Pad_TopBottom", "F", 40.25, 9.75, 0, "M3_HOLE", "M3 Drone Stack Hole NE", {"1": "GND"}),
    ("H3", "M3_GND", "Mechanical", "MountingHole_Pad", "MountingHole", "MountingHole_3.2mm_M3_Pad_TopBottom", "F", 9.75, 40.25, 0, "M3_HOLE", "M3 Drone Stack Hole SW", {"1": "GND"}),
    ("H4", "M3_GND", "Mechanical", "MountingHole_Pad", "MountingHole", "MountingHole_3.2mm_M3_Pad_TopBottom", "F", 40.25, 40.25, 0, "M3_HOLE", "M3 Drone Stack Hole SE", {"1": "GND"}),

    # --- Top-Side Major ICs, Modules, Connectors, Sensors, Supercap ---
    ("U2", "ESP32-WROOM-32", "RF_Module", "ESP32-WROOM-32", "RF_Module", "ESP32-WROOM-32", "F", 25.0, 15.8, 0, "ESP32-WROOM-32E-N4", "Wi-Fi/BLE Telemetry Module", {
        "1": "GND", "2": "+3V3", "3": "ESP_EN",
        "15": "GND", "25": "ESP_IO0", "34": "ESP_RXD0", "35": "ESP_TXD0",
        "38": "GND", "39": "GND",
    }),

    ("U1", "STM32F446RET6", "MCU_ST_STM32F4", "STM32F446RETx", "Package_QFP", "LQFP-64_10x10mm_P0.5mm", "F", 25.0, 33.4, 0, "STM32F446RET6", "ARM Cortex-M4F MCU 180MHz 512KB", {
        "1": "+3V3",
        "2": "MPU_INT",
        "3": "LSE_IN",
        "4": "LSE_OUT",
        "5": "HSE_IN",
        "6": "HSE_OUT",
        "7": "NRST",
        "12": "GND",
        "13": "+3.3VA",
        "14": "V_BAT_ADC",
        "18": "GND",
        "19": "+3V3",
        "21": "SPI1_SCK",
        "22": "SPI1_MISO",
        "23": "SPI1_MOSI",
        "24": "ESP_EN",
        "25": "STAT_LED",
        "26": "BEACON_CTRL",
        "27": "PFD_INT",
        "30": "VCAP_1",
        "31": "GND",
        "32": "+3V3",
        "33": "FRAM_CS",
        "42": "USART1_TX",
        "43": "USART1_RX",
        "46": "SWDIO",
        "47": "GND",
        "48": "+3V3",
        "49": "SWCLK",
        "58": "SD_CS",
        "59": "SD_CD",
        "60": "BOOT0",
        "61": "I2C1_SCL",
        "62": "I2C1_SDA",
        "63": "GND",
        "64": "+3V3",
    }),

    ("U3", "MPU-6050", "Sensor_Motion", "MPU-6050", "Sensor_Motion", "InvenSense_QFN-24_4x4mm_P0.5mm", "F", 17.5, 45.2, 0, "MPU-6050", "6-Axis IMU (I2C 0x68, AD0=GND)", {
        "1": "GND", "8": "+3V3", "9": "GND", "10": "MPU_REGOUT", "11": "GND",
        "12": "MPU_INT", "13": "+3V3", "18": "GND", "20": "MPU_CPOUT",
        "23": "I2C1_SCL", "24": "I2C1_SDA",
    }),

    ("U4", "BMP280", "Sensor_Pressure", "BMP280", "Package_LGA", "Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering", "F", 24.0, 45.4, 0, "BMP280", "Barometric Pressure Sensor (I2C 0x76)", {
        "1": "GND", "2": "+3V3", "3": "I2C1_SDA", "4": "I2C1_SCL",
        "5": "GND", "6": "+3V3", "7": "GND", "8": "+3V3",
    }),

    ("J3", "MicroSD_SPI", "Connector", "Micro_SD_Card_Det", "Connector_Card", "microSD_HC_Hirose_DM3D-SF", "F", 7.0, 25.0, -90, "DM3D-SF", "MicroSD Card Push-Pull Socket (SPI)", {
        "1": "SD_DAT2", "2": "SD_CS", "3": "SPI1_MOSI", "4": "+3V3",
        "5": "SPI1_SCK", "6": "GND", "7": "SPI1_MISO", "8": "SD_DAT1",
        "9": "GND", "10": "SD_CD", "11": "GND",
    }),

    ("C_SC1", "1.0F_5.5V", "Device", "C_Polarized", "Capacitor_THT", "DX_5R5VxxxxU_D11.5mm_P5.00mm", "F", 41.2, 16.8, 0, "DX-5R5V105U", "5.5V 1.0F Backup Supercapacitor", {
        "1": "V_SCAP", "2": "GND",
    }),

    ("J1", "JST_PH_BATT", "Connector_Generic", "Conn_01x02", "Connector_JST", "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", "F", 43.0, 31.8, 180, "B2B-PH-K-S", "3.7V Li-ion Input JST-PH 2-Pin", {
        "1": "V_BATT_RAW", "2": "GND",
    }),

    ("J2", "SWD_5P", "Connector_Generic", "Conn_01x05", "Connector_PinHeader_2.54mm", "PinHeader_1x05_P2.54mm_Vertical", "F", 47.2, 36.5, 0, "HDR-1x5-2.54", "SWD Debug (3V3, SWCLK, GND, SWDIO, NRST)", {
        "1": "+3V3", "2": "SWCLK", "3": "GND", "4": "SWDIO", "5": "NRST",
    }),

    ("J4", "ESP_UART_6P", "Connector_Generic", "Conn_01x06", "Connector_PinHeader_1.27mm", "PinHeader_1x06_P1.27mm_Vertical", "F", 2.8, 38.8, 0, "HDR-1x6-1.27", "ESP32 UART/Flash 6-Pin Header", {
        "1": "+3V3", "2": "GND", "3": "USART1_TX", "4": "USART1_RX", "5": "ESP_EN", "6": "ESP_IO0",
    }),

    ("SW1", "RESET", "Switch", "SW_Push", "Button_Switch_SMD", "SW_SPST_CK_KXT3", "F", 37.6, 47.2, 0, "KXT311LHS", "MCU Reset Tactile Switch", {
        "1": "NRST", "2": "GND",
    }),

    ("D1", "BEACON_LED_HI", "Device", "LED", "LED_SMD", "LED_PLCC_2835", "F", 32.0, 47.4, 0, "SM2835-RED-HI", "High-Intensity Optical Emergency Beacon LED", {
        "1": "BEACON_K", "2": "BEACON_A",
    }),

    ("D2", "STAT_GREEN", "Device", "LED", "LED_SMD", "LED_0603_1608Metric", "F", 5.8, 47.6, 0, "LTST-C190KGKT", "Status / Logger Activity LED (PC5)", {
        "1": "GND", "2": "STAT_LED_A",
    }),

    ("D3", "PWR_BLUE", "Device", "LED", "LED_SMD", "LED_0603_1608Metric", "F", 42.5, 47.6, 0, "LTST-C190TBKT", "3.3V Power Indicator LED", {
        "1": "GND", "2": "PWR_LED_A",
    }),

    # --- Top-Side MCU Clocks, Pull-ups, Beacon FET, and Critical Close Decoupling ---
    ("Y1", "8MHz", "Device", "Crystal_GND24", "Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm", "F", 16.2, 35.1, 90, "ABM8-8.000MHZ-B2-T", "8 MHz HSE Crystal", {
        "1": "HSE_IN", "2": "GND", "3": "HSE_OUT", "4": "GND",
    }),
    ("Y2", "32.768kHz", "Device", "Crystal_Small", "Crystal", "Crystal_SMD_3215-2Pin_3.2x1.5mm", "F", 16.7, 29.2, 90, "ABS07-32.768KHZ-T", "32.768 kHz LSE RTC Crystal", {
        "1": "LSE_IN", "2": "LSE_OUT",
    }),
    ("C3", "18pF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 15.0, 38.9, 90, "CC0603JRNPO9BN180", "HSE Load Cap 1", {"1": "HSE_IN", "2": "GND"}),
    ("C4", "18pF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 16.8, 38.9, 90, "CC0603JRNPO9BN180", "HSE Load Cap 2", {"1": "HSE_OUT", "2": "GND"}),
    ("C5", "12pF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 14.6, 28.0, 90, "CC0603JRNPO9BN120", "LSE Load Cap 1", {"1": "LSE_IN", "2": "GND"}),
    ("C6", "12pF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 14.6, 31.3, 90, "CC0603JRNPO9BN120", "LSE Load Cap 2", {"1": "LSE_OUT", "2": "GND"}),

    # MicroSD Pull-ups & Filter Caps (above & below J3)
    ("R5", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 2.5, 16.0, 0, "RC0603FR-0710KL", "SD_CS Pull-up", {"1": "+3V3", "2": "SD_CS"}),
    ("R6", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 6.0, 16.0, 0, "RC0603FR-0710KL", "SPI1_MISO Pull-up", {"1": "+3V3", "2": "SPI1_MISO"}),
    ("R7", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 9.5, 16.0, 0, "RC0603FR-0710KL", "SPI1_MOSI Pull-up", {"1": "+3V3", "2": "SPI1_MOSI"}),
    ("R8", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 13.0, 16.0, 0, "RC0603FR-0710KL", "SD_DAT1 Pull-up", {"1": "+3V3", "2": "SD_DAT1"}),
    ("R9", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 2.0, 34.0, 0, "RC0603FR-0710KL", "SD_DAT2 Pull-up", {"1": "+3V3", "2": "SD_DAT2"}),
    ("R10", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 5.2, 34.0, 0, "RC0603FR-0710KL", "SD_CD Pull-up", {"1": "+3V3", "2": "SD_CD"}),
    ("C21", "10uF", "Device", "C", "Capacitor_SMD", "C_0805_2012Metric", "F", 8.8, 34.0, 0, "CL21A106KAYNNNE", "SD VDD Bulk Cap", {"1": "+3V3", "2": "GND"}),
    ("C22", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 12.2, 34.0, 0, "CC0603KRX7R7BB104", "SD VDD Decoupling Cap", {"1": "+3V3", "2": "GND"}),

    # Beacon Driver & Support on Top Bottom-Right
    ("Q4", "2N7002", "Transistor_FET", "2N7002", "Package_TO_SOT_SMD", "SOT-23", "F", 34.0, 42.8, 0, "2N7002LT1G", "Beacon N-MOSFET Driver", {"1": "BEACON_G", "2": "GND", "3": "BEACON_K"}),
    ("R18", "22R", "Device", "R", "Resistor_SMD", "R_0805_2012Metric", "F", 35.2, 38.6, 90, "ERJ-6GEYJ220V", "Beacon LED Current Limit", {"1": "+3V3", "2": "BEACON_A"}),
    ("R19", "100R", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 36.2, 35.4, 180, "RC0603FR-07100RL", "Beacon Gate Series R", {"1": "BEACON_CTRL", "2": "BEACON_G"}),
    ("R20", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 29.8, 41.6, 0, "RC0603FR-0710KL", "Beacon Gate Pull-Down", {"1": "BEACON_G", "2": "GND"}),
    ("R21", "1k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 6.6, 45.2, 0, "RC0603FR-071KL", "Status LED Resistor", {"1": "STAT_LED", "2": "STAT_LED_A"}),
    ("R22", "1k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 42.5, 45.5, 0, "RC0603FR-071KL", "Power LED Resistor", {"1": "+3V3", "2": "PWR_LED_A"}),

    # I2C1 Pull-ups & IMU/BMP Caps on Top
    ("R3", "4.7k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 27.5, 43.8, 0, "RC0603FR-074K7L", "I2C1 SCL Pull-up", {"1": "+3V3", "2": "I2C1_SCL"}),
    ("R4", "4.7k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "F", 27.5, 46.8, 0, "RC0603FR-074K7L", "I2C1 SDA Pull-up", {"1": "+3V3", "2": "I2C1_SDA"}),
    ("C17", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 10.0, 45.4, 0, "CC0603KRX7R7BB104", "MPU-6050 REGOUT Cap", {"1": "MPU_REGOUT", "2": "GND"}),
    ("C18", "2.2nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 12.6, 47.8, 0, "CC0603KRX7R9BB222", "MPU-6050 CPOUT Charge Pump Cap", {"1": "MPU_CPOUT", "2": "GND"}),
    ("C19", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "F", 24.0, 48.2, 0, "CC0603KRX7R7BB104", "BMP280 VDD Decoupling", {"1": "+3V3", "2": "GND"}),

    # --- Bottom-Side (B.Cu) FRAM, Power Management, Ideal Diodes, PFD, ADC Divider, & Decoupling ---
    # Note: Keep B.Cu components clear of THT pins: H1..H4, C_SC1 (41.5,18..23), J1 (41..45, 31.8), J2 (47.2, 36.5), J4 (2.8, 38.8..45.2)
    ("U5", "CY15B102QN-50SXIT", "Memory_NVRAM", "MB85RS2MT", "Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm", "B", 8.0, 31.0, 0, "CY15B102QN-50SXIT", "2 Mbit SPI FRAM Non-Volatile Memory", {
        "1": "FRAM_CS", "2": "SPI1_MISO", "3": "+3V3", "4": "GND",
        "5": "SPI1_MOSI", "6": "SPI1_SCK", "7": "+3V3", "8": "+3V3",
    }),
    ("R11", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 5.2, 35.3, 0, "RC0603FR-0710KL", "FRAM_CS Pull-up", {"1": "+3V3", "2": "FRAM_CS"}),
    ("C20", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 4.0, 26.8, 0, "CC0603KRX7R7BB104", "FRAM VDD Decoupling", {"1": "+3V3", "2": "GND"}),

    # Power Input Reverse-Polarity + Ideal Diode OR-ing + Supercap Charger + 3.3V Buck (Right Half of B.Cu)
    ("Q1", "AO3401A", "Transistor_FET", "AO3401A", "Package_TO_SOT_SMD", "SOT-23", "B", 38.5, 26.5, 0, "AO3401A", "Reverse-Polarity P-MOSFET", {"1": "Q1_G", "2": "V_BAT_PROT", "3": "V_BATT_RAW"}),
    ("R23", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 35.0, 26.5, 90, "RC0603FR-0710KL", "Reverse-Polarity Gate PD", {"1": "Q1_G", "2": "GND"}),
    ("C26", "1uF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 35.0, 23.0, 90, "CL10A105KB8NNNC", "Fast-Trip Input Cap", {"1": "V_BAT_PROT", "2": "GND"}),

    ("U7", "LTC4412ES6", "Power_Management", "LTC4412xS6", "Package_TO_SOT_SMD", "TSOT-23-6", "B", 33.2, 19.5, 0, "LTC4412ES6#TRPBF", "Primary Battery PowerPath Ideal Diode Ctrl", {"1": "V_BAT_PROT", "2": "GND", "3": "PFD_INT", "5": "GATE_BAT", "6": "V_SYS"}),
    ("Q2", "AO3401A", "Transistor_FET", "AO3401A", "Package_TO_SOT_SMD", "SOT-23", "B", 37.3, 19.4, 0, "AO3401A", "Primary Battery Ideal Diode P-MOSFET", {"1": "GATE_BAT", "2": "V_SYS", "3": "V_BAT_PROT"}),

    ("U8", "LTC4412ES6", "Power_Management", "LTC4412xS6", "Package_TO_SOT_SMD", "TSOT-23-6", "B", 33.2, 9.2, 0, "LTC4412ES6#TRPBF", "Supercap Backup Ideal Diode OR-ing Ctrl", {"1": "V_SCAP", "2": "GND", "3": "GND", "5": "GATE_SCAP", "6": "V_SYS"}),
    ("Q3", "AO3401A", "Transistor_FET", "AO3401A", "Package_TO_SOT_SMD", "SOT-23", "B", 35.8, 15.2, 0, "AO3401A", "Supercap Backup Ideal Diode P-MOSFET", {"1": "GATE_SCAP", "2": "V_SYS", "3": "V_SCAP"}),

    ("U11", "AP22814AW5", "Power_Management", "AP22814AW5", "Package_TO_SOT_SMD", "SOT-23-5", "B", 45.8, 14.5, 0, "AP22814AW5-7", "Slew-Limited Supercap Charge Switch", {"1": "SCAP_SW_OUT", "2": "GND", "3": "SCAP_FLG", "4": "SCAP_CHG_EN", "5": "V_BAT_PROT"}),
    ("R24", "15R", "Device", "R", "Resistor_SMD", "R_0805_2012Metric", "B", 46.2, 19.5, 90, "ERJ-6GEYJ150V", "Supercap Charge Current Limit (~200mA)", {"1": "SCAP_SW_OUT", "2": "D4_A"}),
    ("D4", "B5819W", "Device", "D_Schottky", "Diode_SMD", "D_SOD-123", "B", 46.2, 24.8, 90, "B5819W-7-F", "Supercap Blocking Schottky Diode", {"1": "V_SCAP", "2": "D4_A"}),
    ("R25", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 45.8, 10.5, 0, "RC0603FR-0710KL", "Supercap Charger EN Pull-up", {"1": "V_BAT_PROT", "2": "SCAP_CHG_EN"}),
    ("R26", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 45.8, 8.2, 0, "RC0603FR-0710KL", "Supercap Charger FLG Pull-up", {"1": "+3V3", "2": "SCAP_FLG"}),

    # 3.3V Synchronous Buck Regulator (B.Cu, Center-Right below ESP32 shield, clear of thermal vias)
    ("U6", "TPS62822DLC", "Regulator_Switching", "TPS62822DLC", "Package_DFN_QFN", "Texas_VSON-HR-8_1.5x2mm_P0.5mm", "B", 28.8, 16.5, 0, "TPS62822DLCR", "3.3V 2A Synchronous Buck Regulator", {
        "1": "V_SYS", "2": "BUCK_FB", "3": "GND", "5": "GND", "6": "BUCK_SW", "7": "V_SYS", "8": "BUCK_PG",
    }),
    ("L1", "1.0uH_3.5A", "Device", "L", "Inductor_SMD", "L_Wuerth_MAPI-3015", "B", 28.4, 12.2, 0, "74438334010", "1.0uH Shielded Buck Power Inductor", {"1": "BUCK_SW", "2": "+3V3"}),
    ("C27", "10uF", "Device", "C", "Capacitor_SMD", "C_0805_2012Metric", "B", 28.3, 19.8, 0, "CL21A106KAYNNNE", "Buck Input Decoupling Cap", {"1": "V_SYS", "2": "GND"}),
    ("C28", "22uF", "Device", "C", "Capacitor_SMD", "C_0805_2012Metric", "B", 24.5, 19.8, 0, "CL21A226MAYNNNE", "Buck Output Bulk Cap 1", {"1": "+3V3", "2": "GND"}),
    ("C29", "22uF", "Device", "C", "Capacitor_SMD", "C_0805_2012Metric", "B", 20.8, 19.8, 0, "CL21A226MAYNNNE", "Buck Output Bulk Cap 2", {"1": "+3V3", "2": "GND"}),
    ("R27", "45.3k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 28.8, 22.4, 0, "RC0603FR-0745K3L", "Buck FB Top (3.318V)", {"1": "+3V3", "2": "BUCK_FB"}),
    ("R28", "10.0k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 25.0, 22.4, 0, "RC0603FR-0710KL", "Buck FB Bottom", {"1": "BUCK_FB", "2": "GND"}),
    ("C30", "22pF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 21.2, 22.4, 0, "CC0603JRNPO9BN220", "Buck FB Feedforward Cap", {"1": "+3V3", "2": "BUCK_FB"}),
    ("R29", "100k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 32.3, 22.8, 0, "RC0603FR-07100KL", "Buck PG Pull-up", {"1": "+3V3", "2": "BUCK_PG"}),

    # Power Fail Detect (PFD) Comparator & Precision 2.5V Reference (B.Cu, Bottom-Right)
    ("U9", "TLV7031DBV", "Comparator", "TLV7031DBV", "Package_TO_SOT_SMD", "SOT-23-5", "B", 34.4, 34.4, 0, "TLV7031DBVR", "PFD Comparator (3.00V Trip -> PB1 EXTI1)", {
        "1": "PFD_INT", "2": "GND", "3": "PFD_REF", "4": "PFD_DIV", "5": "+3V3",
    }),
    ("U10", "LM4040DBZ-2.5", "Reference_Voltage", "LM4040DBZ-2.5", "Package_TO_SOT_SMD", "SOT-23", "B", 34.0, 41.8, 0, "LM4040AIM3-2.5", "2.500V Precision Shunt Reference", {
        "1": "V_REF_2V5", "2": "GND",
    }),
    ("R30", "1.5k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 34.0, 45.2, 0, "RC0603FR-071K5L", "LM4040 Bias Resistor", {"1": "+3V3", "2": "V_REF_2V5"}),
    ("R31", "10.0k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 30.4, 42.8, 90, "RC0603FR-0710KL", "PFD Ref Divider Top (1.250V)", {"1": "V_REF_2V5", "2": "PFD_REF"}),
    ("R32", "10.0k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 30.4, 46.0, 90, "RC0603FR-0710KL", "PFD Ref Divider Bottom", {"1": "PFD_REF", "2": "GND"}),
    ("R33", "14.0k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 34.2, 29.2, 0, "RC0603FR-0714KL", "PFD Sense Divider Top (3.00V)", {"1": "V_BAT_PROT", "2": "PFD_DIV"}),
    ("R34", "10.0k_1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 34.2, 31.4, 180, "RC0603FR-0710KL", "PFD Sense Divider Bottom", {"1": "PFD_DIV", "2": "GND"}),
    ("R35", "470k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 37.4, 31.0, 90, "RC0603FR-07470KL", "PFD Positive Hysteresis (~53mV)", {"1": "PFD_INT", "2": "PFD_REF"}),
    ("R36", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 37.4, 34.8, 90, "RC0603FR-0710KL", "PFD_INT Cold-Start Pull-Down", {"1": "PFD_INT", "2": "GND"}),
    ("C31", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 34.0, 38.2, 0, "CC0603KRX7R7BB104", "Comparator Supply Cap", {"1": "+3V3", "2": "GND"}),
    ("C32", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 37.8, 46.5, 0, "CC0603KRX7R7BB104", "V_REF_2V5 Filter Cap", {"1": "V_REF_2V5", "2": "GND"}),

    # ADC1 Battery Voltage Divider (0.1% Tolerance) & Analog Filter (B.Cu, Bottom-Left-Center near PA0/VDDA)
    ("R1", "10k_0.1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 25.8, 38.5, 0, "ERA-3AEB103V", "ADC1 Divider Top (0.1%)", {"1": "V_BAT_PROT", "2": "V_BAT_ADC"}),
    ("R2", "2k_0.1%", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 22.4, 38.5, 0, "ERA-3AEB202V", "ADC1 Divider Bottom (0.1%)", {"1": "V_BAT_ADC", "2": "GND"}),
    ("C1", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 19.0, 38.5, 0, "CC0603KRX7R7BB104", "ADC1 PA0 Decoupling Cap", {"1": "V_BAT_ADC", "2": "GND"}),
    ("FB1", "600R@100MHz", "Device", "FerriteBead_Small", "Inductor_SMD", "L_0603_1608Metric", "B", 16.0, 28.0, 0, "BLM18AG601SN1D", "VDDA Analog Filter Bead", {"1": "+3V3", "2": "+3.3VA"}),
    ("C13", "1uF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 16.5, 36.0, 0, "CL10A105KB8NNNC", "VDDA Bulk Cap", {"1": "+3.3VA", "2": "GND"}),
    ("C14", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 16.5, 33.2, 0, "CC0603KRX7R7BB104", "VDDA Bypass Cap", {"1": "+3.3VA", "2": "GND"}),

    # MCU Core/VDD Decoupling, Boot, Reset & ESP32 Support (B.Cu centered cleanly inside U1 & U2 shields)
    ("C7", "2.2uF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 28.5, 35.5, 0, "CL10A225KQ8NNNC", "STM32 VCAP_1 Core Cap", {"1": "VCAP_1", "2": "GND"}),
    ("C8", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 21.0, 32.0, 0, "CC0603KRX7R7BB104", "MCU VDD1 Decoupling", {"1": "+3V3", "2": "GND"}),
    ("C9", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 25.0, 32.0, 0, "CC0603KRX7R7BB104", "MCU VDD2 Decoupling", {"1": "+3V3", "2": "GND"}),
    ("C10", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 29.0, 32.0, 0, "CC0603KRX7R7BB104", "MCU VDD3 Decoupling", {"1": "+3V3", "2": "GND"}),
    ("C11", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 21.0, 35.5, 0, "CC0603KRX7R7BB104", "MCU VDD4 Decoupling", {"1": "+3V3", "2": "GND"}),
    ("C12", "4.7uF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 25.0, 35.5, 0, "CL10A475KQ8NNNC", "MCU VDD Bulk Cap", {"1": "+3V3", "2": "GND"}),
    ("R12", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 19.0, 24.8, 0, "RC0603FR-0710KL", "MCU BOOT0 Pull-Down", {"1": "BOOT0", "2": "GND"}),
    ("R13", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 21.0, 29.2, 0, "RC0603FR-0710KL", "MCU NRST Pull-up", {"1": "+3V3", "2": "NRST"}),
    ("C2", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 25.0, 29.2, 0, "CC0603KRX7R7BB104", "MCU NRST Filter Cap", {"1": "NRST", "2": "GND"}),
    ("C15", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 20.5, 42.3, 0, "CC0603KRX7R7BB104", "MPU-6050 VDD Bypass", {"1": "+3V3", "2": "GND"}),
    ("C16", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 20.5, 44.4, 0, "CC0603KRX7R7BB104", "MPU-6050 VDDIO Bypass", {"1": "+3V3", "2": "GND"}),

    # ESP32 Support Caps & UART Resistors (B.Cu under U2 body y=10..23, clearing U1-U2 throat y=24..29)
    ("C23", "22uF", "Device", "C", "Capacitor_SMD", "C_0805_2012Metric", "B", 15.5, 19.0, 90, "CL21A226MAYNNNE", "ESP32 VDD Bulk Cap", {"1": "+3V3", "2": "GND"}),
    ("C24", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 18.0, 19.0, 90, "CC0603KRX7R7BB104", "ESP32 VDD Bypass Cap", {"1": "+3V3", "2": "GND"}),
    ("C25", "100nF", "Device", "C", "Capacitor_SMD", "C_0603_1608Metric", "B", 16.5, 23.0, 0, "CC0603KRX7R7BB104", "ESP32 EN RC Cap", {"1": "ESP_EN", "2": "GND"}),
    ("R14", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 16.0, 14.5, 0, "RC0603FR-0710KL", "ESP32 EN Pull-up", {"1": "+3V3", "2": "ESP_EN"}),
    ("R15", "10k", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 30.4, 24.8, 0, "RC0603FR-0710KL", "ESP32 IO0 Pull-up", {"1": "+3V3", "2": "ESP_IO0"}),
    ("R16", "100R", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 19.0, 8.8, 0, "RC0603FR-07100RL", "USART1 TX -> ESP RXD0 Series", {"1": "USART1_TX", "2": "ESP_RXD0"}),
    ("R17", "100R", "Device", "R", "Resistor_SMD", "R_0603_1608Metric", "B", 23.0, 8.8, 0, "RC0603FR-07100RL", "ESP TXD0 -> USART1 RX Series", {"1": "ESP_TXD0", "2": "USART1_RX"}),
]

# Net classification for PCB trace widths & clearances
POWER_NETS = {
    "V_BATT_RAW", "V_BAT_PROT", "V_SYS", "V_SCAP", "SCAP_SW_OUT", "D4_A",
    "BUCK_SW", "+3V3", "+3.3VA", "GND", "BEACON_A", "BEACON_K"
}
FINE_NETS = {
    "I2C1_SCL", "I2C1_SDA", "SPI1_SCK", "SPI1_MISO", "SPI1_MOSI",
    "SD_CS", "SD_CD", "SD_DAT1", "SD_DAT2", "FRAM_CS",
    "USART1_TX", "USART1_RX", "ESP_RXD0", "ESP_TXD0", "ESP_EN", "ESP_IO0",
    "SWDIO", "SWCLK", "NRST", "HSE_IN", "HSE_OUT", "LSE_IN", "LSE_OUT",
    "MPU_INT", "MPU_REGOUT", "MPU_CPOUT", "PFD_INT", "V_BAT_ADC", "VCAP_1"
}

