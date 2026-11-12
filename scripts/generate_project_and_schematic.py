"""
Generate KiCad 6.0 Project (.kicad_pro) and Schematic (.kicad_sch)
for the Crash-Safe UAV Flight Data Logger PCB.
"""
import json
import math
import os
import re
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
from design_spec import COMPONENTS, POWER_NETS, FINE_NETS

OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SYM_DIR = r"D:\FOSSEE\KiCad\share\kicad\symbols"

# Deterministic UUID generator so files are reproducible
_uuid_counter = 0
def make_uuid(seed=""):
    global _uuid_counter
    _uuid_counter += 1
    return str(uuid.uuid5(uuid.NAMESPACE_DNS, f"uav-fdl-{_uuid_counter}-{seed}"))


def find_sexp_end(txt, start):
    """Return end index (exclusive) of balanced S-expression starting at txt[start]=='('."""
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(txt)):
        ch = txt[i]
        if esc:
            esc = False
            continue
        if ch == "\\" and in_str:
            esc = True
            continue
        if ch == '"':
            in_str = not in_str
        elif not in_str:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    return i + 1
    raise RuntimeError(f"Unbalanced S-expression starting at {start}")


def extract_top_level_blocks(blk, prefix):
    """Extract direct child S-expression blocks starting with prefix (e.g. '(property ')."""
    results = []
    # Scan top-level body (after outer header) at depth == 1
    depth = 0
    in_str = False
    esc = False
    i = 0
    n = len(blk)
    while i < n:
        ch = blk[i]
        if esc:
            esc = False
            i += 1
            continue
        if ch == "\\" and in_str:
            esc = True
            i += 1
            continue
        if ch == '"':
            in_str = not in_str
            i += 1
            continue
        if not in_str:
            if ch == "(":
                if depth == 1 and blk.startswith(prefix, i):
                    end = find_sexp_end(blk, i)
                    results.append(blk[i:end])
                    i = end
                    continue
                depth += 1
            elif ch == ")":
                depth -= 1
        i += 1
    return results


def extract_symbol_block(lib_name, sym_name):
    path = os.path.join(SYM_DIR, f"{lib_name}.kicad_sym")
    txt = open(path, encoding="utf-8").read()
    # Find top-level (symbol "sym_name" ...) inside (kicad_symbol_lib ...)
    m = re.search(rf'\(symbol "{re.escape(sym_name)}"(?=[\s\)])', txt)
    if not m:
        raise RuntimeError(f"Symbol {lib_name}:{sym_name} not found")
    idx = m.start()
    return txt[idx:find_sexp_end(txt, idx)]


def resolve_symbol_raw(lib_name, sym_name):
    blk = extract_symbol_block(lib_name, sym_name)
    ext_m = re.search(r'\(extends "([^"]+)"\)', blk)
    if ext_m:
        parent_name = ext_m.group(1)
        parent_blk = resolve_symbol_raw(lib_name, parent_name)
        child_props = extract_top_level_blocks(blk, "(property ")
        # Preserve header tokens after symbol name (pin_names, in_bom, on_board, pin_numbers)
        header_tokens = []
        for tag in ("(pin_numbers ", "(pin_names ", "(in_bom ", "(on_board "):
            t_idx = parent_blk.find(tag)
            # Only use if at top level of parent_blk (before first property/symbol child)
            first_prop = parent_blk.find("(property ")
            if 0 <= t_idx < (first_prop if first_prop >= 0 else len(parent_blk)):
                header_tokens.append(parent_blk[t_idx:find_sexp_end(parent_blk, t_idx)])
        header_flags = " ".join(header_tokens)
        sub_syms = extract_top_level_blocks(parent_blk, "(symbol ")
        sub_body = "\n    ".join(
            s.replace(f'(symbol "{parent_name}_', f'(symbol "{sym_name}_', 1) for s in sub_syms
        )
        props_str = "\n    ".join(child_props)
        return f'(symbol "{sym_name}" {header_flags}\n    {props_str}\n    {sub_body}\n  )'
    return blk


def resolve_symbol_for_lib(lib_name, sym_name):
    """Extract symbol block from KiCad library and flatten any (extends "...") parent."""
    raw = resolve_symbol_raw(lib_name, sym_name)
    return raw.replace(f'(symbol "{sym_name}"', f'(symbol "{lib_name}:{sym_name}"', 1)


def parse_pins_from_symbol_block(blk):
    """Return list of (pin_num, pin_name, px, py, angle, length, ptype)."""
    pins = []
    idx = 0
    while True:
        pos = blk.find("(pin ", idx)
        if pos < 0:
            break
        end = find_sexp_end(blk, pos)
        pblk = blk[pos:end]
        idx = end
        ptype = pblk.split()[1]
        at_m = re.search(r'\(at\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\)', pblk)
        len_m = re.search(r'\(length\s+(-?\d+\.?\d*)\)', pblk)
        nm_m = re.search(r'\(name "([^"]*)"', pblk)
        num_m = re.search(r'\(number "([^"]*)"', pblk)
        if at_m and nm_m and num_m:
            pins.append((
                num_m.group(1),
                nm_m.group(1),
                float(at_m.group(1)),
                float(at_m.group(2)),
                float(at_m.group(3)),
                float(len_m.group(1)) if len_m else 2.54,
                ptype,
            ))
    return pins


def generate_kicad_pro():
    pro = {
        "board": {
            "design_settings": {
                "defaults": {
                    "board_outline_line_width": 0.1,
                    "copper_line_width": 0.25,
                    "copper_text_size_h": 1.2,
                    "copper_text_size_v": 1.2,
                    "copper_text_thickness": 0.25,
                    "other_line_width": 0.15,
                    "silk_line_width": 0.12,
                    "silk_text_size_h": 0.8,
                    "silk_text_size_v": 0.8,
                    "silk_text_thickness": 0.12
                },
                "diff_pair_dimensions": [],
                "drc_exclusions": [],
                "rules": {
                    "min_clearance": 0.15,
                    "min_copper_edge_clearance": 0.3,
                    "min_hole_clearance": 0.25,
                    "min_hole_to_hole": 0.5,
                    "min_microvia_diameter": 0.3,
                    "min_microvia_drill": 0.1,
                    "min_silk_clearance": 0.0,
                    "min_through_hole_diameter": 0.2,
                    "min_track_width": 0.15,
                    "min_via_annular_width": 0.125,
                    "min_via_diameter": 0.45,
                    "solder_mask_clearance": 0.05,
                    "solder_mask_min_width": 0.0
                },
                "track_widths": [0.0, 0.2, 0.25, 0.4, 0.6],
                "via_dimensions": [
                    {"diameter": 0.0, "drill": 0.0},
                    {"diameter": 0.6, "drill": 0.3},
                    {"diameter": 0.8, "drill": 0.4}
                ]
            },
            "layer_presets": [],
            "viewports": []
        },
        "boards": [],
        "libraries": {
            "pinned_footprint_libs": [],
            "pinned_symbol_libs": []
        },
        "meta": {
            "filename": "FlightDataLogger.kicad_pro",
            "version": 1
        },
        "net_settings": {
            "classes": [
                {
                    "clearance": 0.15,
                    "diff_pair_gap": 0.25,
                    "diff_pair_via_gap": 0.25,
                    "diff_pair_width": 0.2,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "name": "Default",
                    "nets": [],
                    "track_width": 0.25,
                    "via_diameter": 0.6,
                    "via_drill": 0.3
                },
                {
                    "clearance": 0.2,
                    "diff_pair_gap": 0.25,
                    "diff_pair_via_gap": 0.25,
                    "diff_pair_width": 0.2,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "name": "Power",
                    "nets": sorted(POWER_NETS),
                    "track_width": 0.5,
                    "via_diameter": 0.8,
                    "via_drill": 0.4
                },
                {
                    "clearance": 0.15,
                    "diff_pair_gap": 0.25,
                    "diff_pair_via_gap": 0.25,
                    "diff_pair_width": 0.2,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "name": "FinePitch",
                    "nets": sorted(FINE_NETS),
                    "track_width": 0.2,
                    "via_diameter": 0.55,
                    "via_drill": 0.3
                }
            ],
            "meta": {"version": 2}
        },
        "pcbnew": {"page_layout_descr_file": ""},
        "schematic": {
            "annotate_start_num": 0,
            "drawing": {
                "dashed_lines_dash_length_ratio": 12.0,
                "dashed_lines_gap_length_ratio": 3.0,
                "default_line_thickness": 6.0,
                "default_text_size": 50.0,
                "field_names": [],
                "intersheets_ref_own_page": False,
                "intersheets_ref_prefix": "",
                "intersheets_ref_short": False,
                "intersheets_ref_show": False,
                "intersheets_ref_suffix": "",
                "junction_size_choice": 3,
                "label_size_ratio": 0.375,
                "pin_symbol_size": 25.0,
                "text_offset_ratio": 0.15
            },
            "legacy_lib_dir": "",
            "legacy_lib_list": [],
            "meta": {"version": 1},
            "net_format_name": "",
            "page_layout_descr_file": "",
            "plot_directory": "",
            "spice_current_sheet_as_root": False,
            "spice_external_command": "spice \"%I\"",
            "spice_model_current_sheet_as_root": True,
            "spice_save_all_currents": False,
            "spice_save_all_voltages": False,
            "subpart_first_id": 65,
            "subpart_id_separator": 0
        },
        "sheets": [],
        "text_variables": {}
    }
    p = os.path.join(OUT_DIR, "FlightDataLogger.kicad_pro")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(pro, f, indent=2)
    print(f"Wrote {p}")


# Schematic A2 grid placements for each component ref (x_mm, y_mm) snapped to 2.54mm grid
def g(nx, ny):
    return (round(nx * 2.54, 2), round(ny * 2.54, 2))


SCH_LAYOUT = {
    # Section 1: Power Input, Reverse Polarity & PowerPath OR-ing (Top-Left)
    "J1": g(14, 14), "Q1": g(28, 14), "R23": g(28, 22), "C26": g(38, 18),
    "U7": g(22, 34), "Q2": g(36, 34),
    "U8": g(22, 48), "Q3": g(36, 48),
    "U11": g(54, 14), "R25": g(48, 22), "R26": g(62, 14), "R24": g(66, 14), "D4": g(74, 14), "C_SC1": g(82, 18),

    # Section 2: 3.3V Synchronous Buck Regulator (Mid-Left)
    "U6": g(22, 68), "L1": g(36, 65), "C27": g(12, 70), "C28": g(44, 70), "C29": g(50, 70),
    "R27": g(36, 76), "R28": g(36, 84), "C30": g(42, 76), "R29": g(12, 78),

    # Section 3: Power Fail Detect (PFD) Comparator & 2.5V Ref (Bottom-Left)
    "U9": g(30, 102), "U10": g(14, 114), "R30": g(14, 104), "C32": g(20, 114),
    "R31": g(22, 98), "R32": g(22, 106), "R33": g(12, 94), "R34": g(12, 102),
    "R35": g(38, 96), "R36": g(44, 104), "C31": g(30, 112),

    # Section 4: Central MCU STM32F446RET6 + Clocks + Reset + SWD + Decoupling (Center)
    "U1": g(110, 56),
    "Y1": g(74, 42), "C3": g(70, 50), "C4": g(78, 50),
    "Y2": g(74, 30), "C5": g(70, 36), "C6": g(78, 36),
    "SW1": g(74, 62), "R13": g(82, 58), "C2": g(82, 66),
    "R12": g(144, 62),
    "FB1": g(74, 76), "C13": g(80, 78), "C14": g(86, 78),
    "C7": g(94, 92), "C8": g(104, 92), "C9": g(110, 92), "C10": g(116, 92), "C11": g(122, 92), "C12": g(128, 92),
    "J2": g(144, 42),

    # Section 5: ADC1 Battery Divider & Emergency Optical Beacon & Status LEDs (Bottom-Center)
    "R1": g(74, 102), "R2": g(74, 110), "C1": g(82, 110),
    "Q4": g(110, 116), "D1": g(120, 108), "R18": g(128, 108), "R19": g(100, 116), "R20": g(102, 124),
    "D2": g(140, 108), "R21": g(134, 108), "D3": g(140, 118), "R22": g(134, 118),

    # Section 6: Shared I2C1 Sensors - MPU-6050 & BMP280 (Top-Right)
    "U3": g(184, 24), "C15": g(168, 20), "C16": g(168, 28), "C17": g(198, 24), "C18": g(198, 34),
    "U4": g(184, 52), "C19": g(168, 52),
    "R3": g(168, 38), "R4": g(174, 38),

    # Section 7: Shared SPI1 Storage - MicroSD & CY15B102QN FRAM (Mid-Right)
    "J3": g(184, 78), "R5": g(164, 70), "R6": g(164, 78), "R7": g(164, 86),
    "R8": g(170, 70), "R9": g(170, 78), "R10": g(170, 86), "C21": g(200, 74), "C22": g(206, 74),
    "U5": g(184, 104), "R11": g(168, 104), "C20": g(200, 104),

    # Section 8: ESP32-WROOM-32 Wireless & 6-pin UART Header (Far-Right)
    "U2": g(244, 48), "C23": g(220, 26), "C24": g(226, 26), "C25": g(220, 36),
    "R14": g(220, 44), "R15": g(266, 44), "R16": g(266, 28), "R17": g(266, 36),
    "J4": g(244, 96),

    # Section 9: Mechanical Mounting Holes (Bottom-Right)
    "H1": g(220, 114), "H2": g(230, 114), "H3": g(240, 114), "H4": g(250, 114),
}


def generate_kicad_sch():
    # Collect unique symbols to embed in lib_symbols
    sym_defs = {}
    sym_pins = {}

    needed = set((c[2], c[3]) for c in COMPONENTS)
    needed.add(("power", "+3V3"))
    needed.add(("power", "+3.3VA"))
    needed.add(("power", "GND"))
    needed.add(("power", "PWR_FLAG"))

    for lib_name, sym_name in sorted(needed):
        resolved = resolve_symbol_for_lib(lib_name, sym_name)
        sym_defs[(lib_name, sym_name)] = resolved
        sym_pins[(lib_name, sym_name)] = parse_pins_from_symbol_block(resolved)

    lines = []
    lines.append('(kicad_sch (version 20211123) (generator eeschema)')
    lines.append(f'  (uuid {make_uuid("root")})')
    lines.append('  (paper "A2")')
    lines.append('  (title_block')
    lines.append('    (title "Crash-Safe UAV Flight Data Logger (STM32F446RE + ESP32 + FRAM + Supercap)")')
    lines.append('    (date "2026-11-11")')
    lines.append('    (rev "1.0")')
    lines.append('    (company "UAV Avionics Hardware")')
    lines.append('    (comment 1 "4-Layer 50x50mm PCB, 30.5x30.5mm M3 Stack")')
    lines.append('    (comment 2 "3.7V Li-ion -> Ideal Diode OR-ing -> 3.3V 2A Buck + 5.5V 1.0F Supercap + PFD EXTI")')
    lines.append('  )')

    # lib_symbols
    lines.append('  (lib_symbols')
    for _, sdef in sorted(sym_defs.items()):
        lines.append('    ' + sdef)
    lines.append('  )')

    # Functional block frame boxes & titles on A2 sheet (594 x 420 mm)
    sections = [
        (8, 12, 232, 134, "1. PRIMARY POWER INPUT, REVERSE POLARITY, IDEAL DIODE OR-ING & SUPERCAP CHARGER"),
        (8, 138, 152, 224, "2. MAIN 3.3V 2A SYNCHRONOUS BUCK (TPS62822)"),
        (8, 228, 152, 314, "3. POWER FAIL DETECT (PFD 3.00V) & 2.5V REF"),
        (158, 12, 396, 250, "4. CENTRAL MCU (STM32F446RET6), HSE/LSE CLOCKS, SWD & DECOUPLING"),
        (158, 254, 396, 334, "5. ADC1 BATTERY DIVIDER (0.1%) & OPTICAL EMERGENCY BEACON"),
        (402, 12, 540, 154, "6. SHARED I2C1 SENSORS (MPU-6050 0x68 & BMP280 0x76)"),
        (402, 158, 540, 296, "7. SHARED SPI1 STORAGE (MICROSD & CY15B102QN FRAM)"),
        (546, 12, 690, 270, "8. WIRELESS TELEMETRY (ESP32-WROOM-32 & 6P UART)"),
        (546, 274, 690, 320, "9. M3 STACK MOUNTING HOLES & POWER FLAGS"),
    ]
    for x0, y0, x1, y1, title in sections:
        lines.append(f'  (rectangle (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.254) (type dash) (color 0 80 140 1)) (fill (type none)) (uuid {make_uuid(title)}))')
        lines.append(f'  (text "{title}" (at {x0 + 2} {y0 + 4} 0) (effects (font (size 2.2 2.2) bold) (justify left)) (uuid {make_uuid(title + "t")}))')

    pwr_counter = 1
    symbol_instances = []

    def add_power_sym(net_name, x, y, rot=0):
        nonlocal pwr_counter
        ref = f"#PWR{pwr_counter:03d}"
        pwr_counter += 1
        uid = make_uuid(f"pwr-{ref}")
        lib_id = f"power:{net_name}"
        lines.append(f'  (symbol (lib_id "{lib_id}") (at {x:.2f} {y:.2f} {rot}) (unit 1)')
        lines.append(f'    (in_bom no) (on_board yes)')
        lines.append(f'    (uuid {uid})')
        lines.append(f'    (property "Reference" "{ref}" (id 0) (at {x:.2f} {y:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Value" "{net_name}" (id 1) (at {x:.2f} {y + (3.8 if rot == 0 else -3.8):.2f} 0) (effects (font (size 1.27 1.27))))')
        lines.append(f'    (property "Footprint" "" (id 2) (at {x:.2f} {y:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Datasheet" "" (id 3) (at {x:.2f} {y:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (pin "1" (uuid {make_uuid(ref + "p1")}))')
        lines.append('  )')
        symbol_instances.append((uid, ref, 1, net_name, ""))

    def add_wire(x1, y1, x2, y2):
        if abs(x1 - x2) < 1e-4 and abs(y1 - y2) < 1e-4:
            return
        lines.append(f'  (wire (pts (xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f})) (stroke (width 0) (type default) (color 0 0 0 0)) (uuid {make_uuid("w")}))')

    def add_label(text, x, y, angle=0):
        just = "left" if angle == 0 else "right"
        lines.append(f'  (global_label "{text}" (shape bidirectional) (at {x:.2f} {y:.2f} {angle}) (fields_autoplaced) (effects (font (size 1.27 1.27)) (justify {just})) (uuid {make_uuid("lbl-" + text)}))')

    def add_nc(x, y):
        lines.append(f'  (no_connect (at {x:.2f} {y:.2f}) (uuid {make_uuid("nc")}))')

    # Place all real components from design_spec.py
    for comp in COMPONENTS:
        ref, val, sym_lib, sym_name, fp_lib, fp_name, side, pcb_x, pcb_y, pcb_rot, mpn, desc, pin_nets = comp
        sx, sy = SCH_LAYOUT[ref]
        uid = make_uuid(f"comp-{ref}")
        lib_id = f"{sym_lib}:{sym_name}"
        fp_full = f"{fp_lib}:{fp_name}"
        pins = sym_pins[(sym_lib, sym_name)]

        # Offset labels cleanly above/below symbol
        lines.append(f'  (symbol (lib_id "{lib_id}") (at {sx:.2f} {sy:.2f} 0) (unit 1)')
        lines.append(f'    (in_bom yes) (on_board yes)')
        lines.append(f'    (uuid {uid})')
        lines.append(f'    (property "Reference" "{ref}" (id 0) (at {sx + 2.54:.2f} {sy - 2.54:.2f} 0) (effects (font (size 1.27 1.27)) (justify left)))')
        lines.append(f'    (property "Value" "{val}" (id 1) (at {sx + 2.54:.2f} {sy:.2f} 0) (effects (font (size 1.27 1.27)) (justify left)))')
        lines.append(f'    (property "Footprint" "{fp_full}" (id 2) (at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Datasheet" "" (id 3) (at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "MPN" "{mpn}" (id 4) (at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.27 1.27)) hide))')

        # Deduplicate pin numbers (some multi-unit or hidden pins)
        seen_nums = set()
        for pnum, pname, px, py, pang, plen, ptype in pins:
            if pnum not in seen_nums:
                seen_nums.add(pnum)
                lines.append(f'    (pin "{pnum}" (uuid {make_uuid(f"{ref}-pin-{pnum}")}))')
        lines.append('  )')
        symbol_instances.append((uid, ref, 1, val, fp_full))

        # Wire each pin cleanly according to design_spec pin_nets!
        # In KiCad sch: world position = (sx + px, sy - py) for angle=0
        # Pin angle: 0 means pin extends RIGHT from body (wire endpoint at LEFT? Wait:
        # in KiCad sym, (at px py 0) means wire connects at (px, py) and goes LEFT(-X) into body, so wire continues LEFT(-X) to exit!
        # Verify: R pin 1 is at (0, 3.81, 270) -> body is below (0..2.54), endpoint is at TOP (0, 3.81), so outward wire goes UP (-Y in sch)!
        # angle=0: endpoint on LEFT of body -> outward wire goes LEFT (-X, angle_lbl=180)
        # angle=180: endpoint on RIGHT of body -> outward wire goes RIGHT (+X, angle_lbl=0)
        # angle=270: endpoint on TOP of body -> outward wire goes UP (-Y in sch)
        # angle=90: endpoint on BOTTOM of body -> outward wire goes DOWN (+Y in sch)
        seen_wire_coords = set()
        for pnum, pname, px, py, pang, plen, ptype in pins:
            wx = round(sx + px, 2)
            wy = round(sy - py, 2)
            # Handle duplicate overlapped pins (like STM32 or ESP32 hidden GND/PGND)
            if (wx, wy) in seen_wire_coords and pnum not in pin_nets:
                continue
            seen_wire_coords.add((wx, wy))

            net = pin_nets.get(pnum)
            if not net:
                add_nc(wx, wy)
                continue

            # Compute outward stub direction
            if abs(pang - 0) < 1:
                dx, dy, lbl_ang = -5.08, 0.0, 180
            elif abs(pang - 180) < 1:
                dx, dy, lbl_ang = 5.08, 0.0, 0
            elif abs(pang - 270) < 1:
                dx, dy, lbl_ang = 0.0, -5.08, 0
            else:  # 90
                dx, dy, lbl_ang = 0.0, 5.08, 0

            ex = round(wx + dx, 2)
            ey = round(wy + dy, 2)
            add_wire(wx, wy, ex, ey)

            if net in ("GND", "+3V3", "+3.3VA"):
                if dy < 0:
                    add_power_sym(net, ex, ey, 0 if net != "GND" else 180)
                elif dy > 0:
                    add_power_sym(net, ex, ey, 0 if net == "GND" else 180)
                elif dx < 0:
                    rot_p = 270 if net == "GND" else 90
                    add_power_sym(net, ex, ey, rot_p)
                else:
                    rot_p = 90 if net == "GND" else 270
                    add_power_sym(net, ex, ey, rot_p)
            else:
                add_label(net, ex, ey, lbl_ang)

    # Add PWR_FLAGs for GND, +3V3, +3.3VA so KiCad ERC passes cleanly
    for idx, pnet in enumerate(["GND", "+3V3", "+3.3VA"]):
        px, py = g(218 + idx * 8, 122)
        add_power_sym(pnet, px, py, 0 if pnet == "GND" else 180)
        add_wire(px, py, px, py + 2.54)
        # PWR_FLAG pin is at (0, 0)
        uid = make_uuid(f"pwrflag-{pnet}")
        ref = f"#FLG{idx+1:02d}"
        lines.append(f'  (symbol (lib_id "power:PWR_FLAG") (at {px:.2f} {py + 2.54:.2f} 0) (unit 1)')
        lines.append(f'    (in_bom no) (on_board yes) (uuid {uid})')
        lines.append(f'    (property "Reference" "{ref}" (id 0) (at {px:.2f} {py + 5:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Value" "PWR_FLAG" (id 1) (at {px:.2f} {py + 7.5:.2f} 0) (effects (font (size 1.27 1.27))))')
        lines.append(f'    (property "Footprint" "" (id 2) (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Datasheet" "" (id 3) (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (pin "1" (uuid {make_uuid(ref + "p1")}))')
        lines.append('  )')
        symbol_instances.append((uid, ref, 1, "PWR_FLAG", ""))

    # Power flags for non-global-power rails connected to Power Input pins (LTC4412 VIN / TPS62822 VIN / VCAP_1)
    for j_idx, inet in enumerate(["V_BAT_PROT", "V_SCAP", "V_SYS", "VCAP_1"]):
        px, py = g(244 + j_idx * 7, 122)
        add_label(inet, px, py, 0)
        add_wire(px, py, px, py + 2.54)
        uid = make_uuid(f"pwrflag-{inet}")
        ref = f"#FLG{j_idx+4:02d}"
        lines.append(f'  (symbol (lib_id "power:PWR_FLAG") (at {px:.2f} {py + 2.54:.2f} 0) (unit 1)')
        lines.append(f'    (in_bom no) (on_board yes) (uuid {uid})')
        lines.append(f'    (property "Reference" "{ref}" (id 0) (at {px:.2f} {py + 5:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Value" "PWR_FLAG" (id 1) (at {px:.2f} {py + 7.5:.2f} 0) (effects (font (size 1.27 1.27))))')
        lines.append(f'    (property "Footprint" "" (id 2) (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (property "Datasheet" "" (id 3) (at {px:.2f} {py:.2f} 0) (effects (font (size 1.27 1.27)) hide))')
        lines.append(f'    (pin "1" (uuid {make_uuid(ref + "p1")}))')
        lines.append('  )')
        symbol_instances.append((uid, ref, 1, "PWR_FLAG", ""))

    # sheet_instances & symbol_instances
    lines.append('  (sheet_instances')
    lines.append('    (path "/" (page "1"))')
    lines.append('  )')
    lines.append('  (symbol_instances')
    for uid, ref, unit, val, fp in symbol_instances:
        lines.append(f'    (path "/{uid}" (reference "{ref}") (unit {unit}) (value "{val}") (footprint "{fp}"))')
    lines.append('  )')
    lines.append(')')

    out_sch = os.path.join(OUT_DIR, "FlightDataLogger.kicad_sch")
    with open(out_sch, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {out_sch} ({len(COMPONENTS)} components, {pwr_counter-1} power symbols)")


if __name__ == "__main__":
    generate_kicad_pro()
    generate_kicad_sch()
