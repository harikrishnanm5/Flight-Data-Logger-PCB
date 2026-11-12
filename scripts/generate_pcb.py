"""
Generate Complete 4-Layer Flight Data Logger PCB (FlightDataLogger.kicad_pcb)
via KiCad 6.0 pcbnew API:
- 50.0 x 50.0 mm rounded (R=2.5 mm) outline, 30.5 x 30.5 mm M3 stack holes
- 4-Layer EMC stackup: F.Cu (Sig+GND), In1.Cu (Solid GND), In2.Cu (+3V3), B.Cu (Sig+GND)
- Respects ALL footprint built-in keepouts (U2 antenna, U3 MPU-6050, J3 MicroSD)
- Enforces >= 0.20 mm KiCad clearance and >= 0.54 mm hole-to-hole wall spacing
- Negotiated-congestion Pathfinder A* router (0.05 mm grid) + transactional rollback
- Clean silkscreen + GND/power stitching vias + LoadBoard Zone Fill + KiCad DRC
"""
import array
import heapq
import math
import os
import sys
import time
from collections import Counter, defaultdict

import pcbnew

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

sys.path.insert(0, os.path.dirname(__file__))
from design_spec import BOARD_H, BOARD_W, COMPONENTS, CORNER_R, FINE_NETS, POWER_NETS

OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FP_ROOT = r"D:\FOSSEE\KiCad\share\kicad\footprints"
VERIF_DIR = os.path.join(OUT_DIR, "verification")

# Routing grid: 0.05 mm (50 um) over 50.0 x 50.0 mm -> 1001 x 1001
RES = 0.05
INV_RES = 20.0
NX = int(round(BOARD_W / RES)) + 1
NY = int(round(BOARD_H / RES)) + 1
SIZE = NX * NY

# Clearances & hole spacing (strictly exceeds KiCad 0.20mm / 0.50mm rules)
CLEARANCE_MM = 0.218
GRID_BASE_HW = 0.090     # 0.18 mm fine trace base half-width
HOLE_WALL_CLEAR = 0.54   # > 0.500 mm KiCad m_HoleToHoleMin rule
VIA_D = 0.52             # 0.135 mm annular ring (> 0.125 mm rule)
VIA_DRILL = 0.25         # >= 0.200 mm min drill
PWR_VIA_D = 0.62
PWR_VIA_DRILL = 0.30
ANTENNA_KEEPOUT_Y = 6.25


def to_grid(x):
    return max(0, min(NX - 1, int(round(x * INV_RES))))


def to_mm(g):
    return g * RES


def plane_outline_pts(margin=0.35, y_top=6.30, r_bot=2.15):
    x0, x1 = margin, BOARD_W - margin
    y0, y1 = y_top, BOARD_H - margin
    pts = [(x0, y0), (x1, y0)]
    for cx, cy, a0, a1 in [
        (x1 - r_bot, y1 - r_bot, 0.0, math.pi / 2),
        (x0 + r_bot, y1 - r_bot, math.pi / 2, math.pi),
    ]:
        for k in range(7):
            a = a0 + (a1 - a0) * (k / 6)
            pts.append((cx + r_bot * math.cos(a), cy + r_bot * math.sin(a)))
    return pts


def draw_board_outline(board):
    w, h, r = BOARD_W, BOARD_H, CORNER_R
    segs = [
        ((r, 0.0), (w - r, 0.0)),
        ((w, r), (w, h - r)),
        ((w - r, h), (r, h)),
        ((0.0, h - r), (0.0, r)),
    ]
    for (x1, y1), (x2, y2) in segs:
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetStart(pcbnew.wxPointMM(x1, y1))
        s.SetEnd(pcbnew.wxPointMM(x2, y2))
        s.SetWidth(pcbnew.FromMM(0.1))
        board.Add(s)

    arcs = [
        ((w - r, 0.0), (w - r + r * math.cos(-math.pi / 4), r + r * math.sin(-math.pi / 4)), (w, r)),
        ((w, h - r), (w - r + r * math.cos(math.pi / 4), h - r + r * math.sin(math.pi / 4)), (w - r, h)),
        ((r, h), (r + r * math.cos(3 * math.pi / 4), h - r + r * math.sin(3 * math.pi / 4)), (0.0, h - r)),
        ((0.0, r), (r + r * math.cos(-3 * math.pi / 4), r + r * math.sin(-3 * math.pi / 4)), (r, 0.0)),
    ]
    for st, mid, en in arcs:
        a = pcbnew.PCB_SHAPE(board)
        a.SetShape(pcbnew.SHAPE_T_ARC)
        a.SetLayer(pcbnew.Edge_Cuts)
        a.SetArcGeometry(pcbnew.wxPointMM(*st), pcbnew.wxPointMM(*mid), pcbnew.wxPointMM(*en))
        a.SetWidth(pcbnew.FromMM(0.1))
        board.Add(a)


class PadGeom:
    __slots__ = ("ref", "pname", "net", "x", "y", "hw", "hh", "cos_a", "sin_a",
                 "is_tht", "on_f", "on_b", "drill_r", "shape", "fp_x", "fp_y", "is_fine")

    def __init__(self, fp, pad, net_name):
        self.ref = fp.GetReference()
        self.pname = pad.GetName()
        self.net = net_name
        pos = pad.GetPosition()
        self.x = pcbnew.ToMM(pos.x)
        self.y = pcbnew.ToMM(pos.y)
        sz = pad.GetSize()
        self.hw = 0.5 * pcbnew.ToMM(sz.x)
        self.hh = 0.5 * pcbnew.ToMM(sz.y)
        ang_rad = math.radians(pad.GetOrientationDegrees())
        self.cos_a = math.cos(ang_rad)
        self.sin_a = math.sin(ang_rad)
        self.shape = pad.GetShape()
        self.on_f = pad.IsOnLayer(pcbnew.F_Cu)
        self.on_b = pad.IsOnLayer(pcbnew.B_Cu)
        self.drill_r = 0.5 * pcbnew.ToMM(pad.GetDrillSize().x) if pad.GetDrillSize().x > 0 else 0.0
        self.is_tht = self.drill_r > 0.0 and self.on_f and self.on_b
        fpos = fp.GetPosition()
        self.fp_x = pcbnew.ToMM(fpos.x)
        self.fp_y = pcbnew.ToMM(fpos.y)
        self.is_fine = (min(self.hw, self.hh) * 2.0) <= 0.36

    def dist_to_point(self, px, py):
        dx = px - self.x
        dy = py - self.y
        lx = dx * self.cos_a + dy * self.sin_a
        ly = -dx * self.sin_a + dy * self.cos_a
        if self.shape == pcbnew.PAD_SHAPE_CIRCLE:
            return max(0.0, math.hypot(lx, ly) - max(self.hw, self.hh))
        elif self.shape == pcbnew.PAD_SHAPE_OVAL:
            if self.hw >= self.hh:
                core = max(0.0, self.hw - self.hh)
                rx = max(-core, min(core, lx))
                return max(0.0, math.hypot(lx - rx, ly) - self.hh)
            core = max(0.0, self.hh - self.hw)
            ry = max(-core, min(core, ly))
            return max(0.0, math.hypot(lx, ly - ry) - self.hw)
        elif self.shape == pcbnew.PAD_SHAPE_ROUNDRECT:
            r = min(self.hw, self.hh) * 0.25
            qx = max(0.0, abs(lx) - (self.hw - r))
            qy = max(0.0, abs(ly) - (self.hh - r))
            return max(0.0, math.hypot(qx, qy) - r)
        qx = max(0.0, abs(lx) - self.hw)
        qy = max(0.0, abs(ly) - self.hh)
        return math.hypot(qx, qy)

    def max_extent(self):
        return math.hypot(self.hw, self.hh)


def extract_keepout_boxes(board):
    """Extract all built-in footprint keepout zones (onF, onB, noTrk, noVia, x0, y0, x1, y1)."""
    boxes = []
    for fp in board.GetFootprints():
        for z in fp.Zones():
            if not z.IsKeepout():
                continue
            bb = z.GetBoundingBox()
            boxes.append((
                z.IsOnLayer(pcbnew.F_Cu),
                z.IsOnLayer(pcbnew.B_Cu),
                z.GetDoNotAllowTracks(),
                z.GetDoNotAllowVias(),
                pcbnew.ToMM(bb.GetX()),
                pcbnew.ToMM(bb.GetY()),
                pcbnew.ToMM(bb.GetRight()),
                pcbnew.ToMM(bb.GetBottom()),
            ))
    return boxes


def dist_point_to_seg(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    l2 = vx * vx + vy * vy
    if l2 <= 1e-12:
        return math.hypot(wx, wy)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / l2))
    return math.hypot(wx - t * vx, wy - t * vy)


def clean_silkscreen_artifacts(board, pads):
    """Trim library silkscreen graphics that clip Edge.Cuts or solder-mask pad openings."""
    for fp in board.GetFootprints():
        side = "B" if fp.IsFlipped() else "F"
        fp_pads = [p for p in pads if p.ref == fp.GetReference()]
        to_rem = []
        for item in fp.GraphicalItems():
            if "Silk" not in item.GetLayerName():
                continue
            bb = item.GetBoundingBox()
            gx0 = pcbnew.ToMM(bb.GetX())
            gy0 = pcbnew.ToMM(bb.GetY())
            gx1 = pcbnew.ToMM(bb.GetRight())
            gy1 = pcbnew.ToMM(bb.GetBottom())
            if gy0 <= 0.12 or gx0 <= 0.12 or gx1 >= BOARD_W - 0.12 or gy1 >= BOARD_H - 0.12:
                to_rem.append(item)
                continue
            # Check if built-in graphic silk line directly overlaps solder mask expansion on a pad
            # Test sample points along its bbox diagonal/center/endpoints if small (< 1.5mm)
            if (gx1 - gx0) < 1.2 and (gy1 - gy0) < 1.2:
                cx, cy = 0.5 * (gx0 + gx1), 0.5 * (gy0 + gy1)
                for p in fp_pads:
                    on_same = p.on_f if side == "F" else p.on_b
                    if on_same and p.dist_to_point(cx, cy) < 0.14:
                        to_rem.append(item)
                        break
            else:
                # For line segments (e.g. BMP280 corner marks), check both endpoints if PCB_SHAPE segment
                if hasattr(item, "GetStart") and item.GetShape() == pcbnew.SHAPE_T_SEGMENT:
                    sx, sy = pcbnew.ToMM(item.GetStart().x), pcbnew.ToMM(item.GetStart().y)
                    ex, ey = pcbnew.ToMM(item.GetEnd().x), pcbnew.ToMM(item.GetEnd().y)
                    hit = False
                    for k in range(5):
                        t = k / 4.0
                        px, py = sx + t * (ex - sx), sy + t * (ey - sy)
                        for p in fp_pads:
                            on_same = p.on_f if side == "F" else p.on_b
                            if on_same and p.dist_to_point(px, py) < 0.13:
                                hit = True
                                break
                        if hit:
                            break
                    if hit:
                        to_rem.append(item)
        for item in to_rem:
            fp.Remove(item)


def arrange_silkscreen(board, pads):
    mask_boxes = []
    for p in pads:
        ex = max(p.hw, p.hh) + 0.25
        x0, y0, x1, y1 = p.x - ex, p.y - ex, p.x + ex, p.y + ex
        if p.on_f:
            mask_boxes.append(("F", x0, y0, x1, y1))
        if p.on_b:
            mask_boxes.append(("B", x0, y0, x1, y1))

    placed_silk = {"F": [], "B": []}
    for fp in board.GetFootprints():
        side = "B" if fp.IsFlipped() else "F"
        for item in fp.GraphicalItems():
            if "Silk" in item.GetLayerName():
                bb = item.GetBoundingBox()
                placed_silk[side].append((
                    pcbnew.ToMM(bb.GetX()) - 0.22,
                    pcbnew.ToMM(bb.GetY()) - 0.22,
                    pcbnew.ToMM(bb.GetRight()) + 0.22,
                    pcbnew.ToMM(bb.GetBottom()) + 0.22,
                ))

    def overlaps_any(side, bx0, by0, bx1, by1):
        if bx0 < 0.40 or by0 < 0.40 or bx1 > BOARD_W - 0.40 or by1 > BOARD_H - 0.40:
            return True
        for mx, my in [(9.75, 9.75), (40.25, 9.75), (9.75, 40.25), (40.25, 40.25)]:
            if not (bx1 < mx - 3.45 or bx0 > mx + 3.45 or by1 < my - 3.45 or by0 > my + 3.45):
                return True
        for ls, x0, y0, x1, y1 in mask_boxes:
            if ls == side and not (bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1):
                return True
        for x0, y0, x1, y1 in placed_silk[side]:
            if not (bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1):
                return True
        return False

    hidden_refs = 0
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        fp.Value().SetVisible(False)
        ref_txt = fp.Reference()
        ref = fp.GetReference()
        side = "B" if fp.IsFlipped() else "F"
        if ref in ("H1", "H2", "H3", "H4"):
            ref_txt.SetVisible(False)
            hidden_refs += 1
            continue

        sz_h = 0.70 if ref[0] in ("U", "J") else 0.50
        th = 0.11 if sz_h >= 0.7 else 0.09
        ref_txt.SetTextSize(pcbnew.wxSizeMM(sz_h, sz_h))
        ref_txt.SetTextThickness(pcbnew.FromMM(th))
        tw = len(ref) * sz_h * 1.12 + 0.44
        th_box = sz_h * 1.45 + 0.44
        fpos = fp.GetPosition()
        fx, fy = pcbnew.ToMM(fpos.x), pcbnew.ToMM(fpos.y)

        cands = [(fx, fy, 0)]
        for r_off in (1.3, 1.8, 2.4, 3.1, 4.0, 5.0):
            for ang in (90, -90, 0, 180, 45, 135, -45, -135):
                ar = math.radians(ang)
                cands.append((fx + r_off * math.cos(ar), fy + r_off * math.sin(ar), 0))
                cands.append((fx + r_off * math.cos(ar), fy + r_off * math.sin(ar), 90))

        placed = False
        for cx, cy, rot_t in cands:
            w_box = tw if rot_t == 0 else th_box
            h_box = th_box if rot_t == 0 else tw
            bx0, by0, bx1, by1 = cx - w_box / 2, cy - h_box / 2, cx + w_box / 2, cy + h_box / 2
            if not overlaps_any(side, bx0, by0, bx1, by1):
                ref_txt.SetPosition(pcbnew.wxPointMM(cx, cy))
                ref_txt.SetTextAngle(rot_t * 10)
                ref_txt.SetKeepUpright(True)
                ref_txt.SetVisible(True)
                placed_silk[side].append((bx0 - 0.20, by0 - 0.20, bx1 + 0.20, by1 + 0.20))
                placed = True
                break
        if not placed:
            ref_txt.SetVisible(False)
            hidden_refs += 1

    legends = [
        ("UAV FLIGHT LOGGER v1.0", 25.0, 2.5, 0.95, 0.16, pcbnew.F_SilkS),
        ("M3 30.5x30.5 STACK", 25.0, 47.0, 0.65, 0.11, pcbnew.B_SilkS),
    ]
    for text, lx, ly, lsz, lth, layer in legends:
        side = "F" if layer == pcbnew.F_SilkS else "B"
        tw = len(text) * lsz * 0.78 + 0.25
        th_box = lsz * 1.25 + 0.25
        if not overlaps_any(side, lx - tw / 2, ly - th_box / 2, lx + tw / 2, ly + th_box / 2):
            t = pcbnew.PCB_TEXT(board)
            t.SetText(text)
            t.SetPosition(pcbnew.wxPointMM(lx, ly))
            t.SetTextSize(pcbnew.wxSizeMM(lsz, lsz))
            t.SetTextThickness(pcbnew.FromMM(lth))
            t.SetLayer(layer)
            if layer == pcbnew.B_SilkS:
                t.SetMirrored(True)
            board.Add(t)
            placed_silk[side].append((lx - tw / 2, ly - th_box / 2, lx + tw / 2, ly + th_box / 2))
    return hidden_refs


class PathfinderRouter:
    """
    0.05 mm 2-layer Signal + Plane-Linker with:
    - Exact static footprint keepout zones (U2, U3, J3)
    - Strict hole-to-hole wall clearance across ALL nets (incl. same net)
    - IC perimeter soft guard cost (discourages tangentially cutting off fine pins)
    - Negotiated Pathfinder rip-up & retry with transactional net snapshots
    """

    def __init__(self, board, pads, keepouts):
        self.board = board
        self.pads = pads
        self.keepouts = keepouts
        self.pads_by_net = defaultdict(list)
        for i, p in enumerate(pads):
            if p.net:
                self.pads_by_net[p.net].append(i)

        self.tracks = []  # (layer, x1, y1, x2, y2, w, net)
        self.vias = []    # (x, y, d, drill, net)

        self.pad_bucket = defaultdict(list)
        for i, p in enumerate(pads):
            r = p.max_extent() + 0.8
            for bx in range(max(0, int((p.x - r) / 2.0)), min(25, int((p.x + r) / 2.0)) + 1):
                for by in range(max(0, int((p.y - r) / 2.0)), min(25, int((p.y + r) / 2.0)) + 1):
                    self.pad_bucket[(bx, by)].append(i)

        self.via_bucket = defaultdict(list)
        self.track_bucket = defaultdict(list)

        # Track and Via obstacle counts (H = uint16) + hard pad/keepout mask + soft/history maps
        self.trk_occ = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
        self.via_occ = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
        self.hard_occ = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
        self.soft_cost = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
        self.hist_cost = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]

        # Which cells have active signal tracks of each net (stored per net for fast toggle/ripup)
        self.net_trk_cells = defaultdict(lambda: [set(), set()])
        self.net_via_cells = defaultdict(lambda: [set(), set()])
        self.net_hole_cells = defaultdict(set)
        self.pad_halo_cells = [[] for _ in pads]
        self.pad_halo_active = [False] * len(pads)
        self.pad_halo_perm_off = [False] * len(pads)
        self.ic_guard_cells = [[], []]
        self.via_soft_cost = array.array("H", [0]) * SIZE

        t0 = time.time()
        self._stamp_static_borders_and_keepouts()
        self.pad_trk_stamp = []
        self.pad_via_stamp = []
        self._precompute_and_stamp_pads()
        self._build_halos_and_ic_guards()
        self._base_trk_occ = [array.array("H", self.trk_occ[0]), array.array("H", self.trk_occ[1])]
        self._base_via_occ = [array.array("H", self.via_occ[0]), array.array("H", self.via_occ[1])]
        self._base_hard_occ = [array.array("H", self.hard_occ[0]), array.array("H", self.hard_occ[1])]
        self._base_ic_soft_cost = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
        for l in (0, 1):
            sc = self._base_ic_soft_cost[l]
            for idx in self.ic_guard_cells[l]:
                sc[idx] += 65
        print(f"50um Pathfinder grid init finished in {time.time() - t0:.2f}s")

    def _stamp_static_borders_and_keepouts(self):
        k_y = to_grid(ANTENNA_KEEPOUT_Y + 0.12)
        b_trk = 10  # 0.50 mm margin
        b_via = 12  # 0.60 mm margin
        r_safe_trk = CORNER_R - 0.45
        r_safe_via = CORNER_R - 0.60

        t0, t1 = self.trk_occ
        v0, v1 = self.via_occ
        h0, h1 = self.hard_occ
        row_full = array.array("H", [1000]) * NX
        for y in range(NY):
            off = y * NX
            if y <= k_y or y < b_trk or y > NY - 1 - b_trk:
                t0[off:off + NX] = row_full; t1[off:off + NX] = row_full
                h0[off:off + NX] = row_full; h1[off:off + NX] = row_full
                v0[off:off + NX] = row_full; v1[off:off + NX] = row_full
                continue
            if y <= k_y + 2 or y < b_via or y > NY - 1 - b_via:
                v0[off:off + NX] = row_full; v1[off:off + NX] = row_full
            for x in range(b_trk):
                t0[off + x] = 1000; t1[off + x] = 1000
                t0[off + NX - 1 - x] = 1000; t1[off + NX - 1 - x] = 1000
                h0[off + x] = 1000; h1[off + x] = 1000
                h0[off + NX - 1 - x] = 1000; h1[off + NX - 1 - x] = 1000
            for x in range(b_via):
                v0[off + x] = 1000; v1[off + x] = 1000
                v0[off + NX - 1 - x] = 1000; v1[off + NX - 1 - x] = 1000
            ym = y * RES
            if ym > BOARD_H - CORNER_R:
                for x in list(range(0, to_grid(CORNER_R) + 1)) + list(range(to_grid(BOARD_W - CORNER_R), NX)):
                    xm = x * RES
                    cx = CORNER_R if xm < 25 else BOARD_W - CORNER_R
                    dc = math.hypot(xm - cx, ym - (BOARD_H - CORNER_R))
                    if dc > r_safe_trk:
                        t0[off + x] = 1000; t1[off + x] = 1000
                        h0[off + x] = 1000; h1[off + x] = 1000
                    if dc > r_safe_via:
                        v0[off + x] = 1000; v1[off + x] = 1000

        # Stamp ALL footprint keepouts (U2 antenna, U3 MPU-6050, J3 MicroSD)
        for on_f, on_b, no_trk, no_via, kx0, ky0, kx1, ky1 in self.keepouts:
            if no_trk:
                m = GRID_BASE_HW + 0.04
                gx0, gx1 = max(0, int((kx0 - m) * INV_RES)), min(NX - 1, int((kx1 + m) * INV_RES))
                gy0, gy1 = max(0, int((ky0 - m) * INV_RES)), min(NY - 1, int((ky1 + m) * INV_RES))
                for gy in range(gy0, gy1 + 1):
                    off = gy * NX
                    for gx in range(gx0, gx1 + 1):
                        if on_f: t0[off + gx] = 1000; h0[off + gx] = 1000
                        if on_b: t1[off + gx] = 1000; h1[off + gx] = 1000
            if no_via and (on_f or on_b):
                mv = 0.5 * VIA_D + 0.05
                gx0, gx1 = max(0, int((kx0 - mv) * INV_RES)), min(NX - 1, int((kx1 + mv) * INV_RES))
                gy0, gy1 = max(0, int((ky0 - mv) * INV_RES)), min(NY - 1, int((ky1 + mv) * INV_RES))
                for gy in range(gy0, gy1 + 1):
                    off = gy * NX
                    for gx in range(gx0, gx1 + 1):
                        v0[off + gx] = 1000; v1[off + gx] = 1000

    def _precompute_and_stamp_pads(self):
        # 0.090 + 0.212 = 0.302 mm (< 0.325 worst-case center-to-neighbor on 0.50mm pitch!)
        thr_t = GRID_BASE_HW + 0.212
        thr_v = 0.5 * VIA_D + CLEARANCE_MM
        for p in self.pads:
            thr_h = p.drill_r + 0.5 * VIA_DRILL + HOLE_WALL_CLEAR if p.drill_r > 0 else 0.0
            reach = p.max_extent() + max(thr_v, thr_h) + 0.10
            gx0 = max(0, int((p.x - reach) * INV_RES))
            gx1 = min(NX - 1, int((p.x + reach) * INV_RES))
            gy0 = max(0, int((p.y - reach) * INV_RES))
            gy1 = min(NY - 1, int((p.y + reach) * INV_RES))

            t_f, t_b, v_f, v_b = [], [], [], []
            for gy in range(gy0, gy1 + 1):
                ym = gy * RES
                off = gy * NX
                for gx in range(gx0, gx1 + 1):
                    xm = gx * RES
                    d = p.dist_to_point(xm, ym)
                    idx = off + gx
                    if d < thr_t:
                        if p.on_f: t_f.append(idx)
                        if p.on_b: t_b.append(idx)
                    if p.drill_r > 0 and math.hypot(xm - p.x, ym - p.y) < max(thr_h, thr_v):
                        # Drill hole exclusion ALWAYS applies to ALL nets including same net!
                        self.via_occ[0][idx] += 1000
                        self.via_occ[1][idx] += 1000
                    elif d < thr_v:
                        if p.on_f: v_f.append(idx)
                        if p.on_b: v_b.append(idx)

            for idx in t_f: self.trk_occ[0][idx] += 1; self.hard_occ[0][idx] += 1
            for idx in t_b: self.trk_occ[1][idx] += 1; self.hard_occ[1][idx] += 1
            for idx in v_f: self.via_occ[0][idx] += 1
            for idx in v_b: self.via_occ[1][idx] += 1
            self.pad_trk_stamp.append((t_f, t_b))
            self.pad_via_stamp.append((v_f, v_b))

    def _build_halos_and_ic_guards(self):
        """
        Compute dynamic IC interior soft penalties + per-pad 1.20 mm escape runway halos.
        Runways reserve space ahead of still-unrouted fine pins so early signals route outside or on B.Cu.
        """
        by_ref = defaultdict(list)
        for pid, p in enumerate(self.pads):
            by_ref[p.ref].append((pid, p))

        # Dynamic IC body interior guard on pad side (discourages crossing through QFP/QFN pad rings on F.Cu)
        for ref, plist in by_ref.items():
            if ref not in ("U1", "U3"):
                continue
            on_b_only = all((not p.on_f and p.on_b) for _, p in plist)
            layer_idx = 1 if on_b_only else 0
            xs = [p.x for _, p in plist]
            ys = [p.y for _, p in plist]
            x0, x1 = min(xs) + 0.85, max(xs) - 0.85
            y0, y1 = min(ys) + 0.85, max(ys) - 0.85
            gx0, gx1 = max(0, int(x0 * INV_RES)), min(NX - 1, int(x1 * INV_RES))
            gy0, gy1 = max(0, int(y0 * INV_RES)), min(NY - 1, int(y1 * INV_RES))
            seen_g = set()
            for gy in range(gy0, gy1 + 1):
                off = gy * NX
                for gx in range(gx0, gx1 + 1):
                    idx = off + gx
                    if idx not in seen_g:
                        seen_g.add(idx)
                        self.ic_guard_cells[layer_idx].append(idx)

        # Fine-pitch IC via-drop standoff buffer: gently discourage signal vias within 0.78 mm of U1/U3 pad edges
        # so early signals escape outward before dropping vias without bloating A* cost or starving SOT-23/B.Cu ICs.
        vs_c = self.via_soft_cost
        for p in self.pads:
            if p.is_tht or p.ref not in ("U1", "U3"):
                continue
            r_buf = 0.78
            reach = p.max_extent() + r_buf + 0.05
            gx0 = max(0, int((p.x - reach) * INV_RES))
            gx1 = min(NX - 1, int((p.x + reach) * INV_RES))
            gy0 = max(0, int((p.y - reach) * INV_RES))
            gy1 = min(NY - 1, int((p.y + reach) * INV_RES))
            for gy in range(gy0, gy1 + 1):
                ym = gy * RES
                off = gy * NX
                for gx in range(gx0, gx1 + 1):
                    if p.dist_to_point(gx * RES, ym) < r_buf:
                        vs_c[off + gx] = max(vs_c[off + gx], 320)

        # Narrow outward escape runway ahead of IC / FET / fine pins (protects unrouted pins without over-blocking 0603 passives)
        for pid, p in enumerate(self.pads):
            if not p.net or p.is_tht or (p.ref == "U2" and p.pname == "39"):
                continue
            is_ic = (p.is_fine or p.ref.startswith("U"))
            if not (is_ic or p.ref.startswith("Q")):
                continue
            dx, dy = p.x - p.fp_x, p.y - p.fp_y
            L = math.hypot(dx, dy)
            if L < 0.25:
                continue
            if abs(p.hw - p.hh) > 0.05:
                ux, uy = (p.cos_a, p.sin_a) if p.hw > p.hh else (-p.sin_a, p.cos_a)
                if ux * dx + uy * dy < 0:
                    ux, uy = -ux, -uy
            else:
                ux, uy = dx / L, dy / L
            nx_p, ny_p = -uy, ux
            tip_start = max(p.hw, p.hh) + 0.08
            tip_end = tip_start + (1.15 if is_ic else 0.75)
            w_halo = 135 if is_ic else 75
            w_span = (-0.20, -0.10, 0.0, 0.10, 0.20) if is_ic else (-0.05, 0.0, 0.05)
            steps = 24 if is_ic else 16
            cells = set()
            lay = 0 if p.on_f else 1
            for s in range(steps):
                d_along = tip_start + (tip_end - tip_start) * (s / max(1, steps - 1))
                for w_off in w_span:
                    cx = p.x + d_along * ux + w_off * nx_p
                    cy = p.y + d_along * uy + w_off * ny_p
                    gx, gy = int(round(cx * INV_RES)), int(round(cy * INV_RES))
                    if 0 <= gx < NX and 0 <= gy < NY:
                        cells.add(gy * NX + gx)
            self.pad_halo_cells[pid] = (lay, list(cells), w_halo)
        self._rebuild_soft_cost()

    def _rebuild_soft_cost(self):
        if hasattr(self, "_base_ic_soft_cost"):
            self.soft_cost = [array.array("H", self._base_ic_soft_cost[0]), array.array("H", self._base_ic_soft_cost[1])]
        else:
            self.soft_cost = [array.array("H", [0]) * SIZE, array.array("H", [0]) * SIZE]
            for l in (0, 1):
                sc = self.soft_cost[l]
                for idx in self.ic_guard_cells[l]:
                    sc[idx] += 65
        for pid, entry in enumerate(self.pad_halo_cells):
            if entry and not self.pad_halo_perm_off[pid]:
                self.pad_halo_active[pid] = True
                l, cells, w_halo = entry
                sc = self.soft_cost[l]
                for idx in cells:
                    sc[idx] += w_halo
            else:
                self.pad_halo_active[pid] = False

    def _set_pad_halo(self, pid, desired_active):
        entry = self.pad_halo_cells[pid]
        if not entry or self.pad_halo_active[pid] == desired_active:
            return
        l, cells, w_halo = entry
        sc = self.soft_cost[l]
        delta = w_halo if desired_active else -w_halo
        for idx in cells:
            sc[idx] += delta
        self.pad_halo_active[pid] = desired_active

    def _set_net_halo(self, net, desired_active, permanent=False):
        for pid in self.pads_by_net[net]:
            if permanent:
                self.pad_halo_perm_off[pid] = (not desired_active)
            if desired_active and self.pad_halo_perm_off[pid]:
                continue
            self._set_pad_halo(pid, desired_active)

    def _stamp_seg_to_occ(self, l, x1, y1, x2, y2, tw, net):
        r_t = 0.5 * tw + GRID_BASE_HW + CLEARANCE_MM + 0.012
        r_v = 0.5 * tw + 0.5 * VIA_D + CLEARANCE_MM + 0.012
        gx0 = max(0, int((min(x1, x2) - r_v) * INV_RES))
        gx1 = min(NX - 1, int((max(x1, x2) + r_v) * INV_RES))
        gy0 = max(0, int((min(y1, y2) - r_v) * INV_RES))
        gy1 = min(NY - 1, int((max(y1, y2) + r_v) * INV_RES))
        nt = self.net_trk_cells[net][l]
        nv = self.net_via_cells[net][l]
        to = self.trk_occ[l]
        vo = self.via_occ[l]
        for gy in range(gy0, gy1 + 1):
            ym = gy * RES
            off = gy * NX
            for gx in range(gx0, gx1 + 1):
                d = dist_point_to_seg(gx * RES, ym, x1, y1, x2, y2)
                idx = off + gx
                if d < r_t and idx not in nt:
                    nt.add(idx); to[idx] += 1
                if d < r_v and idx not in nv:
                    nv.add(idx); vo[idx] += 1

    def _stamp_via_to_occ(self, x, y, vd, vdr, net):
        r_t = 0.5 * vd + GRID_BASE_HW + CLEARANCE_MM + 0.012
        r_v_cop = 0.5 * vd + 0.5 * VIA_D + CLEARANCE_MM + 0.012
        r_v_hole = 0.5 * vdr + 0.5 * VIA_DRILL + HOLE_WALL_CLEAR
        r_max = max(r_v_cop, r_v_hole)
        gx0 = max(0, int((x - r_max) * INV_RES))
        gx1 = min(NX - 1, int((x + r_max) * INV_RES))
        gy0 = max(0, int((y - r_max) * INV_RES))
        gy1 = min(NY - 1, int((y + r_max) * INV_RES))
        nh = self.net_hole_cells[net]
        for gy in range(gy0, gy1 + 1):
            ym = gy * RES
            off = gy * NX
            for gx in range(gx0, gx1 + 1):
                d = math.hypot(gx * RES - x, ym - y)
                idx = off + gx
                if d < r_t:
                    for l in (0, 1):
                        if idx not in self.net_trk_cells[net][l]:
                            self.net_trk_cells[net][l].add(idx); self.trk_occ[l][idx] += 1
                if d < r_v_hole:
                    # Same-net vias ALSO must stay hole-to-hole clear!
                    if idx not in nh:
                        nh.add(idx)
                        self.via_occ[0][idx] += 1000
                        self.via_occ[1][idx] += 1000
                elif d < r_v_cop:
                    for l in (0, 1):
                        if idx not in self.net_via_cells[net][l]:
                            self.net_via_cells[net][l].add(idx); self.via_occ[l][idx] += 1

    def add_track(self, layer, x1, y1, x2, y2, w, net):
        if math.hypot(x2 - x1, y2 - y1) < 1e-4:
            return
        ti = len(self.tracks)
        self.tracks.append((layer, x1, y1, x2, y2, w, net))
        for bx in range(max(0, int((min(x1, x2) - 0.8) / 2.0)), min(25, int((max(x1, x2) + 0.8) / 2.0)) + 1):
            for by in range(max(0, int((min(y1, y2) - 0.8) / 2.0)), min(25, int((max(y1, y2) + 0.8) / 2.0)) + 1):
                self.track_bucket[(bx, by)].append(ti)
        self._stamp_seg_to_occ(layer, x1, y1, x2, y2, w, net)

    def via_clears_holes(self, x, y, drill):
        hr = 0.5 * drill
        bx = max(0, min(25, int(x / 2.0)))
        by = max(0, min(25, int(y / 2.0)))
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                for vi in self.via_bucket.get((bx + ddx, by + ddy), ()):
                    vx, vy, _, vdr, _ = self.vias[vi]
                    if math.hypot(x - vx, y - vy) < hr + 0.5 * vdr + HOLE_WALL_CLEAR:
                        return False
                for pid in self.pad_bucket.get((bx + ddx, by + ddy), ()):
                    p = self.pads[pid]
                    if p.drill_r > 0 and math.hypot(x - p.x, y - p.y) < hr + p.drill_r + HOLE_WALL_CLEAR:
                        return False
        return True

    def add_via(self, x, y, d, drill, net):
        if not self.via_clears_holes(x, y, drill):
            print(f"  SKIPPED illegal hole-colliding via for {net} at ({x:.2f}, {y:.2f})")
            return False
        vi = len(self.vias)
        self.vias.append((x, y, d, drill, net))
        self.via_bucket[(max(0, min(25, int(x / 2.0))), max(0, min(25, int(y / 2.0))))].append(vi)
        self._stamp_via_to_occ(x, y, d, drill, net)
        return True

    def snapshot_state(self):
        return (
            list(self.tracks),
            list(self.vias),
            list(self.pad_halo_perm_off),
            list(self.pad_halo_active),
            [array.array("H", self.trk_occ[0]), array.array("H", self.trk_occ[1])],
            [array.array("H", self.via_occ[0]), array.array("H", self.via_occ[1])],
            [array.array("H", self.hard_occ[0]), array.array("H", self.hard_occ[1])],
            [array.array("H", self.soft_cost[0]), array.array("H", self.soft_cost[1])],
            {k: list(v) for k, v in self.track_bucket.items()},
            {k: list(v) for k, v in self.via_bucket.items()},
            {k: [set(v[0]), set(v[1])] for k, v in self.net_trk_cells.items()},
            {k: [set(v[0]), set(v[1])] for k, v in self.net_via_cells.items()},
            {k: set(v) for k, v in self.net_hole_cells.items()},
        )

    def restore_state(self, snap):
        """Fast exact rollback from copied occupancy grids and cell sets."""
        (tracks, vias, perm_off, halo_active,
         trk_o, via_o, hard_o, soft_c,
         t_bkt, v_bkt, n_trk, n_via, n_hole) = snap
        self.tracks = list(tracks)
        self.vias = list(vias)
        self.pad_halo_perm_off = list(perm_off)
        self.pad_halo_active = list(halo_active)
        self.trk_occ = [array.array("H", trk_o[0]), array.array("H", trk_o[1])]
        self.via_occ = [array.array("H", via_o[0]), array.array("H", via_o[1])]
        self.hard_occ = [array.array("H", hard_o[0]), array.array("H", hard_o[1])]
        self.soft_cost = [array.array("H", soft_c[0]), array.array("H", soft_c[1])]
        self.track_bucket = defaultdict(list, {k: list(v) for k, v in t_bkt.items()})
        self.via_bucket = defaultdict(list, {k: list(v) for k, v in v_bkt.items()})
        self.net_trk_cells = defaultdict(lambda: [set(), set()], {k: [set(v[0]), set(v[1])] for k, v in n_trk.items()})
        self.net_via_cells = defaultdict(lambda: [set(), set()], {k: [set(v[0]), set(v[1])] for k, v in n_via.items()})
        self.net_hole_cells = defaultdict(set, {k: set(v) for k, v in n_hole.items()})

    def remove_nets_routing(self, net_set):
        for net in net_set:
            if net in self.net_trk_cells:
                tc = self.net_trk_cells.pop(net)
                for l in (0, 1):
                    to = self.trk_occ[l]
                    for idx in tc[l]:
                        to[idx] -= 1
            if net in self.net_via_cells:
                vc = self.net_via_cells.pop(net)
                for l in (0, 1):
                    vo = self.via_occ[l]
                    for idx in vc[l]:
                        vo[idx] -= 1
            if net in self.net_hole_cells:
                nh = self.net_hole_cells.pop(net)
                vo0, vo1 = self.via_occ[0], self.via_occ[1]
                for idx in nh:
                    vo0[idx] -= 1000
                    vo1[idx] -= 1000
            self._set_net_halo(net, True, permanent=True)

        self.tracks = [t for t in self.tracks if t[6] not in net_set]
        self.vias = [v for v in self.vias if v[4] not in net_set]
        self.track_bucket.clear()
        for ti, (l, x1, y1, x2, y2, tw, _) in enumerate(self.tracks):
            for bx in range(max(0, int((min(x1, x2) - 0.8) / 2.0)), min(25, int((max(x1, x2) + 0.8) / 2.0)) + 1):
                for by in range(max(0, int((min(y1, y2) - 0.8) / 2.0)), min(25, int((max(y1, y2) + 0.8) / 2.0)) + 1):
                    self.track_bucket[(bx, by)].append(ti)
        self.via_bucket.clear()
        for vi, (vx, vy, _, _, _) in enumerate(self.vias):
            self.via_bucket[(max(0, min(25, int(vx / 2.0))), max(0, min(25, int(vy / 2.0))))].append(vi)

    def _toggle_net(self, net, sign):
        for pid in self.pads_by_net[net]:
            t_f, t_b = self.pad_trk_stamp[pid]
            v_f, v_b = self.pad_via_stamp[pid]
            if sign < 0:
                for idx in t_f: self.trk_occ[0][idx] -= 1; self.hard_occ[0][idx] -= 1
                for idx in t_b: self.trk_occ[1][idx] -= 1; self.hard_occ[1][idx] -= 1
                for idx in v_f: self.via_occ[0][idx] -= 1
                for idx in v_b: self.via_occ[1][idx] -= 1
            else:
                for idx in t_f: self.trk_occ[0][idx] += 1; self.hard_occ[0][idx] += 1
                for idx in t_b: self.trk_occ[1][idx] += 1; self.hard_occ[1][idx] += 1
                for idx in v_f: self.via_occ[0][idx] += 1
                for idx in v_b: self.via_occ[1][idx] += 1
        if sign < 0:
            self._set_net_halo(net, False)
        else:
            self._set_net_halo(net, True)
        for l in (0, 1):
            to = self.trk_occ[l]; vo = self.via_occ[l]
            for idx in self.net_trk_cells[net][l]: to[idx] += sign
            for idx in self.net_via_cells[net][l]: vo[idx] += sign

    def inside_safe_board(self, layer_idx, x, y, radius, is_via=False):
        margin = radius + 0.31
        if x < margin or x > BOARD_W - margin or y < max(margin, ANTENNA_KEEPOUT_Y) or y > BOARD_H - margin:
            return False
        r = CORNER_R
        for cx, cy in [(r, BOARD_H - r), (BOARD_W - r, BOARD_H - r)]:
            in_x = (x < r) if cx < BOARD_W / 2 else (x > BOARD_W - r)
            if in_x and y > BOARD_H - r:
                if math.hypot(x - cx, y - cy) > r - margin:
                    return False
        for on_f, on_b, no_trk, no_via, kx0, ky0, kx1, ky1 in self.keepouts:
            if is_via and no_via and (on_f or on_b):
                if (kx0 - radius - 0.02) <= x <= (kx1 + radius + 0.02) and (ky0 - radius - 0.02) <= y <= (ky1 + radius + 0.02):
                    return False
            elif not is_via and no_trk:
                if (layer_idx == -1 or (layer_idx == 0 and on_f) or (layer_idx == 1 and on_b)):
                    if (kx0 - radius - 0.02) <= x <= (kx1 + radius + 0.02) and (ky0 - radius - 0.02) <= y <= (ky1 + radius + 0.02):
                        return False
        return True

    def point_clears_world(self, layer_idx, x, y, radius, net, clr=CLEARANCE_MM, is_via=False, drill_r=0.0, skip_via_xy=None):
        if not self.inside_safe_board(layer_idx, x, y, radius, is_via=is_via):
            return False
        bx = max(0, min(25, int(x / 2.0)))
        by = max(0, min(25, int(y / 2.0)))
        seen_p = set()
        seen_t = set()
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                for pid in self.pad_bucket.get((bx + ddx, by + ddy), ()):
                    if pid in seen_p:
                        continue
                    seen_p.add(pid)
                    p = self.pads[pid]
                    if p.drill_r > 0 and is_via:
                        # ALL drilled holes (incl. same net!) must maintain hole wall clearance!
                        if math.hypot(x - p.x, y - p.y) < drill_r + p.drill_r + HOLE_WALL_CLEAR:
                            return False
                    if p.net == net:
                        continue
                    on_same = (layer_idx == -1) or (layer_idx == 0 and p.on_f) or (layer_idx == 1 and p.on_b)
                    if on_same and p.dist_to_point(x, y) < radius + clr:
                        return False
                    if p.drill_r > 0 and math.hypot(x - p.x, y - p.y) < radius + p.drill_r + 0.26:
                        return False
                for vi in self.via_bucket.get((bx + ddx, by + ddy), ()):
                    vx, vy, vd, vdr, vnet = self.vias[vi]
                    if skip_via_xy and vnet == net and abs(vx - skip_via_xy[0]) < 1e-4 and abs(vy - skip_via_xy[1]) < 1e-4:
                        continue
                    if is_via and math.hypot(x - vx, y - vy) < drill_r + 0.5 * vdr + HOLE_WALL_CLEAR:
                        return False
                    if vnet != net and math.hypot(x - vx, y - vy) < radius + 0.5 * vd + clr:
                        return False
                for ti in self.track_bucket.get((bx + ddx, by + ddy), ()):
                    if ti in seen_t:
                        continue
                    seen_t.add(ti)
                    tlay, x1, y1, x2, y2, tw, tnet = self.tracks[ti]
                    if tnet != net and (layer_idx == -1 or tlay == layer_idx):
                        if dist_point_to_seg(x, y, x1, y1, x2, y2) < radius + 0.5 * tw + clr:
                            return False
        return True

    def seg_clears_world(self, layer_idx, x1, y1, x2, y2, w, net, clr=CLEARANCE_MM):
        hw = 0.5 * w
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(2, int(math.ceil(length / 0.05)))
        for k in range(steps + 1):
            t = k / steps
            if not self.point_clears_world(layer_idx, x1 + t * (x2 - x1), y1 + t * (y2 - y1), hw, net, clr):
                return False
        return True

    def find_escape_pt(self, pad_idx, w):
        p = self.pads[pad_idx]
        outward_dirs = []
        fallback_dirs = []
        out_u = None
        if abs(p.hw - p.hh) > 0.05:
            ux, uy = (p.cos_a, p.sin_a) if p.hw > p.hh else (-p.sin_a, p.cos_a)
            if ux * (p.x - p.fp_x) + uy * (p.y - p.fp_y) < 0:
                ux, uy = -ux, -uy
            out_u = (ux, uy)
            outward_dirs.append((ux, uy))
            fallback_dirs.append((-ux, -uy))
        dx, dy = p.x - p.fp_x, p.y - p.fp_y
        L = math.hypot(dx, dy)
        if L > 0.05:
            outward_dirs.append((dx / L, dy / L))
        for dvec in [
            (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
            (0.7071, 0.7071), (0.7071, -0.7071), (-0.7071, 0.7071), (-0.7071, -0.7071),
        ]:
            if out_u and (dvec[0] * out_u[0] + dvec[1] * out_u[1]) < -0.5:
                fallback_dirs.append(dvec)
            else:
                outward_dirs.append(dvec)

        tip_d = max(p.hw, p.hh) + 0.30
        dists = [tip_d - 0.08, tip_d, tip_d + 0.25, tip_d + 0.55, 0.0, 0.45, 0.70] if p.is_fine or max(p.hw, p.hh) >= 0.42 else [0.0, 0.45, 0.75, 1.05]
        layers_try = [1, 0] if p.is_tht else [0 if p.on_f else 1]
        w_e = min(w, 0.18 if p.is_fine else w)

        dir_groups = (outward_dirs, fallback_dirs) if p.is_fine and fallback_dirs else (outward_dirs + fallback_dirs,)
        for l_try in layers_try:
            grid = self.trk_occ[l_try]
            for dgroup in dir_groups:
                for d in dists:
                    if d == 0.0:
                        gx, gy = to_grid(p.x), to_grid(p.y)
                        mx, my = to_mm(gx), to_mm(gy)
                        if grid[gy * NX + gx] == 0 and self.point_clears_world(l_try, mx, my, 0.5 * w_e, p.net, clr=0.205):
                            return (l_try, mx, my)
                        continue
                    for ux, uy in dgroup:
                        gx, gy = to_grid(p.x + d * ux), to_grid(p.y + d * uy)
                        mx, my = to_mm(gx), to_mm(gy)
                        if grid[gy * NX + gx] == 0 and self.seg_clears_world(l_try, p.x, p.y, mx, my, w_e, p.net, clr=0.205):
                            return (l_try, mx, my)
        return None

    def astar_route(self, start_pts, goal_pts, minx=0, maxx=NX - 1, miny=0, maxy=NY - 1,
                    max_exp=34000, allow_congestion=False, h_num=12, h_den=10, cong_pen=45):
        g_set = {(l, gx, gy) for l, gx, gy in goal_pts if 0 <= gx < NX and 0 <= gy < NY}
        if not g_set:
            return None
        unique_g = sorted({(gx, gy) for _, gx, gy in g_set})
        if len(unique_g) <= 4:
            goal_coords = unique_g
        else:
            step_g = max(1, len(unique_g) // 4)
            goal_coords = [unique_g[i] for i in range(0, len(unique_g), step_g)][:4]

        if len(goal_coords) == 1:
            tx0, ty0 = goal_coords[0]

            def h_cost(x, y):
                dx = x - tx0
                if dx < 0:
                    dx = -dx
                dy = y - ty0
                if dy < 0:
                    dy = -dy
                d = 10 * dx + 4 * dy if dx > dy else 10 * dy + 4 * dx
                return (d * h_num) // h_den
        else:
            def h_cost(x, y):
                best = 10**9
                for tx, ty in goal_coords:
                    dx = abs(x - tx); dy = abs(y - ty)
                    d = 10 * (dx if dx > dy else dy) + 4 * (dy if dx > dy else dx)
                    if d < best:
                        best = d
                return (best * h_num) // h_den

        dist = {}
        parent = {}
        pq = []
        for l, gx, gy in start_pts:
            if not (0 <= gx < NX and 0 <= gy < NY):
                continue
            sid = l * SIZE + gy * NX + gx
            dist[sid] = 0; parent[sid] = -1
            heapq.heappush(pq, (h_cost(gx, gy), 0, sid))

        moves = (
            (1, 0, 10), (-1, 0, 10), (0, 1, 10), (0, -1, 10),
            (1, 1, 14), (1, -1, 14), (-1, 1, 14), (-1, -1, 14),
        )
        VIA_COST = 105
        CONG_PEN = cong_pen
        to = self.trk_occ
        ho = self.hard_occ
        vo = self.via_occ
        sc_arr = self.soft_cost
        hi_arr = self.hist_cost
        vs_arr = self.via_soft_cost
        exp = 0
        found_sid = -1

        while pq:
            _, g, sid = heapq.heappop(pq)
            if g > dist.get(sid, 10**9):
                continue
            l = sid // SIZE
            rem = sid - l * SIZE
            gy = rem // NX
            gx = rem - gy * NX

            if (l, gx, gy) in g_set:
                found_sid = sid
                break

            exp += 1
            if exp > max_exp:
                break

            grid = to[l]
            hgrid = ho[l]
            soft = sc_arr[l]
            hist = hi_arr[l]
            p_sid = parent[sid]
            pdx = pdy = 0
            if p_sid >= 0 and (p_sid // SIZE) == l:
                prem = p_sid - l * SIZE
                pdx = gx - (prem % NX)
                pdy = gy - (prem // NX)

            for dx, dy, sc in moves:
                nx_g = gx + dx; ny_g = gy + dy
                if nx_g < minx or nx_g > maxx or ny_g < miny or ny_g > maxy:
                    continue
                nrem = ny_g * NX + nx_g
                is_goal = (l, nx_g, ny_g) in g_set
                occ_v = grid[nrem]
                if not is_goal:
                    if allow_congestion:
                        if hgrid[nrem] > 0:
                            continue
                    elif occ_v > 0:
                        continue
                if dx != 0 and dy != 0:
                    if allow_congestion:
                        if hgrid[gy * NX + nx_g] > 0 and hgrid[ny_g * NX + gx] > 0:
                            continue
                    elif grid[gy * NX + nx_g] > 0 and grid[ny_g * NX + gx] > 0:
                        continue
                turn_pen = 0 if (pdx == dx and pdy == dy) else 2
                dir_bias = 2 if ((l == 0 and dx == 0 and dy != 0) or (l == 1 and dy == 0 and dx != 0)) else 0
                cong = (CONG_PEN * occ_v) if (allow_congestion and occ_v > 0 and not is_goal) else 0
                ng = g + sc + turn_pen + dir_bias + soft[nrem] + hist[nrem] + cong
                nsid = l * SIZE + nrem
                if ng < dist.get(nsid, 10**9):
                    dist[nsid] = ng; parent[nsid] = sid
                    heapq.heappush(pq, (ng + h_cost(nx_g, ny_g), ng, nsid))

            v_occ_sum = vo[0][rem] + vo[1][rem]
            if v_occ_sum == 0 or (allow_congestion and vo[0][rem] < 1000 and vo[1][rem] < 1000):
                too_close_via = False
                cur_p = sid
                prev_l = l
                for _ in range(40):
                    pp = parent.get(cur_p, -1)
                    if pp < 0:
                        break
                    pl = pp // SIZE
                    if pl != prev_l:
                        prem = cur_p - prev_l * SIZE
                        vx_g = prem % NX
                        vy_g = prem // NX
                        if (gx - vx_g) * (gx - vx_g) + (gy - vy_g) * (gy - vy_g) < 289:
                            too_close_via = True
                        break
                    cur_p = pp
                if not too_close_via:
                    ol = 1 - l
                    nsid = ol * SIZE + rem
                    v_cong = (CONG_PEN * v_occ_sum) if (allow_congestion and v_occ_sum > 0) else 0
                    ng = g + VIA_COST + v_cong + vs_arr[rem] + ((sc_arr[ol][rem] + hi_arr[ol][rem] + sc_arr[l][rem] + hi_arr[l][rem]) >> 1)
                    if ng < dist.get(nsid, 10**9):
                        dist[nsid] = ng; parent[nsid] = sid
                        heapq.heappush(pq, (ng + h_cost(gx, gy), ng, nsid))

        if found_sid < 0:
            self.last_exhausted_empty = (len(pq) == 0)
            return None
        self.last_exhausted_empty = False

        raw = []
        cur = found_sid
        while cur >= 0:
            l = cur // SIZE
            rem = cur - l * SIZE
            raw.append((l, rem % NX, rem // NX))
            cur = parent[cur]
        raw.reverse()
        return raw

    def _route_corridor(self, s_pts, t_pts, allow_congestion=False, fast=False, cong_pen=18, tier_idx=None):
        xs = [p[1] for p in s_pts + t_pts]
        ys = [p[2] for p in s_pts + t_pts]
        span_g = (max(xs) - min(xs)) + (max(ys) - min(ys))
        if allow_congestion:
            sched = ((NX if span_g > 320 else 200, 14000 if fast else 45000, 15), (NX, 42000 if fast else 150000, 13))
        elif span_g > 320:
            sched = ((NX, 28000 if fast else 100000, 15 if fast else 14), (NX, 65000 if fast else 240000, 13))
        else:
            sched = ((160, 18000 if fast else 55000, 15 if fast else 14), (NX, 55000 if fast else 220000, 13))
        if tier_idx is not None:
            sched = (sched[tier_idx],)
        for pad_g, me, hn in sched:
            res = self.astar_route(
                s_pts, t_pts,
                max(0, min(xs) - pad_g), min(NX - 1, max(xs) + pad_g),
                max(0, min(ys) - pad_g), min(NY - 1, max(ys) + pad_g),
                max_exp=me, allow_congestion=allow_congestion, h_num=hn, cong_pen=cong_pen,
            )
            if res is not None:
                return res
            if self.last_exhausted_empty and (min(xs) - pad_g <= 0) and (max(xs) + pad_g >= NX - 1) and (min(ys) - pad_g <= 0) and (max(ys) + pad_g >= NY - 1):
                return None
        return None

    def _find_colliding_nets(self, raw_gpts, exclude_nets):
        hits = Counter()
        for idx_p in range(0, len(raw_gpts), 2):
            l, gx, gy = raw_gpts[idx_p]
            xm, ym = to_mm(gx), to_mm(gy)
            bx, by = max(0, min(25, int(xm / 2.0))), max(0, min(25, int(ym / 2.0)))
            if self.trk_occ[l][gy * NX + gx] > 0 or self.via_occ[l][gy * NX + gx] > 0:
                self.hist_cost[l][gy * NX + gx] = min(700, self.hist_cost[l][gy * NX + gx] + 60)
            for ddx in (-1, 0, 1):
                for ddy in (-1, 0, 1):
                    for ti in self.track_bucket.get((bx + ddx, by + ddy), ()):
                        tlay, x1, y1, x2, y2, tw, tnet = self.tracks[ti]
                        if tlay == l and tnet not in exclude_nets:
                            if dist_point_to_seg(xm, ym, x1, y1, x2, y2) < 0.5 * tw + GRID_BASE_HW + CLEARANCE_MM + 0.05:
                                hits[tnet] += 1
                    for vi in self.via_bucket.get((bx + ddx, by + ddy), ()):
                        vx, vy, vd, _, vnet = self.vias[vi]
                        if vnet not in exclude_nets:
                            if math.hypot(xm - vx, ym - vy) < 0.5 * vd + GRID_BASE_HW + CLEARANCE_MM + 0.05:
                                hits[vnet] += 1
        return [net for net, _ in hits.most_common()]

    def _commit_path(self, raw_gpts, w, net):
        """Simplify collinear steps safely (never violating clearances or keepouts) and commit."""
        pts_mm = [(l, to_mm(gx), to_mm(gy)) for l, gx, gy in raw_gpts]
        runs = []
        cur_l = pts_mm[0][0]
        cur_pts = [(pts_mm[0][1], pts_mm[0][2])]
        for l, x, y in pts_mm[1:]:
            if l == cur_l:
                cur_pts.append((x, y))
            else:
                runs.append((cur_l, cur_pts))
                cur_l = l
                cur_pts = [(x, y)]
        runs.append((cur_l, cur_pts))

        added_nodes = []
        for idx, (l, pts) in enumerate(runs):
            # Merge strictly collinear points
            col = [pts[0]]
            for p in pts[1:]:
                if len(col) >= 2:
                    dx1, dy1 = col[-1][0] - col[-2][0], col[-1][1] - col[-2][1]
                    dx2, dy2 = p[0] - col[-1][0], p[1] - col[-1][1]
                    if abs(dx1 * dy2 - dy1 * dx2) < 1e-6 and (dx1 * dx2 + dy1 * dy2) >= 0:
                        col[-1] = p
                        continue
                col.append(p)
            # Safe continuous shortcut (only shortcut if clr=0.218 passes EVERY step!)
            if len(col) > 2:
                sm = [col[0]]
                i = 0
                while i < len(col) - 1:
                    best_j = i + 1
                    for j in range(min(len(col) - 1, i + 12), i + 1, -1):
                        if self.seg_clears_world(l, col[i][0], col[i][1], col[j][0], col[j][1], min(w, 0.22), net, clr=CLEARANCE_MM):
                            best_j = j
                            break
                    sm.append(col[best_j])
                    i = best_j
                col = sm

            for a, b in zip(col[:-1], col[1:]):
                seg_w = w
                if w > 0.20 and not self.seg_clears_world(l, a[0], a[1], b[0], b[1], w, net, clr=CLEARANCE_MM):
                    seg_w = 0.22 if self.seg_clears_world(l, a[0], a[1], b[0], b[1], 0.22, net, clr=CLEARANCE_MM) else 0.18
                self.add_track(l, a[0], a[1], b[0], b[1], seg_w, net)
                added_nodes.append((l, to_grid(a[0]), to_grid(a[1])))
                added_nodes.append((l, to_grid(b[0]), to_grid(b[1])))
            if idx > 0:
                vx, vy = col[0]
                self.add_via(vx, vy, VIA_D, VIA_DRILL, net)
                added_nodes.append((0, to_grid(vx), to_grid(vy)))
                added_nodes.append((1, to_grid(vx), to_grid(vy)))
        return added_nodes

    def net_priority(self, net):
        if net in ("BUCK_SW", "BUCK_FB", "VCAP_1", "HSE_IN", "HSE_OUT", "MPU_INT",
                   "MPU_REGOUT", "MPU_CPOUT", "V_REF_2V5", "V_BAT_ADC", "V_BATT_RAW", "GATE_BAT", "GATE_SCAP",
                   "PFD_REF", "PFD_DIV", "BEACON_G"):
            return (0, net)
        if net in FINE_NETS or net in ("BOOT0", "ESP_EN", "ESP_IO0", "BEACON_CTRL", "STAT_LED", "NRST", "PFD_INT",
                                       "SWCLK", "SWDIO", "USART1_TX", "USART1_RX", "I2C1_SCL", "SPI1_SCK",
                                       "SD_CD", "LSE_IN", "LSE_OUT"):
            return (1, net)
        if net in POWER_NETS:
            return (2, net)
        return (3, net)

    def net_span(self, net):
        pids = self.pads_by_net[net]
        xs = [self.pads[i].x for i in pids]
        ys = [self.pads[i].y for i in pids]
        return (max(xs) - min(xs)) + (max(ys) - min(ys))

    def net_width(self, net):
        if net in POWER_NETS:
            return 0.36
        if net in FINE_NETS:
            return 0.18
        return 0.22

    def _route_single_net(self, net, fast=False, debug_fail=False, allow_congestion=False, cong_pen=45):
        """
        Transactionally route ALL pins of `net` via Prim-like tree growth.
        Never leaves dangling escape stubs when an edge fails!
        """
        pids = list(self.pads_by_net[net])
        if len(pids) < 2:
            self._set_net_halo(net, False, permanent=True)
            return True
        w = self.net_width(net)

        self._toggle_net(net, -1)
        esc = {}
        for pid in pids:
            p = self.pads[pid]
            e = self.find_escape_pt(pid, min(w, 0.18 if p.is_fine else 0.24))
            if e is None:
                e = (0 if p.on_f else 1, to_mm(to_grid(p.x)), to_mm(to_grid(p.y)))
            esc[pid] = e
        self._toggle_net(net, +1)

        def esc_node(pid):
            el, ex, ey = esc[pid]
            return (el, to_grid(ex), to_grid(ey))

        comp_pads = {pids[0]}
        unconn = set(pids[1:])
        comp_nodes = [esc_node(pids[0])]
        committed_stubs = set()
        unreachable_v = set()

        while unconn:
            cands = []
            for u in comp_pads:
                for v in unconn:
                    if v not in unreachable_v:
                        d_uv = (math.hypot(self.pads[u].x - self.pads[v].x, self.pads[u].y - self.pads[v].y)
                                + (0.0 if self.pads[u].on_f == self.pads[v].on_f else 2.0))
                        cands.append((d_uv, u, v))
            if not cands:
                break
            _, u, v = min(cands, key=lambda x: x[0])
            su = esc_node(u)
            sv = esc_node(v)
            s_cands = list(dict.fromkeys([su] + comp_nodes))

            self._toggle_net(net, -1)
            if sv in s_cands:
                raw = [sv]
            else:
                fwd_first = (len(comp_pads) == 1) and (self.pads[u].is_fine or self.pads[u].ref in ("U1", "U2", "U3", "U4", "U5", "U6")) and not (self.pads[v].ref == "U7")
                a_pts, b_pts = (s_cands, [sv]) if fwd_first else ([sv], s_cands)
                raw = self._route_corridor(a_pts, b_pts, fast=fast, allow_congestion=allow_congestion, cong_pen=cong_pen, tier_idx=0)
                if raw is None and len(a_pts) > 0:
                    raw = self._route_corridor(b_pts, a_pts, fast=fast, allow_congestion=allow_congestion, cong_pen=cong_pen, tier_idx=0)
                if raw is None:
                    raw = self._route_corridor(a_pts, b_pts, fast=fast, allow_congestion=allow_congestion, cong_pen=cong_pen, tier_idx=1)
                    if raw is None and len(a_pts) > 0 and not getattr(self, "last_exhausted_empty", False):
                        raw = self._route_corridor(b_pts, a_pts, fast=fast, allow_congestion=allow_congestion, cong_pen=cong_pen, tier_idx=1)
            self._toggle_net(net, +1)

            if raw is None:
                unreachable_v.add(v)
                continue

            unreachable_v.clear()
            skip_stub = set()
            sv_node = sv
            if len(raw) >= 3:
                for pid, end_pt in ((u, s_cands[0] if len(s_cands) == 1 else None), (v, sv)):
                    if pid in committed_stubs or end_pt is None or len(raw) < 3:
                        continue
                    p = self.pads[pid]
                    el, ex, ey = esc[pid]
                    if math.hypot(ex - p.x, ey - p.y) <= 0.35:
                        continue
                    pg = (el, to_grid(p.x), to_grid(p.y))
                    if raw[0] == end_pt:
                        k_hit = None
                        for idx in range(len(raw) - 2, 0, -1):
                            rl, rgx, rgy = raw[idx]
                            if rl == el and math.hypot(to_mm(rgx) - p.x, to_mm(rgy) - p.y) < 0.30:
                                k_hit = idx
                                break
                        if k_hit is not None:
                            raw = [pg] + raw[k_hit:]
                            skip_stub.add(pid)
                            if pid == v:
                                sv_node = pg
                    elif raw[-1] == end_pt:
                        k_hit = None
                        for idx in range(1, len(raw) - 1):
                            rl, rgx, rgy = raw[idx]
                            if rl == el and math.hypot(to_mm(rgx) - p.x, to_mm(rgy) - p.y) < 0.30:
                                k_hit = idx
                                break
                        if k_hit is not None:
                            raw = raw[:k_hit + 1] + [pg]
                            skip_stub.add(pid)
                            if pid == v:
                                sv_node = pg
            for pid in skip_stub:
                p_sk = self.pads[pid]
                el_sk = esc[pid][0]
                old_pt = (el_sk, to_grid(esc[pid][1]), to_grid(esc[pid][2]))
                pg_sk = (el_sk, to_grid(p_sk.x), to_grid(p_sk.y))
                esc[pid] = (el_sk, to_mm(pg_sk[1]), to_mm(pg_sk[2]))
                comp_nodes = [pg_sk if n == old_pt else n for n in comp_nodes]
            for pid in (u, v):
                if pid not in committed_stubs:
                    committed_stubs.add(pid)
                    if pid in skip_stub:
                        continue
                    p = self.pads[pid]
                    el, ex, ey = esc[pid]
                    if math.hypot(ex - p.x, ey - p.y) > 0.01:
                        self.add_track(el, p.x, p.y, ex, ey, min(w, 0.18 if p.is_fine else 0.24), net)
            if len(raw) >= 2:
                added = self._commit_path(raw, w, net)
                comp_nodes.extend(added[::max(1, len(added) // 6)])
            comp_nodes.append(sv_node)
            comp_pads.add(v)
            unconn.remove(v)

        ok = (len(unconn) == 0)
        if ok:
            self._set_net_halo(net, False, permanent=True)
        elif debug_fail:
            u_names = [f"{self.pads[i].ref}.{self.pads[i].pname}@({self.pads[i].x:.1f},{self.pads[i].y:.1f},{'F' if self.pads[i].on_f else 'B'})" for i in sorted(unconn)[:4]]
            c_names = [f"{self.pads[i].ref}.{self.pads[i].pname}" for i in sorted(comp_pads)[:3]]
            print(f"    [FAIL_EDGE] {net}: reached {c_names} ({len(comp_pads)}/{len(pids)}), unconn={u_names}")
        return ok

    def _find_congestion_blockers(self, net):
        """Route `net` ignoring other signal tracks (at high penalty) to pinpoint exact blocking nets."""
        pids = list(self.pads_by_net[net])
        if len(pids) < 2:
            return []
        w = self.net_width(net)
        self._toggle_net(net, -1)
        esc = {}
        for pid in pids:
            p = self.pads[pid]
            e = self.find_escape_pt(pid, min(w, 0.18 if p.is_fine else 0.24))
            if e is None:
                e = (0 if p.on_f else 1, to_mm(to_grid(p.x)), to_mm(to_grid(p.y)))
            esc[pid] = e

        def esc_node(pid):
            el, ex, ey = esc[pid]
            return (el, to_grid(ex), to_grid(ey))

        comp = {pids[0]}
        comp_nodes = [esc_node(pids[0])]
        rem = set(pids[1:])
        colliders = []
        seen = set()
        unreach = set()
        while rem:
            cands = [(math.hypot(self.pads[u].x - self.pads[v].x, self.pads[u].y - self.pads[v].y), u, v)
                     for u in comp for v in rem if v not in unreach]
            if not cands:
                break
            _, u, v = min(cands, key=lambda x: x[0])
            su, sv = esc_node(u), esc_node(v)
            s_cands = list(dict.fromkeys([su] + comp_nodes))
            raw = [sv] if sv in s_cands else self.astar_route(
                s_cands, [sv], 0, NX - 1, 0, NY - 1,
                max_exp=140000, allow_congestion=True, h_num=13, cong_pen=10,
            )
            if raw is None and sv not in s_cands:
                raw = self.astar_route(
                    [sv], s_cands, 0, NX - 1, 0, NY - 1,
                    max_exp=160000, allow_congestion=True, h_num=12, cong_pen=3,
                )
            if raw is None:
                unreach.add(v)
                continue
            unreach.clear()
            for cn in self._find_colliding_nets(raw, {"GND", "+3V3", net}):
                if cn not in seen:
                    seen.add(cn); colliders.append(cn)
            comp.add(v)
            comp_nodes.append(sv)
            rem.remove(v)
        self._toggle_net(net, +1)
        return colliders

    def pre_fanout_safe_plane_vias(self):
        """Drop non-blocking inward U1 power vias and open bypass-cap GND/+3V3 vias before signals."""
        self.fanned_plane_pids = set()
        pre_cnt = 0
        for net_name in ("GND", "+3V3"):
            for pid in self.pads_by_net[net_name]:
                p = self.pads[pid]
                if p.is_tht or (p.ref == "U2" and p.pname == "39"):
                    continue
                if not (p.ref == "U1" or p.ref in ("U6", "U7", "U8", "U9", "U10", "U11") or p.ref[0] in ("C", "R", "L", "D") or p.ref.startswith("FB")):
                    continue
                src_lay = 0 if p.on_f else 1
                vd, vdr = (PWR_VIA_D, PWR_VIA_DRILL) if p.ref in ("C27", "C28", "C29", "L1", "U6") else (VIA_D, VIA_DRILL)
                w_stub = 0.18 if p.is_fine else 0.25
                in_dx, in_dy = p.fp_x - p.x, p.fp_y - p.y
                in_len = math.hypot(in_dx, in_dy)

                cand_pts = []
                if p.ref == "U1" and in_len > 1.0:
                    ux, uy = in_dx / in_len, in_dy / in_len
                    dists = (1.35, 1.65, 2.00, 2.40, 2.85)
                    angles = (0, 22, -22, 38, -38)
                    for d in dists:
                        for ang in angles:
                            ar = math.radians(ang)
                            ca, sa = math.cos(ar), math.sin(ar)
                            vx = to_mm(to_grid(p.x + d * (ux * ca - uy * sa)))
                            vy = to_mm(to_grid(p.y + d * (ux * sa + uy * ca)))
                            # Must stay strictly inside U1 perimeter ring
                            if math.hypot(vx - p.fp_x, vy - p.fp_y) < in_len - 0.8:
                                cand_pts.append((vx, vy))
                elif p.ref in ("U6", "U7", "U8", "U9", "U10", "U11") or p.ref[0] in ("C", "R", "L", "D") or p.ref.startswith("FB"):
                    ux, uy = (-in_dx / in_len, -in_dy / in_len) if in_len > 0.05 else (1.0, 0.0)
                    for d in (0.75, 0.95, 1.20):
                        for ang in (0, 90, -90, 45, -45, 180):
                            ar = math.radians(ang)
                            ca, sa = math.cos(ar), math.sin(ar)
                            vx = to_mm(to_grid(p.x + d * (ux * ca - uy * sa)))
                            vy = to_mm(to_grid(p.y + d * (ux * sa + uy * ca)))
                            cand_pts.append((vx, vy))

                self._set_pad_halo(pid, False)
                placed = False
                for vx, vy in cand_pts:
                    gx, gy = to_grid(vx), to_grid(vy)
                    if p.ref != "U1" and self.soft_cost[src_lay][gy * NX + gx] >= 260:
                        continue
                    if (self.point_clears_world(-1, vx, vy, 0.5 * vd, net_name, clr=CLEARANCE_MM, is_via=True, drill_r=0.5 * vdr) and
                            self.seg_clears_world(src_lay, p.x, p.y, vx, vy, w_stub, net_name, clr=0.210)):
                        if self.add_via(vx, vy, vd, vdr, net_name):
                            self.add_track(src_lay, p.x, p.y, vx, vy, w_stub, net_name)
                            self.fanned_plane_pids.add(pid)
                            self.pad_halo_perm_off[pid] = True
                            pre_cnt += 1
                            placed = True
                            break
                if not placed:
                    self._set_pad_halo(pid, not self.pad_halo_perm_off[pid])
        print(f"Pre-fanout placed {pre_cnt} non-blocking GND/+3V3 vias before signal routing")

    def _reroute_with_cascade(self, ordered_nets, locked_nets, fast=True, allow_cascade=True, deadline=None):
        """Route ordered_nets; optionally perform bounded 1-hop congestion cascade against unlocked nets."""
        protect = set(locked_nets)
        todo = list(ordered_nets)
        cascades = 0
        while todo:
            if deadline and time.time() > deadline:
                return False
            rn = todo.pop(0)
            if rn in protect:
                continue
            if self._route_single_net(rn, fast=fast):
                protect.add(rn)
                continue
            if not allow_cascade or cascades >= 2:
                return False
            cascades += 1
            self.remove_nets_routing({rn})
            sub_blockers = [b for b in self._find_congestion_blockers(rn) if b not in protect and b not in todo][:3]
            if not sub_blockers:
                return False
            self.remove_nets_routing({rn} | set(sub_blockers))
            if not self._route_single_net(rn, fast=fast):
                return False
            protect.add(rn)
            for sn in sub_blockers:
                if not self._route_single_net(sn, fast=fast):
                    return False
                protect.add(sn)
        return True

    def _audit_cluster_conflicts(self, cluster, bump_hist=True):
        """Detect exact DRC (<0.208 mm) intersections/clearances across `cluster`, bump hist_cost, and pull external colliders."""
        conflicting = set()
        ext_colliders = set()
        clr = 0.204

        for tlay, x1, y1, x2, y2, tw, tnet in self.tracks:
            if tnet not in cluster:
                continue
            length = math.hypot(x2 - x1, y2 - y1)
            steps = max(2, int(math.ceil(length / 0.14)))
            rad = 0.5 * tw
            hit_here = False
            for k in range(steps + 1):
                t = k / steps
                mx = x1 + t * (x2 - x1)
                my = y1 + t * (y2 - y1)
                if not self.point_clears_world(tlay, mx, my, rad, tnet, clr=clr):
                    hit_here = True
                    bx = max(0, min(25, int(mx / 2.0)))
                    by = max(0, min(25, int(my / 2.0)))
                    for ddx in (-1, 0, 1):
                        for ddy in (-1, 0, 1):
                            for ti in self.track_bucket.get((bx + ddx, by + ddy), ()):
                                o_l, ox1, oy1, ox2, oy2, otw, onet = self.tracks[ti]
                                if o_l == tlay and onet not in ("GND", "+3V3", tnet):
                                    if dist_point_to_seg(mx, my, ox1, oy1, ox2, oy2) < rad + 0.5 * otw + clr:
                                        if onet in cluster:
                                            conflicting.add(onet)
                                        else:
                                            ext_colliders.add(onet)
                            for vi in self.via_bucket.get((bx + ddx, by + ddy), ()):
                                vx, vy, vd, _, vnet = self.vias[vi]
                                if vnet not in ("GND", "+3V3", tnet):
                                    if math.hypot(mx - vx, my - vy) < rad + 0.5 * vd + clr:
                                        if vnet in cluster:
                                            conflicting.add(vnet)
                                        else:
                                            ext_colliders.add(vnet)
                    if bump_hist:
                        gx, gy = to_grid(mx), to_grid(my)
                        hi = self.hist_cost[tlay]
                        for dy_h in range(-2, 3):
                            ny_h = gy + dy_h
                            if 0 <= ny_h < NY:
                                off = ny_h * NX
                                for dx_h in range(-2, 3):
                                    nx_h = gx + dx_h
                                    if 0 <= nx_h < NX:
                                        hi[off + nx_h] = min(700, hi[off + nx_h] + 75)
            if hit_here:
                conflicting.add(tnet)

        for vx, vy, vd, vdr, vnet in self.vias:
            if vnet not in cluster:
                continue
            if not self.point_clears_world(-1, vx, vy, 0.5 * vd, vnet, clr=clr, is_via=True, drill_r=0.5 * vdr, skip_via_xy=(vx, vy)):
                conflicting.add(vnet)
                bx = max(0, min(25, int(vx / 2.0)))
                by = max(0, min(25, int(vy / 2.0)))
                for ddx in (-1, 0, 1):
                    for ddy in (-1, 0, 1):
                        for ti in self.track_bucket.get((bx + ddx, by + ddy), ()):
                            o_l, ox1, oy1, ox2, oy2, otw, onet = self.tracks[ti]
                            if onet not in ("GND", "+3V3", vnet):
                                if dist_point_to_seg(vx, vy, ox1, oy1, ox2, oy2) < 0.5 * vd + 0.5 * otw + clr:
                                    if onet in cluster:
                                        conflicting.add(onet)
                                    else:
                                        ext_colliders.add(onet)
                if bump_hist:
                    gx, gy = to_grid(vx), to_grid(vy)
                    for lay in (0, 1):
                        hi = self.hist_cost[lay]
                        for dy_h in range(-4, 5):
                            ny_h = gy + dy_h
                            if 0 <= ny_h < NY:
                                off = ny_h * NX
                                for dx_h in range(-4, 5):
                                    nx_h = gx + dx_h
                                    if 0 <= nx_h < NX:
                                        hi[off + nx_h] = min(700, hi[off + nx_h] + 90)
        return conflicting, ext_colliders

    def _fanout_critical_ic_plane_pins(self):
        """Place low-profile GND/+3V3 vias for U1, U3.9, and buried B.Cu passives before signal routing."""
        v_r = 0.5 * VIA_D
        dr_r = 0.5 * VIA_DRILL
        cnt = 0
        # 1) Buried B.Cu/F.Cu +3V3/GND passives that sit inside dense signal corridors
        for pid, p in enumerate(self.pads):
            if pid in self.fanned_plane_pids or p.is_tht or p.net not in ("+3V3", "GND"):
                continue
            if (p.ref, p.pname) not in (("FB1", "1"), ("C8", "1"), ("C16", "1"), ("R36", "2"), ("R28", "2")):
                continue
            src_lay = 0 if p.on_f else 1
            pts = []
            for d in (0.95, 1.25, 1.50):
                for ang in range(0, 360, 45):
                    vx = to_mm(to_grid(p.x + d * math.cos(math.radians(ang))))
                    vy = to_mm(to_grid(p.y + d * math.sin(math.radians(ang))))
                    if self.point_clears_world(-1, vx, vy, v_r, p.net, clr=CLEARANCE_MM, is_via=True, drill_r=dr_r):
                        if self.seg_clears_world(src_lay, p.x, p.y, vx, vy, 0.22, p.net, clr=0.205):
                            sc = self.soft_cost[src_lay][to_grid(vy) * NX + to_grid(vx)]
                            pts.append((sc, d, vx, vy))
            if pts:
                pts.sort()
                _, _, vx, vy = pts[0]
                self.add_track(src_lay, p.x, p.y, vx, vy, 0.22, p.net)
                self.add_via(vx, vy, VIA_D, VIA_DRILL, p.net)
                self.fanned_plane_pids.add(pid)
                cnt += 1

        # 2) U1 inward power/GND vias + U3/U9 outward power/GND vias before signal routing
        for pid, p in enumerate(self.pads):
            if pid in self.fanned_plane_pids or p.is_tht or p.net not in ("GND", "+3V3"):
                continue
            if not (p.ref in ("U1", "U9") or (p.ref == "U3" and p.pname in ("9", "18"))):
                continue
            src_lay = 0 if p.on_f else 1
            dx_c, dy_c = p.x - p.fp_x, p.y - p.fp_y
            ux, uy = ((1.0 if dx_c > 0 else -1.0), 0.0) if abs(dx_c) > abs(dy_c) else (0.0, (1.0 if dy_c > 0 else -1.0))
            px, py = -uy, ux
            dir_order = (-1.0, 1.0) if p.ref == "U1" else (1.0,)
            w_stub = 0.18 if p.is_fine else 0.22
            dists_ic = (1.40, 1.65, 1.95, 2.30, 2.75, 3.10) if p.ref == "U1" else (1.10, 1.35, 1.65, 1.95, 2.30)
            cands = []
            for d_idx, sgn in enumerate(dir_order):
                for dist in dists_ic:
                    vx = to_mm(to_grid(p.x + sgn * ux * dist))
                    vy = to_mm(to_grid(p.y + sgn * uy * dist))
                    if self.point_clears_world(-1, vx, vy, v_r, p.net, clr=CLEARANCE_MM, is_via=True, drill_r=dr_r):
                        if self.seg_clears_world(src_lay, p.x, p.y, vx, vy, w_stub, p.net, clr=0.205):
                            sc = self.soft_cost[src_lay][to_grid(vy) * NX + to_grid(vx)]
                            cands.append((d_idx, 0, sc, dist, None, vx, vy))
                if p.ref == "U1":
                    mx = to_mm(to_grid(p.x + sgn * ux * 1.15))
                    my = to_mm(to_grid(p.y + sgn * uy * 1.15))
                    if self.seg_clears_world(src_lay, p.x, p.y, mx, my, 0.18, p.net, clr=0.205):
                        for lat in (-0.45, 0.45):
                            for dist in (1.75, 2.25, 2.85):
                                vx = to_mm(to_grid(p.x + sgn * ux * dist + px * lat))
                                vy = to_mm(to_grid(p.y + sgn * uy * dist + py * lat))
                                if self.point_clears_world(-1, vx, vy, v_r, p.net, clr=CLEARANCE_MM, is_via=True, drill_r=dr_r):
                                    if self.seg_clears_world(src_lay, mx, my, vx, vy, 0.18, p.net, clr=0.205):
                                        sc = self.soft_cost[src_lay][to_grid(vy) * NX + to_grid(vx)]
                                        cands.append((d_idx, 1, sc, math.hypot(vx - p.x, vy - p.y), (mx, my), vx, vy))
            if not cands:
                continue
            cands.sort(key=lambda t: (t[0], t[1], t[2], t[3]))
            _, _, _, _, mid, vx, vy = cands[0]
            if mid is None:
                self.add_track(src_lay, p.x, p.y, vx, vy, w_stub, p.net)
            else:
                self.add_track(src_lay, p.x, p.y, mid[0], mid[1], 0.18, p.net)
                self.add_track(src_lay, mid[0], mid[1], vx, vy, 0.18, p.net)
            self.add_via(vx, vy, VIA_D, VIA_DRILL, p.net)
            self.fanned_plane_pids.add(pid)
            cnt += 1
        print(f"Pre-fanned {cnt} critical U1/U3.9/passive GND/+3V3 vias before signal routing")

    def route_signals_and_local_power(self):
        self.fanned_plane_pids = set()
        self._fanout_critical_ic_plane_pins()
        t0 = time.time()
        nets = [n for n in sorted(self.pads_by_net.keys()) if n not in ("GND", "+3V3")]
        nets.sort(key=lambda n: (
            self.net_priority(n)[0],
            0 if n in ("I2C1_SCL", "I2C1_SDA", "SPI1_SCK", "SPI1_MISO", "SPI1_MOSI", "SD_CS", "SD_CD", "FRAM_CS", "BOOT0", "SWCLK", "SWDIO", "STAT_LED", "NRST", "USART1_RX", "USART1_TX", "ESP_EN", "MPU_INT") else 1,
            -len(self.pads_by_net[n]) if self.net_priority(n)[0] == 1 else len(self.pads_by_net[n]),
            self.net_span(n),
            n,
        ))

        failed = []
        for net in nets:
            if not self._route_single_net(net, fast=True, debug_fail=True):
                failed.append(net)
        print(f"Signals Pass 1 routed {len(nets)} nets in {time.time() - t0:.2f}s (failed: {len(failed)} -> {failed})")

        # Phase A: Fast local rip-up sweep (tight 20s cap)
        deadline_a = time.time() + 20.0
        for sweep in range(1):
            if not failed or time.time() > deadline_a:
                break
            for fnet in list(failed):
                if time.time() > deadline_a:
                    break
                snap = self.snapshot_state()
                pids = list(self.pads_by_net[fnet])
                b_scores = {}
                for _, x1, y1, x2, y2, _, tnet in self.tracks:
                    if tnet != fnet and tnet not in ("GND", "+3V3"):
                        d = min(dist_point_to_seg(self.pads[i].x, self.pads[i].y, x1, y1, x2, y2) for i in pids)
                        if d < 4.5 and (tnet not in b_scores or d < b_scores[tnet]):
                            b_scores[tnet] = d
                blockers = [n for n, _ in sorted(b_scores.items(), key=lambda x: x[1])[:4]]
                if not blockers:
                    continue
                self.remove_nets_routing({fnet} | set(blockers))

                if self._route_single_net(fnet, fast=True) and self._reroute_with_cascade(blockers, {fnet}, fast=True, allow_cascade=False, deadline=deadline_a):
                    failed.remove(fnet)
                    print(f"  Local Sweep {sweep+1}: resolved {fnet} (ripped {blockers})")
                else:
                    self.restore_state(snap)
            print(f"  Local Sweep {sweep+1} done -> remaining failed ({len(failed)}): {failed}")

        # Phase B: True Iterative Negotiated-Congestion Pathfinder on tangled cluster
        if failed:
            best_snap = self.snapshot_state()
            best_rem = set(failed)
            cluster = set(failed)
            for fn in failed:
                for bn in self._find_congestion_blockers(fn)[:4]:
                    cluster.add(bn)
            print(f"  Negotiated Pathfinder cluster ({len(cluster)} nets): {sorted(cluster)} (built @ {time.time() - t0:.1f}s)")
            deadline_b = time.time() + 180.0

            incomplete = set()
            for r_idx in range(30):
                if time.time() > deadline_b:
                    break
                if r_idx == 0:
                    reroute_now = sorted(cluster, key=lambda n: (0 if n in best_rem else 1, -len(self.pads_by_net[n]), -self.net_span(n), n))
                    self.remove_nets_routing(cluster)
                    cong_pen = 95
                    for rn in reroute_now:
                        if not self._route_single_net(rn, fast=True, allow_congestion=True, cong_pen=cong_pen):
                            if not self._route_single_net(rn, fast=True, allow_congestion=True, cong_pen=6):
                                incomplete.add(rn)
                    print(f"  Pathfinder Round 0 initialized ({len(cluster)} nets, incomplete={len(incomplete)}, elapsed={time.time() - t0:.1f}s)")
                    continue

                conflicting, ext_hits = self._audit_cluster_conflicts(cluster, bump_hist=True)
                if ext_hits:
                    cluster |= ext_hits
                    conflicting |= ext_hits
                bad = conflicting | incomplete

                if not bad:
                    failed = []
                    best_rem = set()
                    best_snap = self.snapshot_state()
                    print(f"  Pathfinder Round {r_idx}: CONVERGED 0 conflicts, ALL {len(cluster)} cluster nets routed cleanly!")
                    break

                # Greedy minimal clean-subset snapshot (never keep overlapping nets!)
                if len(bad) <= len(best_rem) + 4:
                    tmp_snap = self.snapshot_state()
                    removed_cur = set(incomplete)
                    if removed_cur:
                        self.remove_nets_routing(removed_cur)
                    while True:
                        rem_conf, _ = self._audit_cluster_conflicts(cluster - removed_cur, bump_hist=False)
                        if not rem_conf:
                            break
                        victim = max(rem_conf, key=lambda n: (len(self.pads_by_net[n]), self.net_span(n)))
                        removed_cur.add(victim)
                        self.remove_nets_routing({victim})
                    if len(removed_cur) < len(best_rem):
                        best_rem = set(removed_cur)
                        best_snap = self.snapshot_state()
                        print(f"  Pathfinder Round {r_idx}: improved clean frontier -> remaining={len(best_rem)} {sorted(best_rem)} (active conflicts={len(conflicting)}, cluster={len(cluster)}, elapsed={time.time() - t0:.1f}s)")
                    else:
                        print(f"  Pathfinder Round {r_idx}: active conflicts={len(conflicting)}, incomplete={len(incomplete)}, ext+={len(ext_hits)} (best_rem={len(best_rem)}, elapsed={time.time() - t0:.1f}s)")
                    self.restore_state(tmp_snap)
                else:
                    print(f"  Pathfinder Round {r_idx}: active conflicts={len(conflicting)}, incomplete={len(incomplete)}, ext+={len(ext_hits)} (best_rem={len(best_rem)}, elapsed={time.time() - t0:.1f}s)")

                if time.time() > deadline_b:
                    break

                reroute_set = set(bad)
                if r_idx % 3 == 0 and len(reroute_set) <= 5:
                    clean_cands = [n for n in cluster if n not in reroute_set]
                    clean_cands.sort(key=lambda n: -len(self.pads_by_net[n]))
                    reroute_set.update(clean_cands[:3])

                if r_idx % 3 == 1:
                    order = sorted(reroute_set, key=lambda n: (0 if n in incomplete else 1, -len(self.pads_by_net[n]), -self.net_span(n), n))
                elif r_idx % 3 == 2:
                    order = sorted(reroute_set, key=lambda n: (0 if n in incomplete else 1, len(self.pads_by_net[n]), self.net_span(n), n))
                else:
                    order = sorted(reroute_set, key=lambda n: (0 if n in incomplete else 1, -self.net_span(n), n))

                self.remove_nets_routing(reroute_set)
                incomplete.clear()
                cong_pen = min(420, 95 + 35 * r_idx)
                for rn in order:
                    if not self._route_single_net(rn, fast=True, allow_congestion=True, cong_pen=cong_pen):
                        if not self._route_single_net(rn, fast=True, allow_congestion=True, cong_pen=4):
                            incomplete.add(rn)
            else:
                conflicting, ext_hits = self._audit_cluster_conflicts(cluster, bump_hist=False)
                if ext_hits:
                    cluster |= ext_hits
                    conflicting |= ext_hits
                bad = conflicting | incomplete
                if not bad:
                    failed = []
                    best_rem = set()
                    best_snap = self.snapshot_state()
                    print("  Pathfinder final check: CONVERGED 0 conflicts!")

            if best_rem:
                self.restore_state(best_snap)
                failed = sorted(best_rem)
                # Final strict wide-blocker rip-up on any single remaining net (max 100s, up to 2 passes)
                deadline_c = time.time() + 100.0
                for _ in range(2):
                    if not failed or time.time() > deadline_c:
                        break
                    progressed = False
                    for fnet in list(failed):
                        if time.time() > deadline_c:
                            break
                        snap_c = self.snapshot_state()
                        self.remove_nets_routing({fnet})
                        if self._route_single_net(fnet, fast=False):
                            failed.remove(fnet)
                            progressed = True
                            print(f"  Strict Final Rip-up: resolved {fnet} directly")
                            continue
                        self.restore_state(snap_c)
                        self.remove_nets_routing({fnet})
                        blockers = self._find_congestion_blockers(fnet)[:9]
                        if not blockers:
                            self.restore_state(snap_c)
                            continue
                        self.remove_nets_routing({fnet} | set(blockers))
                        if self._route_single_net(fnet, fast=False) and self._reroute_with_cascade(blockers, {fnet}, fast=False, allow_cascade=True, deadline=deadline_c):
                            failed.remove(fnet)
                            progressed = True
                            print(f"  Strict Final Rip-up: resolved {fnet} (ripped {blockers})")
                        else:
                            self.restore_state(snap_c)
                    if not progressed:
                        break

        if failed:
            print("Detailed diagnostics for remaining unrouted nets:")
            for fnet in failed:
                snap = self.snapshot_state()
                self.remove_nets_routing({fnet})
                self._route_single_net(fnet, fast=True, debug_fail=True)
                self.restore_state(snap)

        print(f"Signals final remaining unrouted nets ({len(failed)}): {failed} (total signal time {time.time() - t0:.1f}s)")
        return failed

    def _find_free_vias(self, x0, y0, net, max_r=9.0, limit=16):
        """Find multiple nearest grid-snapped unobstructed cells to drop a new GND or +3V3 via."""
        vd, vdr = VIA_D, VIA_DRILL
        res = []
        seen_g = set()
        for r_ring in [d * 0.45 for d in range(1, int(max_r / 0.45) + 1)]:
            for ang in range(0, 360, 20):
                vx = to_mm(to_grid(x0 + r_ring * math.cos(math.radians(ang))))
                vy = to_mm(to_grid(y0 + r_ring * math.sin(math.radians(ang))))
                gx, gy = to_grid(vx), to_grid(vy)
                if (gx, gy) in seen_g:
                    continue
                seen_g.add((gx, gy))
                if self.via_occ[0][gy * NX + gx] == 0 and self.via_occ[1][gy * NX + gx] == 0:
                    if self.point_clears_world(-1, vx, vy, 0.5 * vd, net, clr=CLEARANCE_MM, is_via=True, drill_r=0.5 * vdr):
                        res.append((vx, vy))
                        if len(res) >= limit:
                            return res
        return res

    def _build_plane_goals(self, p, pid):
        net = p.net
        src_lay = 0 if p.on_f else 1
        goals = []
        via_targets = {}
        for vx, vy, _, _, vnet in self.vias:
            if vnet == net and math.hypot(vx - p.x, vy - p.y) < 14.0:
                gx, gy = to_grid(vx), to_grid(vy)
                goals.append((0, gx, gy))
                goals.append((1, gx, gy))
        for qid in self.pads_by_net[net]:
            q = self.pads[qid]
            if qid != pid and (q.is_tht or qid in self.fanned_plane_pids) and math.hypot(q.x - p.x, q.y - p.y) < 10.0:
                if q.is_tht:
                    goals.append((0, to_grid(q.x), to_grid(q.y)))
                    goals.append((1, to_grid(q.x), to_grid(q.y)))
                else:
                    goals.append((0 if q.on_f else 1, to_grid(q.x), to_grid(q.y)))
        for fv_x, fv_y in self._find_free_vias(p.x, p.y, net, max_r=9.0, limit=16):
            gx, gy = to_grid(fv_x), to_grid(fv_y)
            if (src_lay, gx, gy) not in via_targets:
                goals.append((src_lay, gx, gy))
                via_targets[(src_lay, gx, gy)] = (fv_x, fv_y)
        return goals, via_targets

    def _link_single_plane_pad(self, pid, allow_congestion=False):
        p = self.pads[pid]
        net = p.net
        w = 0.18 if p.is_fine else 0.22
        self._toggle_net(net, -1)
        ep = self.find_escape_pt(pid, w)
        if ep is None:
            ep = (0 if p.on_f else 1, to_mm(to_grid(p.x)), to_mm(to_grid(p.y)))
        goals, via_targets = self._build_plane_goals(p, pid)
        raw = self._route_corridor([(ep[0], to_grid(ep[1]), to_grid(ep[2]))], goals, allow_congestion=allow_congestion) if goals else None
        self._toggle_net(net, +1)
        return ep, via_targets, raw

    def _commit_plane_pad_link(self, pid, ep, via_targets, raw):
        p = self.pads[pid]
        net = p.net
        w = 0.18 if p.is_fine else 0.22
        # Any via along raw already connects GND/+3V3 to In1.Cu/In2.Cu -> truncate right at the first via!
        for k in range(1, len(raw)):
            if raw[k][0] != raw[0][0]:
                raw = raw[:k + 1]
                break

        end_pt = raw[-1]
        add_end_via = None
        if end_pt in via_targets:
            nvx, nvy = via_targets[end_pt]
            if not any(vnet == net and math.hypot(vx - nvx, vy - nvy) < 0.06 for vx, vy, _, _, vnet in self.vias):
                if not self.via_clears_holes(nvx, nvy, VIA_DRILL):
                    return False
                add_end_via = (nvx, nvy)

        if math.hypot(ep[1] - p.x, ep[2] - p.y) > 0.01:
            self.add_track(ep[0], p.x, p.y, ep[1], ep[2], w, net)
        if len(raw) >= 2:
            self._commit_path(raw, w, net)
        if add_end_via:
            self.add_via(add_end_via[0], add_end_via[1], VIA_D, VIA_DRILL, net)
        self.fanned_plane_pids.add(pid)
        self.pad_halo_perm_off[pid] = True
        self._set_pad_halo(pid, False)
        return True

    def route_planes_fanout_and_stitch(self):
        """Drop +3V3 first (plane only on In2.Cu), then GND vias/links (zero dangling stubs)."""
        if not hasattr(self, "fanned_plane_pids"):
            self.fanned_plane_pids = set()
        direct_vias = 0
        fallback_pads = []

        for allow_soft in (False, True):
            for net_name in ("+3V3", "GND"):
                for pid in self.pads_by_net[net_name]:
                    p = self.pads[pid]
                    if pid in self.fanned_plane_pids or p.is_tht or (p.ref == "U2" and p.pname == "39"):
                        continue
                    src_lay = 0 if p.on_f else 1
                    dirs = []
                    if abs(p.hw - p.hh) > 0.05:
                        ux, uy = (p.cos_a, p.sin_a) if p.hw > p.hh else (-p.sin_a, p.cos_a)
                        if ux * (p.x - p.fp_x) + uy * (p.y - p.fp_y) < 0:
                            ux, uy = -ux, -uy
                        dirs.append((ux, uy))
                        dirs.append((-ux, -uy))
                    rdx, rdy = p.x - p.fp_x, p.y - p.fp_y
                    rl = math.hypot(rdx, rdy)
                    if rl > 0.05:
                        dirs.append((rdx / rl, rdy / rl))
                    for ang in range(0, 360, 15):
                        ar = math.radians(ang)
                        dirs.append((math.cos(ar), math.sin(ar)))

                    vd, vdr = (PWR_VIA_D, PWR_VIA_DRILL) if p.ref in ("C27", "C28", "C29", "L1", "U6") else (VIA_D, VIA_DRILL)
                    w_stub = 0.18 if p.is_fine else 0.25
                    found = False
                    for dist in (0.75, 0.95, 1.20, 1.50, 1.85, 2.25, 2.75):
                        for ux, uy in dirs:
                            vx = to_mm(to_grid(p.x + dist * ux))
                            vy = to_mm(to_grid(p.y + dist * uy))
                            if not allow_soft and self.via_soft_cost[to_grid(vy) * NX + to_grid(vx)] > 0:
                                continue
                            if (self.point_clears_world(-1, vx, vy, 0.5 * vd, net_name, clr=CLEARANCE_MM, is_via=True, drill_r=0.5 * vdr) and
                                    self.seg_clears_world(src_lay, p.x, p.y, vx, vy, w_stub, net_name, clr=0.210)):
                                if self.add_via(vx, vy, vd, vdr, net_name):
                                    self.add_track(src_lay, p.x, p.y, vx, vy, w_stub, net_name)
                                    self.fanned_plane_pids.add(pid)
                                    self.pad_halo_perm_off[pid] = True
                                    self._set_pad_halo(pid, False)
                                    direct_vias += 1
                                    found = True
                                    break
                        if found:
                            break
                    if not found and allow_soft:
                        fallback_pads.append(pid)

        linked = 0
        trapped_pids = []
        for pid in fallback_pads:
            ep, via_targets, raw = self._link_single_plane_pad(pid, allow_congestion=False)
            if raw is not None and self._commit_plane_pad_link(pid, ep, via_targets, raw):
                linked += 1
            else:
                trapped_pids.append(pid)

        # Pathfinder negotiation for any trapped plane pad (rips blocking signal nets and re-routes them!)
        deadline_p = time.time() + 45.0
        for pid in trapped_pids:
            p = self.pads[pid]
            net = p.net
            snap = self.snapshot_state()
            snap_fanned = set(self.fanned_plane_pids)
            _, _, raw_cong = self._link_single_plane_pad(pid, allow_congestion=True)
            if raw_cong is None:
                self.restore_state(snap); self.fanned_plane_pids = snap_fanned
                print(f"  WARNING plane pad unlinked (even congestion failed): {p.ref}.{p.pname} ({net})")
                continue
            if time.time() > deadline_p:
                break
            blockers = self._find_colliding_nets(raw_cong, {"GND", "+3V3"})[:5]
            if not blockers:
                self.restore_state(snap); self.fanned_plane_pids = snap_fanned
                print(f"  WARNING plane pad unlinked (no signal blockers found): {p.ref}.{p.pname} ({net})")
                continue
            self.remove_nets_routing(set(blockers))
            ep, via_targets, raw_strict = self._link_single_plane_pad(pid, allow_congestion=False)
            if raw_strict is None or not self._commit_plane_pad_link(pid, ep, via_targets, raw_strict):
                self.restore_state(snap); self.fanned_plane_pids = snap_fanned
                print(f"  WARNING plane pad still unlinked after rip {blockers}: {p.ref}.{p.pname} ({net})")
                continue
            if self._reroute_with_cascade(blockers, set(), fast=True):
                linked += 1
                print(f"  Plane Pathfinder resolved {p.ref}.{p.pname} ({net}) by rerouting {blockers}")
            else:
                self.restore_state(snap); self.fanned_plane_pids = snap_fanned
                print(f"  WARNING plane pad reroute rollback: {p.ref}.{p.pname} ({net}) (blockers: {blockers})")

        print(f"Plane fanout: {len(self.fanned_plane_pids)} total ({direct_vias} post + {linked}/{len(fallback_pads)} A*-linked)")

        # +3V3 buck stitching vias (connected on In2.Cu AND near B.Cu L1/C28/C29 output pour/traces!)
        # Note: place +3V3 extra vias ONLY where B.Cu track/pad already connects so no via_dangling warning!
        gnd_stitch = 0
        for gx_mm in [x * 4.5 + 2.5 for x in range(11)]:
            for gy_mm in [y * 4.5 + 8.0 for y in range(10)]:
                if self.point_clears_world(-1, gx_mm, gy_mm, 0.5 * VIA_D, "GND", clr=0.25, is_via=True, drill_r=0.5 * VIA_DRILL):
                    if not any(math.hypot(gx_mm - vx, gy_mm - vy) < 1.2 for vx, vy, _, _, _ in self.vias):
                        self.add_via(gx_mm, gy_mm, VIA_D, VIA_DRILL, "GND")
                        gnd_stitch += 1
        print(f"Total geometry: {len(self.tracks)} tracks, {len(self.vias)} vias ({gnd_stitch} GND stitch)")


def materialize_to_board(board, router):
    layer_map = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
    for l_idx, x1, y1, x2, y2, w, net in router.tracks:
        t = pcbnew.PCB_TRACK(board)
        t.SetLayer(layer_map[l_idx])
        t.SetStart(pcbnew.wxPointMM(x1, y1))
        t.SetEnd(pcbnew.wxPointMM(x2, y2))
        t.SetWidth(pcbnew.FromMM(w))
        ni = board.FindNet(net)
        if ni:
            t.SetNet(ni)
        board.Add(t)

    for vx, vy, vd, vdr, net in router.vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.wxPointMM(vx, vy))
        v.SetWidth(pcbnew.FromMM(vd))
        v.SetDrill(pcbnew.FromMM(vdr))
        ni = board.FindNet(net)
        if ni:
            v.SetNet(ni)
        board.Add(v)

    pts = plane_outline_pts(margin=0.35, y_top=6.30, r_bot=2.15)
    for zname, layer, net, prio, pad_conn in [
        ("In1.Cu", pcbnew.In1_Cu, "GND", 1, pcbnew.ZONE_CONNECTION_THT_THERMAL),
        ("In2.Cu", pcbnew.In2_Cu, "+3V3", 1, pcbnew.ZONE_CONNECTION_THT_THERMAL),
        ("F.Cu",   pcbnew.F_Cu,   "GND", 0, pcbnew.ZONE_CONNECTION_THERMAL),
        ("B.Cu",   pcbnew.B_Cu,   "GND", 0, pcbnew.ZONE_CONNECTION_THERMAL),
    ]:
        z = pcbnew.ZONE(board)
        z.SetZoneName(zname)
        z.SetLayer(layer)
        ni = board.FindNet(net)
        if ni:
            z.SetNet(ni)
        z.SetPriority(prio)
        z.SetLocalClearance(pcbnew.FromMM(0.22))
        z.SetMinThickness(pcbnew.FromMM(0.20))
        z.SetPadConnection(pad_conn)
        z.SetThermalReliefGap(pcbnew.FromMM(0.28))
        z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.28))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        outline = z.Outline()
        idx = outline.NewOutline()
        for px, py in pts:
            wp = pcbnew.wxPointMM(px, py)
            outline.Append(wp.x, wp.y, idx)
        board.Add(z)


def generate_pcb():
    t0 = time.time()
    os.makedirs(VERIF_DIR, exist_ok=True)
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    board.SetLayerName(pcbnew.F_Cu, "F.Cu")
    board.SetLayerName(pcbnew.In1_Cu, "In1.Cu")
    board.SetLayerName(pcbnew.In2_Cu, "In2.Cu")
    board.SetLayerName(pcbnew.B_Cu, "B.Cu")

    ds = board.GetDesignSettings()
    ds.m_MinClearance = pcbnew.FromMM(0.20)
    ds.m_CopperEdgeClearance = pcbnew.FromMM(0.30)
    ds.m_HoleClearance = pcbnew.FromMM(0.25)
    ds.m_HoleToHoleMin = pcbnew.FromMM(0.50)
    ds.m_TrackMinWidth = pcbnew.FromMM(0.15)
    ds.m_ViasMinAnnularWidth = pcbnew.FromMM(0.125)
    ds.m_ViasMinSize = pcbnew.FromMM(0.45)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.20)
    ds.m_SilkClearance = pcbnew.FromMM(0.05)

    draw_board_outline(board)

    pad_geoms = []
    for ref, val, sl, sn, fl, fn, side, x, y, rot, mpn, desc, pnets in COMPONENTS:
        fp = pcbnew.FootprintLoad(os.path.join(FP_ROOT, f"{fl}.pretty"), fn)
        board.Add(fp)
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.SetPosition(pcbnew.wxPointMM(x, y))
        if rot:
            fp.SetOrientationDegrees(float(rot))
        if side == "B":
            fp.Flip(pcbnew.wxPointMM(x, y), False)

        for pin, net in pnets.items():
            ni = board.FindNet(net)
            if ni is None:
                ni = pcbnew.NETINFO_ITEM(board, net)
                board.Add(ni)
            for p in fp.Pads():
                if p.GetName() == pin:
                    p.SetNet(ni)

        for p in fp.Pads():
            pad_geoms.append(PadGeom(fp, p, pnets.get(p.GetName(), "")))

    clean_silkscreen_artifacts(board, pad_geoms)
    hidden_refs = arrange_silkscreen(board, pad_geoms)
    keepouts = extract_keepout_boxes(board)
    print(f"Loaded {len(board.GetFootprints())} footprints, {len(pad_geoms)} pads, {len(keepouts)} keepouts (hidden silk refs: {hidden_refs})")

    router = PathfinderRouter(board, pad_geoms, keepouts)
    router.route_signals_and_local_power()
    router.route_planes_fanout_and_stitch()
    materialize_to_board(board, router)

    pcb_path = os.path.join(OUT_DIR, "FlightDataLogger.kicad_pcb")
    board.Save(pcb_path)

    board = pcbnew.LoadBoard(pcb_path)
    print("Filling 4-layer copper zones via LoadBoard...")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(pcb_path)
    print(f"Saved filled {pcb_path} in {time.time() - t0:.2f}s")

    conn = board.GetConnectivity()
    unconn = conn.GetUnconnectedCount()
    rpt_path = os.path.join(VERIF_DIR, "drc_report.txt")
    pcbnew.WriteDRCReport(board, rpt_path, pcbnew.EDA_UNITS_MILLIMETRES, True)
    blocks = open(rpt_path).read().split("\n[")
    counts = Counter(b.split("]")[0] for b in blocks[1:])
    print(f"KiCad Unconnected Items: {unconn} | DRC breakdown: {dict(counts)}")


if __name__ == "__main__":
    generate_pcb()
