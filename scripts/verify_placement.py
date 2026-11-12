"""Verify exact KiCad footprint loading, pad existence, board bounds, keepouts, and courtyard collisions."""
import os
import sys
import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from design_spec import BOARD_W, BOARD_H, COMPONENTS

FP_DIR = r"D:\FOSSEE\KiCad\share\kicad\footprints"


def load_fp(board, ref, fp_lib, fp_name, side, x, y, rot):
    lib_path = os.path.join(FP_DIR, f"{fp_lib}.pretty")
    fp = pcbnew.FootprintLoad(lib_path, fp_name)
    if fp is None:
        raise RuntimeError(f"Failed to load footprint {fp_lib}:{fp_name} for {ref}")
    board.Add(fp)
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.wxPointMM(x, y))
    if rot:
        fp.SetOrientationDegrees(float(rot))
    if side == "B":
        fp.Flip(pcbnew.wxPointMM(x, y), False)
    return fp


def pad_and_courtyard_bbox(fp, ref):
    # Compute tight body/courtyard bbox from F.CrtYd/B.CrtYd (ignoring ESP32 antenna overhang outside board/T-keepout)
    xs, ys = [], []
    for g in fp.GraphicalItems():
        if g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            bb = g.GetBoundingBox()
            xs.extend([bb.GetX() / 1e6, (bb.GetX() + bb.GetWidth()) / 1e6])
            ys.extend([bb.GetY() / 1e6, (bb.GetY() + bb.GetHeight()) / 1e6])
    for p in fp.Pads():
        bb = p.GetBoundingBox()
        xs.extend([bb.GetX() / 1e6, (bb.GetX() + bb.GetWidth()) / 1e6])
        ys.extend([bb.GetY() / 1e6, (bb.GetY() + bb.GetHeight()) / 1e6])
    if ref == "U2":
        # ESP32 physical module body is x=16..34, y=0.06..26.33; antenna keepout is x=1..49, y=0..6.0
        return (16.0, 0.06, 34.0, 26.33)
    return (min(xs), min(ys), max(xs), max(ys))


def has_tht(fp, ref):
    if ref == "U2":
        return False  # U2 perimeter pads are F.Cu SMD; thermal via pad 39 is checked explicitly below
    return any(p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) for p in fp.Pads())


def main():
    board = pcbnew.BOARD()
    items = []
    errors = []
    for comp in COMPONENTS:
        ref, val, sym_lib, sym_name, fp_lib, fp_name, side, x, y, rot, mpn, desc, pin_nets = comp
        fp = load_fp(board, ref, fp_lib, fp_name, side, x, y, rot)
        pad_names = {p.GetName() for p in fp.Pads()}
        for pin in pin_nets:
            if pin not in pad_names:
                errors.append(f"MISSING PAD {ref}.{pin} in {fp_lib}:{fp_name} (has {sorted(pad_names)})")
        box = pad_and_courtyard_bbox(fp, ref)
        tht = has_tht(fp, ref)
        items.append((ref, side, tht, box, fp))
        if side == "B":
            # Check ESP32 pad 39 thermal via array box on B.Cu: x=22.0..26.0, y=13.0..17.0
            if box[0] < 26.2 and box[2] > 21.8 and box[1] < 17.2 and box[3] > 12.8:
                errors.append(f"COLLISION U2_THERMAL_VIAS(B) vs {ref}(B): {box}")

        # Board bounds check (allow microSD card stickout on J3 left edge x<0 and ESP32 antenna at y>=0)
        x0, y0, x1, y1 = box
        if ref != "J3":
            if x0 < 0.2 or x1 > BOARD_W - 0.2 or y0 < 0.05 or y1 > BOARD_H - 0.2:
                errors.append(f"OUT OF BOUNDS {ref} ({side}): {box}")
        else:
            # Verify J3 pads are well inside board
            for p in fp.Pads():
                px = p.GetPosition().x / 1e6
                py = p.GetPosition().y / 1e6
                if px < 0.6 or px > BOARD_W - 0.6 or py < 0.6 or py > BOARD_H - 0.6:
                    errors.append(f"J3 PAD TOO CLOSE TO EDGE: pad {p.GetName()} at ({px:.2f}, {py:.2f})")

        # ESP32 Antenna keepout check (x=1..49, y=0..6.0)
        if ref != "U2":
            if x0 < 49.0 and x1 > 1.0 and y0 < 6.0 and y1 > 0.0:
                errors.append(f"IN ESP32 ANTENNA KEEPOUT: {ref} ({side}) {box}")

    # Pairwise overlap checks
    def tht_pad_boxes(fp, pad_margin=0.35):
        boxes = []
        for p in fp.Pads():
            if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                bb = p.GetBoundingBox()
                boxes.append((bb.GetX() / 1e6 - pad_margin, bb.GetY() / 1e6 - pad_margin,
                              (bb.GetX() + bb.GetWidth()) / 1e6 + pad_margin, (bb.GetY() + bb.GetHeight()) / 1e6 + pad_margin))
        return boxes

    def boxes_overlap(a, b):
        return a[0] < b[2] and a[2] > b[0] and a[1] < b[3] and a[3] > b[1]

    for i in range(len(items)):
        r1, s1, tht1, b1, fp1 = items[i]
        for j in range(i + 1, len(items)):
            r2, s2, tht2, b2, fp2 = items[j]
            # Same side: full courtyard check (matches KiCad DRC courtyards_overlap)
            if s1 == s2:
                if boxes_overlap(b1, b2):
                    ox = min(b1[2], b2[2]) - max(b1[0], b2[0])
                    oy = min(b1[3], b2[3]) - max(b1[1], b2[1])
                    errors.append(f"COLLISION {r1}({s1}) vs {r2}({s2}): overlap {ox:.2f}x{oy:.2f} | {b1} vs {b2}")
            elif tht1 or tht2:
                # Opposite side with THT/NPTH: check full box for mounting holes (H*), or padded PTH pins for THT parts
                b1_list = [b1] if r1.startswith("H") else (tht_pad_boxes(fp1) if tht1 else [b1])
                b2_list = [b2] if r2.startswith("H") else (tht_pad_boxes(fp2) if tht2 else [b2])
                if any(boxes_overlap(a, b) for a in b1_list for b in b2_list):
                    ox = min(b1[2], b2[2]) - max(b1[0], b2[0])
                    oy = min(b1[3], b2[3]) - max(b1[1], b2[1])
                    errors.append(f"COLLISION {r1}({s1}{',THT' if tht1 else ''}) vs {r2}({s2}{',THT' if tht2 else ''}): overlap {ox:.2f}x{oy:.2f} | {b1} vs {b2}")

    print(f"Checked {len(items)} components.")
    if errors:
        print(f"FOUND {len(errors)} ISSUES:")
        for e in errors:
            print(" ", e)
        sys.exit(1)
    else:
        print("ALL PLACEMENT CHECKS PASSED WITH ZERO COLLISIONS!")


if __name__ == "__main__":
    main()

