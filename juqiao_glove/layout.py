"""Juqiao tactile-glove sensor layout: index mappings + approximate physical
positions, derived from the vendor spec sheet (Weihai JQ-Industries
JQGY-YL-11 / JQGY-YL-31, V1.1 2026-03-23, pages 9-13).

Coordinate convention (per-glove local frame):
    origin:  center of wrist
    +Y:      wrist  ->  fingertips
    +Z:      out of palm (away from back-of-hand)
    +X:      thumb side, in both LEFT and RIGHT frames

So for a LEFT hand viewed palm-up with the wrist at the bottom, the thumb is
to the viewer's right (+X side). For a RIGHT hand viewed palm-up with the
wrist at the bottom, the thumb is also at +X in its own local frame --
each glove uses its own local frame, mirroring is handled at TF level.

All positions are in millimetres in this module; callers multiply by 1e-3
to get metres for ROS messages.

Spec recap (relevant constants only):
    256 logical slots in a 16x16 grid, of which 162 are active sensors
    per glove (94 are unused padding slots that stay 0).
    Each sensor reports uint8 (0..255), corresponding to ~0..350 N.
    Wire protocol: 921600 baud, 100 Hz per packet type.
    Two interleaved packets per sample:
        packet 0x01 (134 B): sync(4) + 0x01 + type + 128 bytes (slots   1..128)
        packet 0x02 (150 B): sync(4) + 0x02 + type + 128 bytes (slots 129..256) + 16-byte quaternion
    Sensor type byte: 0x01=LH, 0x02=RH, 0x03=LF, 0x04=RF, 0x05=WB.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple

# ---- Wire-protocol constants ---------------------------------------------

SYNC = b"\xaa\x55\x03\x99"
PACKET_LEN_FIRST  = 134   # 0x01 packet: 6 header + 128 sensor bytes
PACKET_LEN_SECOND = 150   # 0x02 packet: 6 header + 128 sensor bytes + 16 quat
PRESSURE_BYTES_PER_PACKET = 128
QUAT_BYTES = 16
TOTAL_SENSOR_SLOTS = 256
PACKET_RATE_HZ = 100
BAUD = 921600
FORCE_FULL_SCALE_N = 350.0   # uint8 0..255 maps to 0..350 N per spec page 9

SIDE_HEX = {"left": 0x01, "right": 0x02}
HEX_SIDE = {v: k for k, v in SIDE_HEX.items()}

# ---- Sensor index maps (verbatim from spec pages 11-13) -------------------
#
# All indices below are 1-indexed as printed in the spec sheet. Convert to
# 0-indexed before using as Python array offsets (or use `to_zero_indexed()`).
# Each per-finger pressure list is 12 sensors in row-major order:
#     row 0 = fingertip, ... row 3 = base; col 0 = left, col 2 = right
#     (where "left/right" follows the spec's "左->右" labelling per page 11)

LEFT_REGIONS: Dict[str, List[int]] = {
    "thumb_pressure":  [19, 18, 17,  3,  2,  1, 243, 242, 241, 227, 226, 225],
    "index_pressure":  [22, 21, 20,  6,  5,  4, 246, 245, 244, 230, 229, 228],
    "middle_pressure": [25, 24, 23,  9,  8,  7, 249, 248, 247, 233, 232, 231],
    "ring_pressure":   [28, 27, 26, 12, 11, 10, 252, 251, 250, 236, 235, 234],
    "pinky_pressure":  [31, 30, 29, 15, 14, 13, 255, 254, 253, 239, 238, 237],
    "thumb_bend":  [210],
    "index_bend":  [213],
    "middle_bend": [216],
    "ring_bend":   [219],
    "pinky_bend":  [222],
    # Palm: 5 rows, top-to-bottom; first row is 12 cells (gap on thumb side),
    # rows 2-5 are 15 cells. See spec page 11 "手掌(左->右; 上->下)".
    "palm": (
        list(range(207, 195, -1)) +   # row 1: 207..196  (12)
        list(range(191, 176, -1)) +   # row 2: 191..177  (15)
        list(range(175, 160, -1)) +   # row 3: 175..161  (15)
        list(range(159, 144, -1)) +   # row 4: 159..145  (15)
        list(range(143, 128, -1))     # row 5: 143..129  (15)
    ),
}

RIGHT_REGIONS: Dict[str, List[int]] = {
    "thumb_pressure":  [240, 239, 238, 256, 255, 254, 16, 15, 14, 32, 31, 30],
    "index_pressure":  [237, 236, 235, 253, 252, 251, 13, 12, 11, 29, 28, 27],
    "middle_pressure": [234, 233, 232, 250, 249, 248, 10,  9,  8, 26, 25, 24],
    "ring_pressure":   [231, 230, 229, 247, 246, 245,  7,  6,  5, 23, 22, 21],
    "pinky_pressure":  [228, 227, 226, 244, 243, 242,  4,  3,  2, 20, 19, 18],
    "thumb_bend":  [47],
    "index_bend":  [44],
    "middle_bend": [41],
    "ring_bend":   [38],
    "pinky_bend":  [35],
    "palm": (
        list(range( 61,  49, -1)) +   # row 1:  61..50   (12)
        list(range( 80,  65, -1)) +   # row 2:  80..66   (15)
        list(range( 96,  81, -1)) +   # row 3:  96..82   (15)
        list(range(112,  97, -1)) +   # row 4: 112..98   (15)
        list(range(128, 113, -1))     # row 5: 128..114  (15)
    ),
}

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]


def get_regions(side: str) -> Dict[str, List[int]]:
    side = side.lower()
    if side == "left":
        return LEFT_REGIONS
    if side == "right":
        return RIGHT_REGIONS
    raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def all_named_indices(side: str) -> List[int]:
    """All 1-indexed sensor IDs that the spec gives a name to (137 of 162)."""
    seen: List[int] = []
    for ids in get_regions(side).values():
        seen.extend(ids)
    return sorted(set(seen))


def to_zero_indexed(ids: List[int]) -> List[int]:
    return [i - 1 for i in ids]


# ---- Physical positions (approximate, in millimetres) --------------------
#
# These are NOT exact mm-perfect positions -- the vendor's diagrams aren't
# dimensioned to that level. They are anatomically reasonable positions
# that respect the spec's "4mm x 6mm sensor resolution" and the per-finger
# 4-row x 3-col grid layout. Good enough for an RViz heatmap; tighten later
# if needed.

# Per-finger geometry constants. The vendor's "4mm x 6mm sensor resolution"
# describes individual pad size, not pitch between addressable cells -- with
# only 4 named rows per finger, sensors are spread across the full ~42mm
# pressure region of the finger, not packed at the tip.
_FINGER_COL_PITCH_MM = 4.5     # ~12mm wide column span (fits ~15mm finger)
_FINGER_ROW_PITCH_MM = 13.0    # 4 rows over 39mm of finger length
# Palm grid sized so 15-cell rows fit inside an 80mm-wide palm silhouette.
_PALM_COL_PITCH_MM   = 4.8     # 15 cells -> ~67mm wide
_PALM_ROW_PITCH_MM   = 11.0    # 5 rows -> ~44mm tall

# Finger center-line X offsets (mm). Thumb sits at largest +X (radial side).
# Index/middle/ring/pinky centers are 18mm apart, fitting 15mm-wide fingers
# with 3mm visual gap between adjacent fingers.
_FINGER_CENTER_X: Dict[str, float] = {
    "thumb":  53.0,
    "index":  27.0,
    "middle":  9.0,
    "ring":   -9.0,
    "pinky": -27.0,
}

# Finger tip Y coordinates (mm above wrist origin). Anatomically realistic
# proportions: middle > index ~ ring > pinky > thumb.
_FINGER_TIP_Y: Dict[str, float] = {
    "thumb":  102.0,
    "index":  148.0,
    "middle": 162.0,
    "ring":   154.0,
    "pinky":  138.0,
}

# Bend sensor sits at the metacarpophalangeal (MCP) knuckle. All fingers'
# knuckles align around y=85; thumb's CMC joint sits lower.
_FINGER_BEND_Y: Dict[str, float] = {
    "thumb":  35.0,
    "index":  82.0,
    "middle": 82.0,
    "ring":   82.0,
    "pinky":  82.0,
}

# Palm: top row sits just below finger bases at y=72; rows go down toward wrist.
_PALM_TOP_Y_MM = 72.0


def _finger_pressure_positions(
    indices_1based: List[int], finger_name: str, side: str
) -> Dict[int, Tuple[float, float, float]]:
    """Place 12 sensors of a finger in a 4-row x 3-col grid centered on the
    finger's longitudinal axis, with row 0 at the fingertip.

    Per spec, each per-finger table reads "left -> right; top -> bottom" from
    the diagram viewer's perspective. The spec image of the LEFT glove shows
    palm facing camera with thumb on viewer's RIGHT; the RIGHT glove image
    is mirrored (thumb on viewer's LEFT). In each glove's own local frame we
    define +X = thumb side, so:
      - LEFT glove: spec col 0 (image-left) = pinky side  = -X
      - RIGHT glove: spec col 0 (image-left) = thumb side = +X
    """
    out: Dict[int, Tuple[float, float, float]] = {}
    cx = _FINGER_CENTER_X[finger_name]
    tip_y = _FINGER_TIP_Y[finger_name]
    # col_sign = +1 means spec-col-0 sits at MAX +X (right-glove behaviour).
    # col_sign = -1 means spec-col-0 sits at MIN -X (left-glove behaviour).
    col_sign = +1 if side == "right" else -1
    for row in range(4):
        for col in range(3):
            slot = row * 3 + col
            idx_1based = indices_1based[slot]
            x = cx + col_sign * (1 - col) * _FINGER_COL_PITCH_MM
            y = tip_y - row * _FINGER_ROW_PITCH_MM
            out[idx_1based - 1] = (x, y, 0.0)
    return out


def _finger_bend_position(
    bend_idx_1based: int, finger_name: str
) -> Tuple[int, Tuple[float, float, float]]:
    cx = _FINGER_CENTER_X[finger_name]
    y = _FINGER_BEND_Y[finger_name]
    return bend_idx_1based - 1, (cx, y, 0.0)


def _palm_positions(
    palm_indices_1based: List[int], side: str
) -> Dict[int, Tuple[float, float, float]]:
    """Lay out the palm in 5 rows. Row 1 has 12 cells (gap on thumb side),
    rows 2-5 have 15 cells.

    Spec convention "left -> right" matches viewer-of-image perspective, so
    we apply the same per-side mirror as the finger pressure grids:
      - LEFT glove: spec col 0 = -X (pinky side)
      - RIGHT glove: spec col 0 = +X (thumb side)
    The 12-cell first row has its gap on the thumb side, so its X range is
    shifted away from +X."""
    out: Dict[int, Tuple[float, float, float]] = {}
    row_lengths = [12, 15, 15, 15, 15]
    cursor = 0
    col_sign = +1 if side == "right" else -1
    for r, n in enumerate(row_lengths):
        y = _PALM_TOP_Y_MM - r * _PALM_ROW_PITCH_MM
        row_indices = palm_indices_1based[cursor:cursor + n]
        cursor += n
        # Index range so col_sign==-1 puts spec-col-0 at most negative X,
        # and col_sign==+1 puts spec-col-0 at most positive X.
        # First-row gap (n==12) sits on thumb side regardless.
        center_col = (n - 1) / 2.0  # column index that maps to x=0
        if n == 12:
            # Shift the 12-cell row away from the thumb side. Cell 0 lives at
            # what would be column 0 of a 15-wide row; we want it on the
            # *pinky* end of the 15-slot range, so offset accordingly.
            cell_offset = 3 if side == "right" else 0
            center_col = (15 - 1) / 2.0
            for col, idx_1based in enumerate(row_indices):
                eff_col = col + cell_offset
                x = col_sign * (center_col - eff_col) * _PALM_COL_PITCH_MM
                out[idx_1based - 1] = (x, y, 0.0)
        else:
            for col, idx_1based in enumerate(row_indices):
                x = col_sign * (center_col - col) * _PALM_COL_PITCH_MM
                out[idx_1based - 1] = (x, y, 0.0)
    return out


def get_positions(side: str) -> Dict[int, Tuple[float, float, float]]:
    """Return {sensor_index_0based: (x_mm, y_mm, z_mm)} for all named sensors.
    +X is mirrored between left and right gloves: the LEFT-glove dictionary
    has its +X axis pointing toward the LEFT-hand thumb (which is on the
    viewer's right when the glove is held palm-up), and likewise RIGHT-glove
    +X points toward the RIGHT-hand thumb. Visualization callers should set
    up TF frames so each glove's +X points to its own anatomical thumb side."""
    regions = get_regions(side)
    positions: Dict[int, Tuple[float, float, float]] = {}
    for finger in FINGER_ORDER:
        positions.update(
            _finger_pressure_positions(regions[f"{finger}_pressure"], finger, side)
        )
        idx0, pos = _finger_bend_position(regions[f"{finger}_bend"][0], finger)
        positions[idx0] = pos
    positions.update(_palm_positions(regions["palm"], side))
    return positions


# ---- Hand silhouette (LINE_STRIP outline points, mm) ---------------------
#
# Coarse outline tracing the perimeter of a palm-up hand. Same shape works
# for both gloves in each glove's own local frame (mirrored at TF level).
# Points roughly trace: wrist-left -> palm-side -> pinky-tip -> ring-tip ->
# middle-tip -> index-tip -> thumb-tip -> thumb-base -> wrist-right -> close.

HAND_SILHOUETTE_MM: List[Tuple[float, float]] = [
    # Wrist, thumb side (+X)
    (35.0, -15.0),
    (40.0,   5.0),
    # Thenar (thumb base mound)
    (42.0,  25.0),
    (54.0,  35.0),
    # Thumb side
    (64.0,  55.0),
    (64.0,  85.0),
    (58.0, 100.0),
    # Thumb tip
    (48.0, 108.0),
    (40.0, 105.0),
    (36.0,  92.0),
    # Web between thumb and index, back into palm edge
    (34.0,  82.0),
    # Index finger right edge
    (34.0,  85.0),
    (34.0, 152.0),
    # Index tip
    (20.0, 152.0),
    # Index left edge
    (20.0,  85.0),
    # Gap between index and middle
    (18.0,  85.0),
    # Middle finger
    (18.0, 166.0),
    ( 0.0, 166.0),
    ( 0.0,  85.0),
    # Gap
    (-2.0,  85.0),
    # Ring finger
    (-2.0, 158.0),
    (-16.0, 158.0),
    (-16.0, 85.0),
    # Gap
    (-18.0, 85.0),
    # Pinky
    (-18.0, 142.0),
    (-34.0, 142.0),
    (-34.0,  82.0),
    # Pinky-side palm down to wrist
    (-40.0,  70.0),
    (-42.0,  40.0),
    (-40.0,  10.0),
    (-35.0, -15.0),
    # Close back to start
    (35.0, -15.0),
]
