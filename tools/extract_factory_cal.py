#!/usr/bin/env python3
"""extract_factory_cal.py -- pull the factory per-unit calibration values out of
a stock-OtO flash backup and (for the valve) suggest the irrigoto frame offset.

The stock OtO firmware stores per-unit encoder home references in NVS under the
"OtO" namespace:

    valve_home   (uint16)  -- valve  AS5600L mounting reference
    nozzle_home  (uint16)  -- nozzle AS5600  mounting reference

irrigoto erases the chip on first flash, so these survive ONLY in the factory
backup image (oto_factory_<MAC>.bin) made before flashing. This tool reads them
straight from that .bin -- no device, no esp-idf, no network -- so a unit can be
calibrated water-free from its backup.

The valve frame differs between units by a constant offset (the encoder magnet's
mounting rotation). Anchoring measured pressure-peak angles against valve_home
gives a linear decode (see VALVE_ANCHORS). The tool prints the predicted peak,
the resulting offset, and the ready-to-run /cal/valve/set command.

    python tools/extract_factory_cal.py path/to/oto_factory_<MAC>.bin
    python tools/extract_factory_cal.py *.bin --device 192.168.1.108   # prints curl

Only the two uint16 home values are read; wifi/account strings in the image are
never touched.
"""
import argparse
import struct
import sys

PAGE_SIZE = 4096
ENTRY_SIZE = 32
ENTRIES_PER_PAGE = 126
NVS_TYPE_U16 = 0x02
PART_TABLE_OFFSET = 0x8000
PART_MAGIC = b"\xAA\x50"

# --- valve frame decode -------------------------------------------------------
# Reference unit's VALVE_PEAK_DEG in components/irrigoto/irrigoto.c (offset 0).
REF_PEAK_DEG = 306.7
# (valve_home, measured_pressure_peak_deg) anchors. APPEND a row each time you
# calibrate a new unit with POST /cal/valve so the fit tightens. Currently a
# 2-point fit -> treat the predicted offset as a starting point, then confirm
# with a settled /valve/probe sweep.
VALVE_ANCHORS = [
    (3777, 306.7),   # ba1f88 (reference; peak = hardcoded VALVE_PEAK_DEG)
    (17022, 256.0),  # f9e994 (measured settled peak)
]


def find_nvs_partition(image):
    """Return (offset, size, label) of the first data/nvs partition."""
    pt = image[PART_TABLE_OFFSET:PART_TABLE_OFFSET + 0xC00]
    off = 0
    while off + 32 <= len(pt):
        e = pt[off:off + 32]
        if e[0:2] != PART_MAGIC:
            break
        ptype, subtype = e[2], e[3]
        p_off, p_size = struct.unpack("<II", e[4:12])
        if ptype == 0x01 and subtype == 0x02:  # data / nvs
            label = e[12:28].split(b"\x00")[0].decode("ascii", "replace")
            return p_off, p_size, label
        off += 32
    return 0x9000, 0x40000, "nvs(default)"


def parse_nvs_u16(nvs):
    """Return {key: value} for every WRITTEN uint16 entry across all pages."""
    out = {}
    for pi in range(len(nvs) // PAGE_SIZE):
        page = nvs[pi * PAGE_SIZE:(pi + 1) * PAGE_SIZE]
        if struct.unpack("<I", page[0:4])[0] == 0xFFFFFFFF:
            continue  # uninitialized page
        bitmap = page[32:64]
        for ei in range(ENTRIES_PER_PAGE):
            state = (bitmap[ei // 4] >> ((ei % 4) * 2)) & 0x3
            if state != 0x2:  # 0x2 = Written (3 = empty, 0 = erased)
                continue
            e = page[64 + ei * ENTRY_SIZE:64 + (ei + 1) * ENTRY_SIZE]
            if e[1] != NVS_TYPE_U16:
                continue
            key = e[8:24].split(b"\x00")[0].decode("ascii", "replace")
            out[key] = struct.unpack("<H", e[24:26])[0]
    return out


def fit_line(anchors):
    """Least-squares peak_deg = a*home + b."""
    n = len(anchors)
    sx = sum(h for h, _ in anchors)
    sy = sum(p for _, p in anchors)
    sxx = sum(h * h for h, _ in anchors)
    sxy = sum(h * p for h, p in anchors)
    denom = n * sxx - sx * sx
    if denom == 0:
        raise ValueError("degenerate anchors (need >=2 distinct valve_home values)")
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    return a, b


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", help="factory backup .bin (full 8 MB flash dump)")
    ap.add_argument("--device", help="device host/IP -> also print the curl to push the offset")
    args = ap.parse_args()

    try:
        image = open(args.image, "rb").read()
    except OSError as ex:
        sys.exit(f"cannot read {args.image}: {ex}")
    if len(image) < PART_TABLE_OFFSET + 0xC00:
        sys.exit(f"{args.image}: too small ({len(image)} bytes) to be a flash dump")

    off, size, label = find_nvs_partition(image)
    nvs = image[off:off + size]
    kv = parse_nvs_u16(nvs)

    valve_home = kv.get("valve_home")
    nozzle_home = kv.get("nozzle_home")

    print(f"image      : {args.image} ({len(image)} bytes)")
    print(f"nvs part   : '{label}' @ {off:#x} ({size // 1024} KB)")
    print(f"valve_home : {valve_home}")
    print(f"nozzle_home: {nozzle_home}")

    if valve_home is None:
        sys.exit("\nvalve_home not found -- is this a stock-OtO factory image?")

    a, b = fit_line(VALVE_ANCHORS)
    peak = a * valve_home + b
    offset = peak - REF_PEAK_DEG
    print()
    print(f"valve decode ({len(VALVE_ANCHORS)}-pt fit: peak = {a:.6f}*home + {b:.3f}):")
    print(f"  predicted peak  : {peak:.2f} deg")
    print(f"  suggested offset: {offset:+.2f} deg   (closed {231.0 + offset:.1f}, open {308.0 + offset:.1f})")
    if not (-60.0 < offset < 60.0):
        print("  WARNING: offset outside +/-60 deg -- fit may not cover this unit; verify by probe.")
    print("  NOTE: 2-point fit -- confirm with a settled /valve/probe sweep before trusting it.")

    push = f"curl -X POST 'http://{args.device or '<device>'}/cal/valve/set?offset={offset:.1f}'"
    print(f"\npush: {push}")

    # nozzle: home captured, but decoding needs a measured nozzle reference we
    # don't have yet (no nozzle aim cal anchor). Report raw for now.
    if nozzle_home is not None:
        print(f"\nnozzle_home={nozzle_home} captured -- decode pending a measured nozzle anchor.")


if __name__ == "__main__":
    main()
