#!/usr/bin/env python3
"""Create the training arena with the target-lane platform removed."""

import json
import shutil
import struct
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[1]
SOURCE = SIM_ROOT / "simulator/assets/GROUND.glb"
OUTPUT = SIM_ROOT / "simulator/assets/GROUND_TRAINING.glb"
X_RANGE = (-9.6, -6.4)
Z_RANGE = (-1.6, 1.6)
PLATFORM_MIN_Y = -0.16


def chunks(data):
    offset = 12
    while offset < len(data):
        size, kind = struct.unpack_from("<II", data, offset)
        yield kind, offset + 8, size
        offset += 8 + size


def main():
    shutil.copyfile(SOURCE, OUTPUT)
    data = bytearray(OUTPUT.read_bytes())
    document = None
    binary_offset = None
    for kind, offset, size in chunks(data):
        if kind == 0x4E4F534A:
            document = json.loads(data[offset : offset + size].rstrip(b"\0 "))
        elif kind == 0x004E4942:
            binary_offset = offset
    if document is None or binary_offset is None:
        raise RuntimeError("GROUND.glb has no JSON or BIN chunk")

    component = {5121: ("B", 1), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
    dimensions = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}

    def accessor(index):
        item = document["accessors"][index]
        view = document["bufferViews"][item["bufferView"]]
        fmt, width = component[item["componentType"]]
        count = dimensions[item["type"]]
        stride = view.get("byteStride", width * count)
        start = binary_offset + view.get("byteOffset", 0) + item.get("byteOffset", 0)
        return item, fmt, count, stride, start

    removed = 0
    for node in document["nodes"]:
        if node.get("name") not in {"GROUND_SPLAT", "GROUND_DENSE"}:
            continue
        translation = node.get("translation", [0.0, 0.0, 0.0])
        for primitive in document["meshes"][node["mesh"]]["primitives"]:
            _, pfmt, pcount, pstride, pstart = accessor(primitive["attributes"]["POSITION"])
            index_item, ifmt, _, istride, istart = accessor(primitive["indices"])

            def position(vertex):
                values = struct.unpack_from("<" + pfmt * pcount, data, pstart + vertex * pstride)
                return tuple(values[i] + translation[i] for i in range(3))

            indices = [
                struct.unpack_from("<" + ifmt, data, istart + i * istride)[0]
                for i in range(index_item["count"])
            ]
            for tri in range(0, len(indices), 3):
                vertices = [position(indices[tri + i]) for i in range(3)]
                cx = sum(v[0] for v in vertices) / 3.0
                cz = sum(v[2] for v in vertices) / 3.0
                if (X_RANGE[0] < cx < X_RANGE[1] and Z_RANGE[0] < cz < Z_RANGE[1]
                        and max(v[1] for v in vertices) > PLATFORM_MIN_Y):
                    for i in (1, 2):
                        struct.pack_into("<" + ifmt, data, istart + (tri + i) * istride, indices[tri])
                    removed += 1

    OUTPUT.write_bytes(data)
    print(f"wrote {OUTPUT} with {removed} platform triangles removed")


if __name__ == "__main__":
    main()
