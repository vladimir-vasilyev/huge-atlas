#!/usr/bin/env python3
"""Writes data/textured_hemisphere/: hemisphere.obj with planar texture coordinates, a material and a PNG texture.

A test model for the texture view and for baking the atlas texture (an exact, known pattern: a colour wheel with
a grid and a few letters' worth of marks, so seams and distortion are easy to see).
"""
import math
import os
import struct
import zlib

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "..", "data", "hemisphere.obj")
out_dir = os.path.join(here, "..", "data", "textured_hemisphere")
os.makedirs(out_dir, exist_ok=True)

# ---- texture: 512 x 512, row 0 is the TOP of the picture (v = 1)
N = 512
rows = []
for y in range(N):
    row = bytearray([0])                         # PNG filter type none
    for x in range(N):
        u, v = (x + 0.5) / N, 1.0 - (y + 0.5) / N
        dx, dy = u - 0.5, v - 0.5
        ang = math.atan2(dy, dx)
        rad = math.hypot(dx, dy)
        h = (ang / (2 * math.pi)) % 1.0
        # colour wheel: hue from the angle, value from the radius
        i = int(h * 6.0)
        f = h * 6.0 - i
        p, q, t = 0.25, 0.25 + 0.75 * (1 - f), 0.25 + 0.75 * f
        r, g, b = [(1, t, p), (q, 1, p), (p, 1, t), (p, q, 1), (t, p, 1), (1, p, q)][i % 6]
        k = 1.0 - 0.6 * min(1.0, rad * 1.6)
        r, g, b = r * k + 0.1, g * k + 0.1, b * k + 0.1
        # a grid every 1/16 and thicker lines every 1/4
        gx, gy = (u * 16) % 1.0, (v * 16) % 1.0
        if min(gx, 1 - gx, gy, 1 - gy) < 0.04 or min((u * 4) % 1.0, 1 - (u * 4) % 1.0, (v * 4) % 1.0, 1 - (v * 4) % 1.0) < 0.012:
            r, g, b = r * 0.25, g * 0.25, b * 0.25
        row += bytes([int(min(1.0, r) * 255), int(min(1.0, g) * 255), int(min(1.0, b) * 255), 255])
    rows.append(bytes(row))


def chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", N, N, 8, 6, 0, 0, 0)) + \
      chunk(b"IDAT", zlib.compress(b"".join(rows), 9)) + chunk(b"IEND", b"")
with open(os.path.join(out_dir, "texture.png"), "wb") as f:
    f.write(png)

with open(os.path.join(out_dir, "hemisphere.mtl"), "w") as f:
    f.write("newmtl surface\nKd 1 1 1\nmap_Kd texture.png\n")

# ---- the mesh with planar uv: u = (x + 2) / 4, v = (y + 2) / 4 (the disk has radius 2)
verts, faces = [], []
for line in open(src):
    t = line.split()
    if t and t[0] == "v":
        verts.append(tuple(map(float, t[1:4])))
    elif t and t[0] == "f":
        faces.append(tuple(int(x.split("/")[0]) for x in t[1:4]))
with open(os.path.join(out_dir, "hemisphere.obj"), "w") as f:
    f.write("mtllib hemisphere.mtl\nusemtl surface\n")
    for x, y, z in verts:
        f.write("v %.9f %.9f %.9f\n" % (x, y, z))
    for x, y, z in verts:
        f.write("vt %.9f %.9f\n" % ((x + 2) / 4, (y + 2) / 4))
    for a, b, c in faces:
        f.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))
print("wrote", os.path.normpath(out_dir), len(verts), "vertices,", len(faces), "faces")
