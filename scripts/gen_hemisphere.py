#!/usr/bin/env python3
"""Writes data/hemisphere.obj: a hemisphere (radius 1) sitting on a flat disk (radius 2).

Test model for the flooding partition (VSA paper, fig. 7).
"""
import math
import os

SEGMENTS = 48    # around the axis
DOME_RINGS = 12  # pole -> equator
FLAT_RINGS = 6   # equator -> outer rim

verts = [(0.0, 0.0, 1.0)]
rings = []
for j in range(1, DOME_RINGS + 1):
    th = 0.5 * math.pi * j / DOME_RINGS
    rings.append([(math.sin(th) * math.cos(2 * math.pi * i / SEGMENTS),
                   math.sin(th) * math.sin(2 * math.pi * i / SEGMENTS),
                   math.cos(th) if j < DOME_RINGS else 0.0) for i in range(SEGMENTS)])
for j in range(1, FLAT_RINGS + 1):
    r = 1.0 + j / FLAT_RINGS
    rings.append([(r * math.cos(2 * math.pi * i / SEGMENTS),
                   r * math.sin(2 * math.pi * i / SEGMENTS), 0.0) for i in range(SEGMENTS)])

index = []
for ring in rings:
    index.append(list(range(len(verts) + 1, len(verts) + 1 + SEGMENTS)))
    verts.extend(ring)

faces = []
for i in range(SEGMENTS):
    faces.append((1, index[0][i], index[0][(i + 1) % SEGMENTS]))
for a, b in zip(index, index[1:]):
    for i in range(SEGMENTS):
        n = (i + 1) % SEGMENTS
        faces.append((a[i], b[i], b[n]))
        faces.append((a[i], b[n], a[n]))

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "hemisphere.obj")
with open(out, "w") as f:
    for v in verts:
        f.write("v %.9f %.9f %.9f\n" % v)
    for t in faces:
        f.write("f %d %d %d\n" % t)
print("wrote", os.path.normpath(out), len(verts), "vertices,", len(faces), "faces")
