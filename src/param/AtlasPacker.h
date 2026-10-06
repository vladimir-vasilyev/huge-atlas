#pragma once

#include "core/MeshTypes.h"
#include "param/PatchExtractor.h"

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

// Atlas packing: every patch is scaled so that its UV area equals its 3D area (the same texel
// density everywhere), rotated to the smallest enclosing rectangle (rotating calipers over the
// convex hull) and placed on shelves with a gutter; the whole atlas is then scaled into [0, 1]^2.

struct AtlasInput {
    const std::vector<Eigen::Vector2d>* uv = nullptr;   // per patch vertex
    double uvArea = 0.0;                                 // total UV area of the patch (any scale)
    double area3D = 0.0;                                 // total 3D area of the patch
};

struct AtlasOptions {
    double gutter = 0.01;     // gap between patches (and to the atlas border) in units of sqrt(total 3D area)
    bool   rotate = true;     // rotate patches to their minimum-area rectangle
};

struct AtlasResult {
    std::vector<std::vector<Eigen::Vector2d>> uv;       // per input, in [0, 1]^2; empty for invalid inputs
    std::vector<std::array<double, 4>> rects;           // x0, y0, x1, y1 of every placed patch, in [0, 1]^2
    double side = 0.0;                                   // atlas side before scaling to [0, 1] (3D area units)
    double scale = 1.0;                                  // factor applied to get into [0, 1]
    double efficiency = 0.0;                             // sum of 3D areas / (side^2)
};

AtlasResult packAtlas(const std::vector<AtlasInput>& inputs, const AtlasOptions& options = {});

// Convex hull (counter-clockwise, no repeated endpoint) and the angle of the minimum-area rectangle.
std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> pts);
double minAreaRectangleAngle(const std::vector<Eigen::Vector2d>& hull);   // rotate by -angle to align

// Writes `source` with texture coordinates: one vt per patch vertex, so seams get distinct vt.
// Only the faces of the given patches are written (a patch without UV is skipped).
// atlasUV[i] belongs to patches[i]. Returns the number of written faces, -1 on I/O error.
int exportAtlasOBJ(const std::string& path, const TriMesh& source, const std::vector<const Patch*>& patches,
                   const std::vector<const std::vector<Eigen::Vector2d>*>& atlasUV);
