#pragma once

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

// Occupancy mask of an atlas. The mask goes into the alpha channel of the texture: 255 on every texel
// that a chart covers, 0 in the gutter and in unused space. A texture baker uses it to know which
// texels carry content and where to extrude the chart borders (by chartMargin() texels, see HorizonPacker.h).

struct MaskChart {
    const TriMesh* mesh = nullptr;                      // the patch mesh (its triangles define the coverage)
    const std::vector<Eigen::Vector2d>* uv = nullptr;   // atlas UV per patch vertex, in [0, 1]
};

// resolution x resolution bytes, row j = texels with v in [j, j + 1) / resolution (row 0 at v = 0).
// A texel is covered if its closed square intersects a triangle (conservative: touching counts).
std::vector<unsigned char> rasterizeCoverage(const std::vector<MaskChart>& charts, int resolution);

// RGBA image of the atlas: alpha = coverage mask, RGB = a colour per chart (0 where nothing is covered).
// flipV writes image row 0 at v = 1 (the atlas looks like the UV view); otherwise row 0 is v = 0.
bool writeAtlasPNG(const std::string& path, const std::vector<MaskChart>& charts, int resolution, bool flipV = true,
                   std::string* error = nullptr);

// Plain 8-bit RGBA PNG writer (zlib).
bool writePNG(const std::string& path, int width, int height, const std::vector<unsigned char>& rgba,
              std::string* error = nullptr);
