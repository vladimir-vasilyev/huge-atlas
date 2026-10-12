#pragma once

#include "param/AtlasPacker.h"

#include <algorithm>
#include <cmath>

// How many squares of the UV checker fit along the atlas so that a square is the same size on the surface everywhere (the density of
// the atlas is the same everywhere): about 1 / squares of the size of the model (the models are normalized to a unit box), and with the
// texel based packer a whole number of texels of the atlas.
inline float atlasCheckerCells(const AtlasResult& a, int squares)
{
    squares = std::max(4, squares);
    if (a.resolution > 0 && a.texelsPerUnit > 0.0) {
        const double texelsPerSquare = std::max(2.0, std::round(a.texelsPerUnit / squares));
        return float(std::max(1.0, std::floor(a.resolution / texelsPerSquare)));
    }
    if (a.side > 0.0) return float(std::max(4.0, std::round(squares * a.side)));   // shelf packer: the uv unit is `side` long in 3D
    return 32.0f;
}
