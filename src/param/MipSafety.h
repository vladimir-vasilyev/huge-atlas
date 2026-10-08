#pragma once

#include "core/MeshTypes.h"

#include <vector>

// How many mip levels of a texture are safe for a UV layout: the highest level L such that the texels a bilinear sample reads
// at levels 0..L inside one chart (an island of faces joined through edges whose uv match) are never read by a sample of
// another chart, so a mip level never mixes two charts. The same rule as the atlas packer's gutter guarantee
// (mipReach(m) = ceil(1.5 * 2^m) - 1 texels beyond the chart edge at level m), measured on an arbitrary layout.
struct MipSafetyResult {
    int  resolution = 0;
    bool wrap = true;               // texel indices are taken modulo the level size (repeat); otherwise clamped
    int  charts = 0;
    int  safeLevel = -1;            // -1: even level 0 mixes charts (bodies touch or overlap); else highest safe level
    int  maxLevel = 0;              // the last level that exists (log2 of the resolution)
    std::vector<int> conflictCharts;    // per level: charts that share a read texel with another chart (body-overlap texels not attributed)
    std::vector<long long> conflicts;   // per level 0..: level-m texels read from two charts (stops after the first bad level)
    long long outsideTexels = 0;    // chart texels outside [0, resolution)^2 (layout outside the unit square), ignored
    long long bodyOverlapTexels = 0;    // texels touched by two charts
    // Free texels between neighbouring charts (Chebyshev distance, so the number a square filter window sees): over all places
    // where two charts face each other. The gutter rule needs 2 * mipReach(L) free texels for level L (L = 0..5: 2, 4, 10, 22, 46, 94).
    long long gapPairs = 0;         // number of texel pairs facing each other
    int    gapMin = -1;             // smallest gap, -1: charts never face each other within gapLimit
    int    gapP01 = -1, gapP05 = -1, gapP50 = -1;   // percentiles over those places
    int    gapP95 = -1;
    int    levelByGapP05 = -1;      // highest level whose gutter fits into the 5th percentile of the gaps
    int    levelByGapMin = -1;      // highest level whose gutter fits into gapMin / gapP01 (-1: not even level 0)
    int    levelByGapP01 = -1;
};

// Highest mip level L (>= 0) with 2 * (ceil(1.5 * 2^L) - 1) <= gap, -1 when even level 0 does not fit.
int mipLevelForGap(int gapTexels);

// cornerUV: 6 floats per face (u, v of the corners in the order of fv_range; NaN = no uv). Faces without uv are ignored.
// `wrap` as in the struct. The cost is about 2 passes over resolution^2 per level examined.
MipSafetyResult measureMipSafety(const TriMesh& mesh, const std::vector<float>& cornerUV, int resolution, bool wrap = true,
                                 int maxLevelToTry = 13, bool stopAtFirstConflict = true, int gapLimit = 128);
