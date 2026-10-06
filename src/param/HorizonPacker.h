#pragma once

#include "param/AtlasPacker.h"

// Horizon packing ("Tetris"), Levy et al. 2002 (LSCM), section 4, made texel exact and mip aware.
//
//  * Every chart is scaled to the same texel density (UV area ~ 3D area).
//  * The chart is oriented with its longest diameter vertical (or the best of several rotations) and
//    replaced by the region between its bottom and top horizon, measured per texel column.
//  * Charts are inserted one by one, longest first. A chart goes where the space lost between its
//    bottom horizon and the current horizon is smallest; then its top horizon becomes the new horizon.
//  * The texture is a fixed square resolution x resolution. The common density is found by bisection:
//    the largest one for which every chart still fits.
//
// Gutter. Linear (and trilinear) filtering reads neighbouring texels, and a texel of mip level m is the
// average of a 2^m x 2^m block of level-0 texels. A sample inside a chart at level m can therefore see
// texels up to mipReach(m) = ceil(1.5 * 2^m) - 1 level-0 texels away from the chart's texel set (for m = 0
// that is the 1 texel of plain bilinear filtering). The texels in that reach have to carry the chart's own
// (extruded) colour, so two charts need a gap of 2 * mipReach(L) texels for every level up to L, the
// highest mip level that will ever be sampled (with trilinear filtering: the ceiling of the largest lambda).
// Each chart is therefore grown by chartMargin(L) texels on every side before it is packed.
// Baking the texture must extrude the chart borders into the gutter by the same amount.

// Level-0 texels reached by a bilinear sample taken at mip level `level`.
int mipReach(int level);
// How far a chart region reaches into the gutter: mipReach(maxMipLevel) + extraTexels (e.g. for anisotropic filtering).
int chartMargin(int maxMipLevel, int extraTexels);
// Guaranteed gap between two charts: 2 * chartMargin.
int chartGutter(int maxMipLevel, int extraTexels);

struct HorizonOptions {
    int  resolution = 2048;       // texture side in texels; a power of two
    int  maxMipLevel = 3;         // highest mip level that has to stay free of bleeding
    int  extraTexels = 0;         // additional margin per chart side
    int  atlasBorder = 0;         // texels kept free around the atlas beyond the chart margin; 0 is safe for wrapping
    int  rotations = 1;           // 1: longest diameter vertical (the paper); N > 1: N equally spaced orientations, best wins
    bool sortByDiameter = true;   // otherwise by area (both decreasing)
    int  searchIterations = 16;   // bisection steps for the texel density
};

// One chart's per-column occupancy: columns firstColumn .. firstColumn + size - 1, rows [lo, hi) in each.
struct HorizonProfile {
    int firstColumn = 0;
    std::vector<int> lo, hi;
    int width() const { return int(lo.size()); }
};

using Segment2 = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// Profile of a region given by its boundary segments in texel coordinates, grown by `dilation` texels
// in every direction (a square window, as the filter footprint is separable). Conservative: a column
// is covered from the lowest to the highest point of the boundary in it, rounded outwards.
HorizonProfile computeProfile(const std::vector<Segment2>& boundary, int dilation);

struct Placement {
    bool ok = false;
    int  u = 0;                   // atlas column of profile column 0
    int  v = 0;                   // vertical shift: column k occupies rows [lo[k] + v, hi[k] + v)
    long long waste = 0;          // texels between the current horizon and the chart's bottom horizon
    int  top = 0;                 // highest row used + 1
};

// The best position of a profile against the horizon (indexed by atlas column): columns [xBegin, xEnd)
// are available, rows up to yLimit. Minimizes the lost space, then the top, then u.
Placement bestPlacement(const HorizonProfile& profile, const std::vector<int>& horizon, int xBegin, int xEnd, int yLimit);

AtlasResult packHorizon(const std::vector<AtlasInput>& inputs, const HorizonOptions& options = {});

// Tries one fixed density (texels per 3D unit); true if every chart fits. Exposed for tests.
bool horizonFits(const std::vector<AtlasInput>& inputs, const HorizonOptions& options, double texelsPerUnit);
