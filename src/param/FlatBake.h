#pragma once

#include "param/AtlasBake.h"
#include "param/FlatPipeline.h"

// The texture of the atlas of a flat result (see FlatPipeline.h), baked like bakeAtlasTexture does for a TriMesh: every atlas
// triangle is rasterized, each texel takes its colour from the source texture through the old texture coordinates of the
// same triangle (trilinear, mip level from the density ratio), then the chart borders are extruded into the gutter by
// options.margin texels. The patches are rasterized in parallel (their texels are disjoint).
// sourceCornerUV: 6 floats per fine face (the old uv, NaN = none); faceMaterial / textureMaterial as in bakeAtlasTexture
// (an empty faceMaterial means that every face uses the texture); `source` null = a flat colour per chart.
// The alpha channel is the occupancy: every texel a chart covers or touches (the covered texels grown by one) is 255.
// Returns resolution x resolution RGBA, row 0 = v = 0.
std::vector<unsigned char> bakeAtlasFlat(const FlatMesh& mesh, const FlatResult& result, const std::vector<float>& sourceCornerUV,
                                         const std::vector<uint8_t>& faceMaterial, int textureMaterial, const BakeSource* source,
                                         const BakeOptions& options);
