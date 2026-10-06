#pragma once

#include "core/MeshTypes.h"
#include "param/PatchExtractor.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

// Bakes the texture of the atlas: every atlas triangle is rasterized into the new texture and each
// texel takes its colour from the source texture at the point that the texel represents (the source
// texture coordinates of the same triangle, interpolated). The source is filtered with a trilinear
// lookup in a mip chain chosen from the density ratio of the triangle, so a smaller atlas does not alias.
// The chart borders are then extruded into the gutter by `margin` texels (see HorizonPacker.h): the
// texels around a chart carry the colour of the nearest texel of the same chart, which is what keeps
// bilinear / mip mapped lookups clean at the chart borders.
// The alpha channel is the occupancy mask: 255 on every texel a chart touches, 0 elsewhere.

struct BakeChart {
    const Patch* patch = nullptr;                       // faceToGlobal / vertexToGlobal map to the source mesh
    const std::vector<Eigen::Vector2d>* uv = nullptr;   // atlas UV per patch vertex, in [0, 1]
};

struct BakeSource {
    int width = 0, height = 0;
    const unsigned char* rgba = nullptr;                // row 0 = v = 0 (bottom), as loadImageRGBA returns it
};

struct BakeOptions {
    int resolution = 2048;                              // the atlas texture is resolution x resolution
    int margin = 11;                                    // extrusion distance in texels (chartMargin of the packer)
    unsigned char background[3] = {128, 128, 128};      // colour of unoccupied texels and of faces without a texture
};

// `source` may be null: every chart then gets a flat colour of its own (a mask you can paint over).
// cornerUV: 6 floats per face of `sourceMesh` (u, v of the corners in the order of fv_range), empty if none.
// faceMaterial / textureMaterial select the faces that use the source texture (other faces get the background);
// an empty faceMaterial means that every face does.
// Returns resolution x resolution RGBA, row 0 = v = 0.
std::vector<unsigned char> bakeAtlasTexture(const TriMesh& sourceMesh, const std::vector<BakeChart>& charts,
                                            const BakeSource* source, const std::vector<float>& cornerUV,
                                            const std::vector<int>& faceMaterial, int textureMaterial,
                                            const BakeOptions& options);

// The atlas as texture coordinates of the corners of the faces of the source mesh (6 floats per face, the order
// of fv_range); NaN for the faces that no chart has. The input of analyzeUVLayout.
std::vector<float> cornerUVFromAtlas(const TriMesh& sourceMesh, const std::vector<BakeChart>& charts);

// How well a new texture reproduces the old one: both are looked up at the same points of the surface
// (three points inside every face, faces with `faceStride` apart) through the old and the new texture
// coordinates, and the colours are compared (RGB, 0..255).
//   rmse     the old texture filtered to the resolution of the new one (what an ideal bake gives)
//   rmseFull the old texture at full resolution (this also counts the detail that the smaller texture cannot hold)
struct RetextureReport {
    size_t samples = 0;
    double rmse = 0.0, psnr = 0.0;
    double rmseFull = 0.0, psnrFull = 0.0;
};

RetextureReport measureRetexture(const TriMesh& mesh, const std::vector<float>& oldCornerUV, const BakeSource& oldTexture,
                                 const std::vector<int>& faceMaterial, int textureMaterial,
                                 const std::vector<float>& newCornerUV, const std::vector<unsigned char>& newRgba,
                                 int newResolution, size_t faceStride = 1);
