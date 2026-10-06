#pragma once

#include "core/MeshTypes.h"

#include <vector>

// Quality of a UV layout of a whole mesh, given as texture coordinates of the corners of every face
// (6 floats per face: u, v of the corners in the order of fv_range; NaN = the face has no uv). The same
// measures apply to the layout stored in a file and to the atlas that this program makes, so the two can
// be compared.
//
//  * distortion is measured after scaling the uv so that its total area equals the 3D area (Sander et al.):
//    L2 stretch sqrt((s1^2 + s2^2) / 2) (1 = isometric), conformal s1 / s2 (1 = angle preserving);
//  * density = s1 * s2, the texel density of a face relative to the average (1 = the same as on average);
//    its spread says how much texture is wasted on some faces and starved on others;
//  * charts are the islands of faces connected through edges whose uv match at both ends; the seams are the
//    other interior edges (every seam is a place where bilinear filtering and mip mapping need a gutter);
//  * covered / overlap are counted on a resolution x resolution texture by the texel centers.
// Percentiles are weighted by the 3D area of the faces.

struct UVLayoutStats {
    size_t faces = 0;
    size_t facesWithoutUV = 0;
    int    flipped = 0;              // faces whose uv orientation differs from that of most of the area (or has no area)
    bool   mirrored = false;         // the layout as a whole is mirrored (negative total uv area): not an error, many exporters do it

    int    charts = 0;
    double seamLength = 0.0;         // 3D length of the seams
    double edgeLength = 0.0;         // 3D length of all interior edges
    double seamRatio() const { return edgeLength > 0.0 ? seamLength / edgeLength : 0.0; }

    double l2Mean = 0.0, l2P50 = 0.0, l2P95 = 0.0, l2P99 = 0.0, l2Max = 0.0;
    double conformalP50 = 0.0, conformalP95 = 0.0;
    double densityP05 = 0.0, densityP50 = 0.0, densityP95 = 0.0;

    int    resolution = 0;
    double covered = 0.0;            // fraction of the texels whose center lies in a triangle
    double overlap = 0.0;            // fraction of the covered texels that lie in two or more triangles
};

UVLayoutStats analyzeUVLayout(const TriMesh& mesh, const std::vector<float>& cornerUV, int resolution);
