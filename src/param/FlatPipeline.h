#pragma once

#include "param/CoarsePipeline.h"
#include "param/HorizonPacker.h"

// The coarse-to-fine pipeline for scans that do not fit into a TriMesh (50-100M faces): the fine mesh stays a
// FlatMesh (about 18 bytes per face), patches are built one at a time from their face lists, lifted or
// parameterized, and only the result is kept: a UV pair per face corner (24 bytes per face) and, per patch, its
// face list and border.
//   fine mesh -> coarse stage (as in runCoarsePipeline) -> fine labels (closest coarse triangle)
//             -> face neighbours -> islands removed, regions cut into connected pieces, tiny pieces merged
//             -> per piece: sub-mesh, patch (ring cuts), lift of the coarse chart        (parallel, memory bounded)
//             -> pieces that failed: ordinary parameterization + refinement on their own sub-mesh (one at a time)
struct FlatPatch {
    std::vector<uint32_t> faces;                         // fine faces of the patch
    double area3D = 0, uvArea = 0;
    std::vector<Eigen::Vector2d> boundaryUV;             // UV of the border vertices
    std::vector<std::pair<int, int>> boundaryEdges;      // border edges as indices into boundaryUV
    ParamMethod method = ParamMethod::None;
    bool   injective = false;
    bool   needsSplit = false;
    double badAreaFraction = 0.0;
    DistortionStats distortion;
};

struct FlatStats {
    CoarseStats coarse;
    size_t pieces = 0;               // connected pieces of the fine regions
    size_t liftedPieces = 0;
    size_t fallbackPieces = 0;       // went through the ordinary pipeline
    size_t facesWithoutUV = 0;       // not parameterized (faces OpenMesh refused, unresolved patches)
    size_t facesRefused = 0;         // of those: refused by the half-edge mesh (non-manifold edges)
    size_t facesUnresolved = 0;      // of those: in patches that stayed bad after the refinement
    size_t patchesUnresolved = 0;
    double adjacencySeconds = 0, cleanSeconds = 0, liftSeconds = 0, fallbackSeconds = 0;
};

struct FlatResult {
    std::vector<float>   cornerUV;       // 6 per face: u0 v0 u1 v1 u2 v2 (corner k = vertex idx[3 f + k]); NaN = none
    std::vector<int32_t> patchOfFace;    // -1 = none
    std::vector<FlatPatch> patches;
    FlatStats stats;
};

// `fine` should fit the unit box (positions are float). faceBudget bounds the number of faces of the patches that
// are processed at the same time (memory: roughly 1 KB per face).
FlatResult runFlatPipeline(const FlatMesh& fine, const CoarseOptions& options = {}, size_t faceBudget = 4000000);

// Horizon pack of the patches; the packed UV replace result.cornerUV (in [0, 1]^2). Returns the packer's result
// (its per-input uv are those of the border vertices).
AtlasResult packFlatAtlas(FlatResult& result, const HorizonOptions& options = {});
