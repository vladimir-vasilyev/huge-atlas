#pragma once

#include "core/Partition.h"

#include <functional>
#include <string>
#include "param/LiftParam.h"
#include "param/MeshSimplify.h"
#include "param/Refinement.h"

// The pipeline for very dense scans: simplify, segment and parameterize the coarse mesh, then lift the charts onto
// the fine mesh. Patches whose lifted map is not good enough fall back to the ordinary parameterization and
// refinement on the fine mesh.
//   fine mesh -> simplified mesh -> VSA -> patches + parameterization (+ refinement) on the coarse mesh
//             -> fine labels (closest coarse triangle) -> border smoothing -> fine patches
//             -> lifted UV per patch -> Refiner on the fine mesh for the patches that did not survive
struct CoarseOptions {
    SimplifyOptions simplify;            // targetFaces is the size of the coarse mesh
    int          k = 100;                // proxies of the coarse VSA
    Metric       metric = Metric::L21;
    SmoothParams smooth{0.1, 0.0, 10};
    LloydParams  lloyd;
    bool         pruneSmallParts = true;   // the simplified mesh drops tiny floating pieces
    int          fillHolesUpTo = 24;       // holes of the simplified mesh up to this many edges are closed (0 = leave them)
    double       fineSmoothing = 0.1;    // border smoothing of the transferred labels on the fine mesh (0 = off)
    RefineOptions refine;                // coarse refinement, extraction and the fine fallback
    LiftOptions  lift;
    std::function<void(const std::string&)> onStage;   // progress messages (called from the working thread)
};

struct CoarseStats {
    size_t coarseFaces = 0;
    int    coarsePatches = 0;
    int    holesClosed = 0;              // holes of the simplified mesh that were closed
    int    coarseInitialBad = 0, coarseSplits = 0, coarseRounds = 0, coarseUnresolved = 0;   // of the refinement of the simplified mesh
    int    finePatches = 0;              // after the lift, before the fallback
    int    lifted = 0;                   // lifted patches that passed the checks
    int    liftFailed = 0;               // lifted patches that did not
    int    notLifted = 0;                // patches without a usable coarse chart (not a disk, ...)
    long   outsideVertices = 0;          // solved again at the chart borders
    double simplifySeconds = 0, coarseVsaSeconds = 0, coarseRefineSeconds = 0, mapSeconds = 0, labelSeconds = 0,
           liftSeconds = 0, fineRefineSeconds = 0;
};

struct CoarseResult {
    RefineResult fine;                   // labels, patches and parameterizations of the fine mesh
    CoarseStats  stats;
};

// The coarse half shared by both pipelines: simplification, VSA, parameterization of the coarse patches.
struct CoarseStage {
    TriMesh      mesh;        // the simplified mesh (faces OpenMesh refused are missing)
    FlatMesh     flat;        // the same mesh as flat arrays, indexed like `mesh`
    RefineResult result;      // patches and charts of the coarse mesh; result.labels = coarse patch per coarse face
};
CoarseStage runCoarseStage(const FlatMesh& fine, const CoarseOptions& options, CoarseStats& stats);

CoarseResult runCoarsePipeline(const TriMesh& fine, const CoarseOptions& options = {});
