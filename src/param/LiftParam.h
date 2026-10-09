#pragma once

#include "param/ParamPipeline.h"

// Lifting the parameterization of a coarse patch onto the fine patch that covers the same part of the surface
// (the coarse mesh is a simplification of the fine one):
//   1. every fine vertex takes the UV of the closest point of the coarse chart (barycentric interpolation);
//   2. vertices whose closest point had to be clamped onto the chart border lie outside of it: their UV is
//      solved again, as a least squares conformal map of the triangles around them with all other vertices fixed;
//   3. the result is checked like any other parameterization (flips, border crossings, stretch).
struct LiftOptions {
    ParamOptions param;              // thresholds of the check (maxStretch, maxBadAreaFraction)
    double       outsideTolerance = 1e-6;   // barycentric weight below which a point counts as lying on an edge
    bool         kernelRepair = true;       // ABF++: put the vertices of flipped faces into the kernel of their link, relax locally
    double       regularization = 1e-6;     // pull of the solved vertices towards their clamped position
};

struct LiftStats {
    int vertices = 0;
    int outside = 0;        // vertices that had to be solved again
    int bandFaces = 0;      // faces of that solve
    int kernelMoves = 0;    // vertices moved into the kernel of their link
    int repairs = 0;        // local re-solves around flipped triangles that helped
};

// Least squares conformal map of a disk with a free border (Levy et al. 2002): two far apart border vertices are pinned,
// the rest is solved. Empty result (and `error`) when the mesh has no border or the solve fails. Not injective in general.
std::vector<Eigen::Vector2d> lscmEmbedding(const TriMesh& mesh, std::string* error = nullptr);

// Repairs a map with a few flipped faces or border crossings (kernel placement, local LSCM in growing rings, straightening of a
// tangled single border loop); `uv` and `report` are updated in place. Used by liftPatch and for LSCM maps.
void repairMap(const TriMesh& mesh, const std::vector<std::vector<int>>& borderLoops, std::vector<Eigen::Vector2d>& uv, UVReport& report,
               const LiftOptions& options, LiftStats& stats);

// `coarse` must be a parameterized disk (coarse.patch.mesh with coarse.uv). `finePatch` is a patch of the fine mesh
// (a disk). The returned parameterization has method Lifted; needsSplit tells whether it must be redone.
// `source` (the mesh finePatch was cut from), when given, lets a failed lift suggest where to split (hintFaceA/B).
PatchParam liftPatch(const Patch& finePatch, const PatchParam& coarse, const LiftOptions& options = {},
                     LiftStats* stats = nullptr, const TriMesh* source = nullptr);

// ABF++ tools, exposed for tests: every interior vertex of a flipped face (indices into `mesh`) is moved into the kernel of its
// link polygon when that clears its triangles (repeated up to a few passes), then the moved vertices and their neighbours are
// relaxed with mean value weights, a vertex staying put when the move would flip a triangle. Returns the number of moved vertices.
int repairFlipsByKernel(const TriMesh& mesh, std::vector<Eigen::Vector2d>& uv, const std::vector<int>& flippedFaces, int relaxSweeps = 3);
