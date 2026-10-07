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
    double       regularization = 1e-6;     // pull of the solved vertices towards their clamped position
};

struct LiftStats {
    int vertices = 0;
    int outside = 0;        // vertices that had to be solved again
    int bandFaces = 0;      // faces of that solve
    int repairs = 0;        // local re-solves around flipped triangles that helped
};

// `coarse` must be a parameterized disk (coarse.patch.mesh with coarse.uv). `finePatch` is a patch of the fine mesh
// (a disk). The returned parameterization has method Lifted; needsSplit tells whether it must be redone.
// `source` (the mesh finePatch was cut from), when given, lets a failed lift suggest where to split (hintFaceA/B).
PatchParam liftPatch(const Patch& finePatch, const PatchParam& coarse, const LiftOptions& options = {},
                     LiftStats* stats = nullptr, const TriMesh* source = nullptr);
