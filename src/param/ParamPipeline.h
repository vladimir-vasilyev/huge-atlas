#pragma once

#include "param/AnglesToUV.h"
#include "param/Distortion.h"
#include "param/Injectivity.h"
#include "param/LinABF.h"
#include "param/PatchFix.h"

#include <string>
#include <vector>

// Parameterization of one patch with fixes of increasing cost, stopping at the first injective map:
//   1. LinABF + least squares UV
//   2. LinABF with the active set (pinned angles, limited boundary fan sums), re-solved
//   3. a slit from the vertex with the largest angular defect to the boundary, then 1-2 again
//      (up to maxCuts times)
//   4. Tutte embedding of the original patch: injective for every disk, but with a distorted boundary
// A patch that stays non-injective, comes from the Tutte fallback, or is too stretched, is marked needsSplit: it has
// to be split by VSA again (the loop of stage 10).

enum class ParamMethod { None, LinABF, ActiveSet, Cut, Tutte, Lifted, Lscm, Repaired };
const char* paramMethodName(ParamMethod method);

struct ParamOptions {
    LinABFParams    lin;
    ActiveSetParams active;
    bool   useActiveSet = true;
    bool   allowCut = true;
    int    maxCuts = 2;
    double cutKappa = 1.0;
    bool   repairNearMisses = true;   // a LinABF / active set map with a few flips or crossings is repaired locally before slits and Tutte
    int    repairMaxProblems = 64;    // ... when it has at most this many flipped faces + crossings
    bool   allowLscm = true;          // before Tutte: least squares conformal map (free border, much less angle distortion), kept when injective
    bool   allowTutte = true;
    double maxStretch = 4.0;   // a triangle with a larger L2 stretch (or a flipped one) is "too distorted" (0 = ignore)
    double maxBadAreaFraction = 0.005;   // needsSplit when the too distorted triangles cover more than this share of the area
};

struct PatchParam {
    ParamMethod method = ParamMethod::None;
    bool        injective = false;
    bool        needsSplit = false;
    std::string log;                       // what was tried and why it failed
    Patch       patch;                     // the patch that was parameterized (cut copy if method == Cut)
    std::vector<double> angles;            // final corner angles (empty for Tutte)
    std::vector<Eigen::Vector2d> uv;       // per vertex of `patch.mesh`
    InjectivityReport report;              // of the final result
    DistortionStats distortion;
    double      badAreaFraction = 0.0;     // share of the area in triangles with L2 stretch above maxStretch (or flipped)
    std::vector<float> perTriangleL2;      // per face of `patch.mesh`
    int         cuts = 0;

    // Where a failed attempt overlapped itself: two faces (source mesh indices) next to a pair of crossing
    // boundary edges that are far apart on the surface, or -1. The refinement seeds the split with them.
    int         hintFaceA = -1, hintFaceB = -1;
};

// The pair of crossing boundary edges (of rep.uv) whose adjacent faces are farthest apart in 3D: their faces (indices in
// `source`) are the seeds of a split. Leaves hintA/hintB alone when there are no crossings.
void crossingHintFaces(const TriMesh& source, const Patch& patch, const InjectivityReport& rep, int& hintA, int& hintB);

PatchParam parameterizePatch(const TriMesh& source, const Patch& patch, const ParamOptions& options = {});
