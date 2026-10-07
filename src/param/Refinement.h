#pragma once

#include "core/Partition.h"
#include "param/ParamPipeline.h"
#include "param/PatchExtractor.h"

#include <cstdint>
#include <unordered_map>
#include <functional>
#include <vector>

// The refinement loop: patches that cannot be flattened well are split again by a local VSA with
// two proxies, restricted to the faces of the patch. Neighbouring patches are never touched.
//
// A patch is "bad" if it is not a disk, or its best parameterization is not injective, or its
// L2 stretch is above ParamOptions::maxStretch (all of that is PatchParam::needsSplit).
//
// The two seeds of a split are, in this order of preference:
//   1. the faces next to a pair of crossing boundary edges that are far apart on the surface
//      (PatchParam::hintFaceA / B): the two overlapping parts of the patch;
//   2. two farthest faces of the patch (distance over the dual graph).
// A split that leaves one part smaller than minFaces falls back to a single flooding step and then
// to a Voronoi split over the dual graph.

enum class RefinePolicy {
    FixFirst,          // run the whole repair ladder (active set, slits, Tutte) and split only what still fails
    SplitImmediately   // plain LinABF only: a patch that is not injective right away is split
};

class ThreadPool;

struct RefineOptions {
    RefinePolicy policy = RefinePolicy::FixFirst;
    ParamOptions param;              // ladder and thresholds (maxStretch decides "too distorted")
    PatchOptions patch;              // extraction of the patches
    Metric       metric = Metric::L21;
    SmoothParams smooth{0.1, 0.0, 10};   // border smoothing of the local VSA
    LloydParams  lloyd;              // the local Lloyd (maxIterations is lowered below)
    int maxDepth = 6;                // how many times a face may be re-split
    int minFaces = 8;                // both parts of a split need at least this many faces
    int maxRounds = 12;
    int maxPatches = 100000;
    // A patch of more faces than this is only tried with plain LinABF when it can still be split: the repair ladder is slow on
    // big patches (a factorization of tens of seconds per step) and the split fixes most of them anyway. 0 = never.
    int splitAtOnceAboveFaces = 40000;
    // Replaces parameterizePatch (the coarse-to-fine pipeline lifts a chart instead). Called from several threads.
    std::function<PatchParam(const TriMesh& source, const Patch& patch, const ParamOptions& options)> parameterize;
    ThreadPool* pool = nullptr;      // for the parallel loops inside (null = the shared pool); a worker of the shared pool must pass its own

    RefineOptions() { lloyd.maxIterations = 30; }
};

struct RefineStats {
    int rounds = 0;
    int initialBad = 0;
    int splits = 0;
    int splitsByCrossingHint = 0, splitsByFarthest = 0, splitsByFlood = 0, splitsByVoronoi = 0;
    int unresolved = 0;                 // bad patches that stayed bad (limits or a split that did not work)
    std::vector<int> depthHistogram;    // patches by depth of re-splitting
    double seconds = 0.0;
};

struct RefineResult {
    std::vector<int> labels;                 // per face, the final region = patch index
    // patches of `labels`. For a patch that has a parameterization the mesh of this copy is released (it is the same
    // mesh as params[i].patch, unless the patch was cut there): use params[i].patch.mesh. faceToGlobal and the
    // topology stay.
    ExtractionResult extraction;
    std::vector<PatchParam> params;          // parallel to extraction.patches (empty method for non-disks)
    std::vector<char> bad;                   // parallel: still needs a split
    RefineStats stats;
};

// Steps through the loop one round at a time. `mesh` must outlive the refiner.
class Refiner {
public:
    Refiner(const TriMesh& mesh, std::vector<int> labels, const RefineOptions& options);

    // One round: evaluate every patch (cached when its faces are unchanged), split the bad ones.
    // false once finished: nothing is bad, nothing could be split, or the round limit is reached.
    bool step();

    // Starts from patches that are already parameterized (same labels as `ex.faceToPatch`): the first round reuses
    // every patch whose faces stay the same instead of parameterizing it again. Call before the first step().
    void seed(ExtractionResult&& ex, std::vector<PatchParam>&& params);

    bool finished() const { return finished_; }
    const RefineResult& result() const { return result_; }   // the last evaluation (final once finished)
    // Moves the result out (no copy of the patches): for the caller that keeps it. The refiner is finished and
    // empty afterwards.
    RefineResult takeResult() { finished_ = true; return std::move(result_); }
    const std::vector<int>& labels() const { return labels_; }

private:
    void evaluate();
    bool splitPatch(int patchIndex);

    const TriMesh& mesh_;
    RefineOptions opt_;
    std::vector<int> labels_;
    std::vector<int> faceDepth_;
    RefineResult result_;
    bool finished_ = false;
    int nextLabel_ = 0;
};

RefineResult refinePatches(const TriMesh& mesh, std::vector<int> labels, const RefineOptions& options = {});
