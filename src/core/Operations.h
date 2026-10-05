#pragma once

#include "core/Partition.h"

// Proxy operations of VSA section 3.5. All of them work on a partitioned state
// (s.label.size() == #faces) and leave s.regions consistent with the labels.

struct RegionPair {
    int    a = -1, b = -1;   // a < b
    double cost = 0.0;       // E(fit(Ra + Rb)) - E(Ra) - E(Rb): error increase of merging, O(1) from the moments
};

// Cheapest merge among region pairs that share an edge. false if there is no such pair.
bool findBestMerge(const VSAInput& in, const VSAState& s, RegionPair& out);

// Merges region b into a (the proxy is refitted) and removes proxy b.
// Indices change: the last proxy takes the place of b.
bool deleteProxy(const VSAInput& in, VSAState& s, int a, int b);

// Adds a proxy at the worst face of the worst region that has at least two faces.
// The next partition uses this face as the seed of the new proxy. false if nothing can be split.
bool insertProxy(const VSAInput& in, VSAState& s);

// Teleportation: delete the cheapest pair and insert a proxy where the error is largest,
// but only if the merge cost is below `ratio` times the error of the worst other region.
bool teleportProxy(const VSAInput& in, VSAState& s, double ratio = 0.5);

struct TeleportParams {
    double ratio = 0.5;
    int    maxTeleports = 50;
};

// Incremental driver for long-running refinement, so a UI can advance it one Lloyd iteration
// per frame. One call of step() is either one Lloyd iteration or the proxy operation that follows
// a finished Lloyd run.
//   Lloyd   - plain Lloyd until convergence.
//   Grow    - Lloyd, then {insert a proxy, Lloyd} until the state has `targetK` proxies.
//   Teleport- Lloyd, then repeated {teleport, Lloyd}. A teleportation is kept only if the final
//             energy improves; otherwise the state is restored and the job ends.
class RefineJob {
public:
    enum class Kind { Lloyd, Grow, Teleport };

    explicit RefineJob(Kind kind, const LloydParams& lp = {}, const TeleportParams& tp = {}, int targetK = 0)
        : kind_(kind), lp_(lp), tp_(tp), targetK_(targetK) {}

    // false once the job is finished (the call that finishes it still did its work)
    bool step(const VSAInput& in, VSAState& s);

    bool finished() const { return done_; }
    int  iterations() const { return iterations_; }   // Lloyd iterations over the whole job
    int  teleports() const { return teleports_; }     // accepted teleportations
    bool lastRunConverged() const { return converged_; }

private:
    Kind        kind_;
    LloydParams lp_;
    TeleportParams tp_;
    int         targetK_;
    bool        done_ = false, converged_ = false, hasSaved_ = false;
    int         iterations_ = 0, runIterations_ = 0, teleports_ = 0;
    VSAState    saved_;
};

LloydResult runLloydTeleport(const VSAInput& in, VSAState& s, const LloydParams& lp = {},
                             const TeleportParams& tp = {});

// Farthest-point seeding: the first seed is random, every next one maximizes the distance
// (over the dual graph, edge length = distance between face centroids) to the chosen seeds.
// Components without seeds are at infinite distance, so each gets a seed early.
void initFarthestSeeds(const VSAInput& in, VSAState& s, int k, uint64_t rngSeed);

// Incremental refinement: Lloyd, then {insert a proxy, Lloyd} until k proxies (or nothing to split).
LloydResult growToK(const VSAInput& in, VSAState& s, int k, const LloydParams& lp = {});
