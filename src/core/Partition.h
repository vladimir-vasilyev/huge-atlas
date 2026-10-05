#pragma once

#include "core/MeshTypes.h"
#include "core/Metrics.h"
#include "core/Proxy.h"

#include <cstdint>
#include <vector>

// Immutable per-mesh inputs of the algorithm (not owned).
struct VSAInput {
    const TriMesh&              mesh;
    const FaceData&             faces;
    const std::vector<Moments>& moments;   // per face, see computeFaceMoments
};

// All mutable VSA state in one object.
struct VSAState {
    Metric              metric = Metric::L21;
    std::vector<Proxy>  proxies;
    std::vector<Moments> regions;    // per-proxy moments of the faces currently labeled with it
    std::vector<int>    label;       // per face: proxy index, -1 = unassigned (empty before the first partition)
    std::vector<double> energyHistory;   // E after every Lloyd step (lloydStep)
    std::vector<int>   seedFace;     // initial seed face per proxy; used by the first partition only

    int k() const { return int(proxies.size()); }
};

// Installs the given faces as seeds (a proxy is fitted to each) and clears the labels.
void setSeeds(const VSAInput& in, VSAState& s, const std::vector<int>& seedFaces);

// Recomputes s.regions from the labels (unassigned faces are skipped).
void recomputeRegions(const VSAInput& in, VSAState& s);

// Picks min(k, #non-degenerate faces) distinct random faces as seeds and fits a proxy to each.
// Deterministic for a given rng seed. Clears the labels.
void initRandomSeeds(const VSAInput& in, VSAState& s, int k, uint64_t rngSeed);

// Geometry partitioning by greedy simultaneous flooding (VSA, section 3.3).
//  - Seeds: the first call uses the initial seed faces, later calls use the face of
//    minimum error inside each previous region.
//  - Faces are claimed in order of increasing error w.r.t. the proxy that reaches them.
//  - A connected component without any seed gets a new proxy (k grows), fitted to its first face.
// Updates label and regions. Returns the number of proxies added.
int partition(const VSAInput& in, VSAState& s);

// Proxy fitting step: recomputes regions from labels and refits every proxy.
void fitProxies(const VSAInput& in, VSAState& s);

// Total error E(R, P) = sum over regions of metricError.
double totalEnergy(const VSAState& s);

// ---- Lloyd iterations (partition -> fit). Greedy flooding does not guarantee a monotone energy,
// so convergence is judged by a fixed point or by a small relative change, never by monotonicity.
struct LloydParams {
    int    maxIterations = 100;
    double relTolerance  = 1e-4;   // |E_{i-1} - E_i| / max(E_{i-1}, E_i) below this ...
    int    patience      = 3;      // ... for this many consecutive iterations
};

struct LloydStepInfo {
    int    added = 0;               // proxies added for unseeded components
    bool   labelsChanged = true;
    double energy = 0.0;            // after the fit
};

struct LloydResult {
    int  iterations = 0;
    bool converged = false;
    int  teleports = 0;             // accepted proxy teleportations (runLloydTeleport)
};

// One iteration: partition + fit. Appends the energy to s.energyHistory.
LloydStepInfo lloydStep(const VSAInput& in, VSAState& s);

// Fixed point (labels unchanged) or relative energy change below tolerance for `patience` steps.
bool lloydConverged(const VSAState& s, const LloydParams& p, const LloydStepInfo& last);

LloydResult runLloyd(const VSAInput& in, VSAState& s, const LloydParams& p = {});
