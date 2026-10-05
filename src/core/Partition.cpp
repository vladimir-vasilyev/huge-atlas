#include "core/Partition.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>
#include <random>

namespace {

struct QueueItem {
    double error;
    int    face;
    int    proxy;
    // std::priority_queue is a max-heap: invert the order; ties broken by face/proxy for determinism
    bool operator<(const QueueItem& o) const
    {
        if (error != o.error) return error > o.error;
        if (face != o.face) return face > o.face;
        return proxy > o.proxy;
    }
};

using Queue = std::priority_queue<QueueItem>;

// Claims `face` for `proxy` and queues its unlabeled neighbors.
void claim(const VSAInput& in, VSAState& s, Queue& q, int face, int proxy)
{
    s.label[face] = proxy;
    for (auto nb : in.mesh.ff_range(TriMesh::FaceHandle(face))) {
        if (s.label[nb.idx()] < 0)
            q.push({metricError(s.metric, in.moments[nb.idx()], s.proxies[proxy]), nb.idx(), proxy});
    }
}

void runQueue(const VSAInput& in, VSAState& s, Queue& q)
{
    while (!q.empty()) {
        const QueueItem it = q.top();
        q.pop();
        if (s.label[it.face] >= 0) continue;   // lazy deletion: already claimed by a cheaper entry
        claim(in, s, q, it.face, it.proxy);
    }
}

// For every proxy: the face of minimum error among those labeled with it in `prev` (one pass).
// Degenerate faces have ~zero error for any proxy, so they are chosen only if nothing else is available.
std::vector<int> minErrorFaces(const VSAInput& in, const VSAState& s, const std::vector<int>& prev)
{
    const size_t k = s.proxies.size();
    std::vector<int> best(k, -1), bestDegenerate(k, -1);
    std::vector<double> bestErr(k, 0.0), bestDegErr(k, 0.0);
    for (size_t f = 0; f < prev.size(); ++f) {
        const int r = prev[f];
        if (r < 0 || size_t(r) >= k) continue;
        const double e = metricError(s.metric, in.moments[f], s.proxies[r]);
        if (in.faces.degenerate[f]) {
            if (bestDegenerate[r] < 0 || e < bestDegErr[r]) { bestDegenerate[r] = int(f); bestDegErr[r] = e; }
        } else if (best[r] < 0 || e < bestErr[r]) {
            best[r] = int(f); bestErr[r] = e;
        }
    }
    for (size_t r = 0; r < k; ++r)
        if (best[r] < 0) best[r] = bestDegenerate[r];
    return best;
}

} // namespace

void recomputeRegions(const VSAInput& in, VSAState& s)
{
    s.regions.assign(s.proxies.size(), Moments());
    for (size_t f = 0; f < s.label.size(); ++f)
        if (s.label[f] >= 0)
            s.regions[s.label[f]] += in.moments[f];
}

void setSeeds(const VSAInput& in, VSAState& s, const std::vector<int>& seedFaces)
{
    s.seedFace = seedFaces;
    s.proxies.clear();
    for (int f : s.seedFace)
        s.proxies.push_back(fitProxy(s.metric, in.moments[f]));
    s.label.clear();
    s.regions.assign(s.proxies.size(), Moments());
    s.energyHistory.clear();
}

void initRandomSeeds(const VSAInput& in, VSAState& s, int k, uint64_t rngSeed)
{
    std::vector<int> candidates;
    for (size_t f = 0; f < in.faces.degenerate.size(); ++f)
        if (!in.faces.degenerate[f]) candidates.push_back(int(f));
    if (candidates.empty()) {   // nothing but degenerate faces: still return a valid state
        candidates.resize(in.faces.degenerate.size());
        std::iota(candidates.begin(), candidates.end(), 0);
    }

    k = std::max(1, std::min<int>(k, int(candidates.size())));

    // partial Fisher-Yates shuffle: the first k entries are a uniform random subset
    std::mt19937_64 rng(rngSeed);
    for (int i = 0; i < k; ++i) {
        std::uniform_int_distribution<size_t> d(size_t(i), candidates.size() - 1);
        std::swap(candidates[i], candidates[d(rng)]);
    }

    setSeeds(in, s, std::vector<int>(candidates.begin(), candidates.begin() + k));
}

int partition(const VSAInput& in, VSAState& s)
{
    const size_t nF = in.mesh.n_faces();
    const int k0 = s.k();

    // seed of every proxy
    std::vector<int> prev;
    prev.swap(s.label);
    std::vector<int> seeds = prev.size() == nF ? minErrorFaces(in, s, prev) : std::vector<int>(k0, -1);
    for (int i = 0; i < k0; ++i)
        if (seeds[i] < 0 && i < int(s.seedFace.size()))
            seeds[i] = s.seedFace[i];

    s.label.assign(nF, -1);
    Queue q;
    for (int i = 0; i < k0; ++i)
        if (seeds[i] >= 0 && s.label[seeds[i]] < 0)
            claim(in, s, q, seeds[i], i);
    runQueue(in, s, q);

    // components that no seed could reach
    int added = 0;
    for (size_t f = 0; f < nF; ++f) {
        if (s.label[f] >= 0) continue;
        s.proxies.push_back(fitProxy(s.metric, in.moments[f]));
        claim(in, s, q, int(f), s.k() - 1);
        runQueue(in, s, q);
        ++added;
    }

    recomputeRegions(in, s);
    return added;
}

void fitProxies(const VSAInput& in, VSAState& s)
{
    if (s.label.size() != in.mesh.n_faces()) return;   // nothing to fit before the first partition
    recomputeRegions(in, s);
    for (size_t i = 0; i < s.proxies.size(); ++i)
        if (s.regions[i].area > 0.0)
            s.proxies[i] = fitProxy(s.metric, s.regions[i]);
}

double totalEnergy(const VSAState& s)
{
    double e = 0.0;
    for (size_t i = 0; i < s.proxies.size() && i < s.regions.size(); ++i)
        e += metricError(s.metric, s.regions[i], s.proxies[i]);
    return e;
}

LloydStepInfo lloydStep(const VSAInput& in, VSAState& s)
{
    LloydStepInfo info;
    const std::vector<int> before = s.label;
    info.added = partition(in, s);
    info.labelsChanged = before != s.label;
    fitProxies(in, s);
    info.energy = totalEnergy(s);
    s.energyHistory.push_back(info.energy);
    return info;
}

bool lloydConverged(const VSAState& s, const LloydParams& p, const LloydStepInfo& last)
{
    // fixed point: the partition did not change, so fitting reproduces the same proxies
    if (!last.labelsChanged && last.added == 0) return true;

    const auto& h = s.energyHistory;
    const int n = int(h.size());
    if (n < p.patience + 1) return false;
    for (int i = n - p.patience; i < n; ++i) {
        const double denom = std::max({std::abs(h[i - 1]), std::abs(h[i]), 1e-300});
        if (std::abs(h[i - 1] - h[i]) / denom >= p.relTolerance) return false;
    }
    return true;
}

LloydResult runLloyd(const VSAInput& in, VSAState& s, const LloydParams& p)
{
    LloydResult r;
    while (r.iterations < p.maxIterations) {
        const LloydStepInfo info = lloydStep(in, s);
        ++r.iterations;
        if (lloydConverged(s, p, info)) { r.converged = true; break; }
    }
    return r;
}
