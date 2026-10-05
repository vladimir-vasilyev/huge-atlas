#include "core/Partition.h"

#include <algorithm>
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

void recomputeRegions(const VSAInput& in, VSAState& s)
{
    s.regions.assign(s.proxies.size(), Moments());
    for (size_t f = 0; f < s.label.size(); ++f)
        if (s.label[f] >= 0)
            s.regions[s.label[f]] += in.moments[f];
}

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

// Face of minimum error among those labeled `proxy` in `prev`. Degenerate faces have ~zero error
// for any proxy, so they are chosen only if nothing else is available.
int minErrorFace(const VSAInput& in, const VSAState& s, const std::vector<int>& prev, int proxy)
{
    int best = -1, bestDegenerate = -1;
    double bestErr = 0.0, bestDegErr = 0.0;
    for (size_t f = 0; f < prev.size(); ++f) {
        if (prev[f] != proxy) continue;
        const double e = metricError(s.metric, in.moments[f], s.proxies[proxy]);
        if (in.faces.degenerate[f]) {
            if (bestDegenerate < 0 || e < bestDegErr) { bestDegenerate = int(f); bestDegErr = e; }
        } else if (best < 0 || e < bestErr) {
            best = int(f); bestErr = e;
        }
    }
    return best >= 0 ? best : bestDegenerate;
}

} // namespace

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

    s.proxies.clear();
    s.seedFace.assign(candidates.begin(), candidates.begin() + k);
    for (int f : s.seedFace)
        s.proxies.push_back(fitProxy(s.metric, in.moments[f]));
    s.label.clear();
    s.regions.assign(s.proxies.size(), Moments());
}

int partition(const VSAInput& in, VSAState& s)
{
    const size_t nF = in.mesh.n_faces();
    const int k0 = s.k();

    // seed of every proxy
    std::vector<int> prev;
    prev.swap(s.label);
    std::vector<int> seeds(k0, -1);
    for (int i = 0; i < k0; ++i) {
        int f = prev.size() == nF ? minErrorFace(in, s, prev, i) : -1;
        if (f < 0 && i < int(s.seedFace.size())) f = s.seedFace[i];
        seeds[i] = f;
    }

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
