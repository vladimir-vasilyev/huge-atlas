#include "core/Operations.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <set>

namespace {

bool partitioned(const VSAInput& in, const VSAState& s)
{
    return s.label.size() == in.mesh.n_faces() && s.regions.size() == s.proxies.size();
}

double regionErr(const VSAState& s, int i)
{
    return metricError(s.metric, s.regions[i], s.proxies[i]);
}

std::vector<int> regionFaceCounts(const VSAState& s)
{
    std::vector<int> n(s.proxies.size(), 0);
    for (int l : s.label) if (l >= 0) ++n[l];
    return n;
}

} // namespace

bool findBestMerge(const VSAInput& in, const VSAState& s, RegionPair& out)
{
    if (!partitioned(in, s) || s.k() < 2) return false;

    // pairs of regions that share an edge: scan interior edges once
    std::set<std::pair<int, int>> pairs;
    for (auto e : in.mesh.edges()) {
        if (in.mesh.is_boundary(e)) continue;
        const auto h = in.mesh.halfedge_handle(e, 0);
        const int la = s.label[in.mesh.face_handle(h).idx()];
        const int lb = s.label[in.mesh.face_handle(in.mesh.opposite_halfedge_handle(h)).idx()];
        if (la >= 0 && lb >= 0 && la != lb)
            pairs.insert({std::min(la, lb), std::max(la, lb)});
    }

    bool found = false;
    for (const auto& [a, b] : pairs) {
        const Moments u = s.regions[a] + s.regions[b];
        const double cost = metricError(s.metric, u, fitProxy(s.metric, u)) - regionErr(s, a) - regionErr(s, b);
        if (!found || cost < out.cost) {
            out = {a, b, cost};
            found = true;
        }
    }
    return found;
}

bool deleteProxy(const VSAInput& in, VSAState& s, int a, int b)
{
    if (!partitioned(in, s) || a == b || a < 0 || b < 0 || a >= s.k() || b >= s.k()) return false;

    const Proxy merged = fitProxy(s.metric, s.regions[a] + s.regions[b]);
    const int last = s.k() - 1;
    // the merged region keeps index a, except that a == last would be popped: then it takes b's slot
    const int na = (a == last && b != last) ? b : a;
    for (int& l : s.label) {
        if (l == b) l = na;
        else if (l == last && b != last) l = b;   // the last proxy takes over b's index
    }
    if (b != last) s.proxies[b] = s.proxies[last];
    s.proxies.pop_back();
    s.proxies[na] = merged;
    s.seedFace.clear();    // only meaningful before the first partition
    recomputeRegions(in, s);
    return true;
}

bool insertProxy(const VSAInput& in, VSAState& s)
{
    if (!partitioned(in, s) || s.k() == 0) return false;

    const std::vector<int> count = regionFaceCounts(s);
    std::vector<int> order;
    for (int i = 0; i < s.k(); ++i)
        if (count[i] >= 2) order.push_back(i);
    std::sort(order.begin(), order.end(), [&](int x, int y) {
        const double ex = regionErr(s, x), ey = regionErr(s, y);
        return ex != ey ? ex > ey : x < y;
    });

    for (int r : order) {
        int worst = -1;
        double worstErr = 0.0;
        for (size_t f = 0; f < s.label.size(); ++f) {
            if (s.label[f] != r || in.faces.degenerate[f]) continue;
            const double e = metricError(s.metric, in.moments[f], s.proxies[r]);
            if (worst < 0 || e > worstErr) { worst = int(f); worstErr = e; }
        }
        if (worst < 0 || worstErr <= 0.0) continue;   // region is already exact

        s.proxies.push_back(fitProxy(s.metric, in.moments[worst]));
        s.label[worst] = s.k() - 1;
        recomputeRegions(in, s);
        // refit the old proxy without the removed face; otherwise it stays equidistant to the
        // neighbours of the new seed and keeps faces that belong to the new proxy
        s.proxies[r] = fitProxy(s.metric, s.regions[r]);
        return true;
    }
    return false;
}

bool teleportProxy(const VSAInput& in, VSAState& s, double ratio)
{
    RegionPair m;
    if (!partitioned(in, s) || s.k() < 3 || !findBestMerge(in, s, m)) return false;

    const std::vector<int> count = regionFaceCounts(s);
    double worstErr = 0.0;
    for (int i = 0; i < s.k(); ++i)
        if (i != m.a && i != m.b && count[i] >= 2)
            worstErr = std::max(worstErr, regionErr(s, i));
    if (worstErr <= 0.0 || m.cost >= ratio * worstErr) return false;

    deleteProxy(in, s, m.a, m.b);
    return insertProxy(in, s);
}

bool RefineJob::step(const VSAInput& in, VSAState& s)
{
    if (done_) return false;

    const LloydStepInfo info = lloydStep(in, s);
    ++iterations_;
    ++runIterations_;
    const bool conv = lloydConverged(s, lp_, info);
    if (!conv && runIterations_ < lp_.maxIterations) return true;

    // this Lloyd run is over
    converged_ = conv;
    runIterations_ = 0;
    switch (kind_) {
    case Kind::Lloyd:
        done_ = true;
        break;
    case Kind::Grow:
        if (s.k() >= targetK_ || !insertProxy(in, s)) done_ = true;
        break;
    case Kind::Teleport:
        if (hasSaved_) {
            if (totalEnergy(s) < totalEnergy(saved_) * (1.0 - 1e-9)) {
                ++teleports_;
            } else {
                s = saved_;
                done_ = true;
                break;
            }
        }
        if (teleports_ >= tp_.maxTeleports) { done_ = true; break; }
        saved_ = s;
        hasSaved_ = true;
        if (!teleportProxy(in, s, tp_.ratio)) {
            s = saved_;          // a failed teleport may have deleted a proxy already
            done_ = true;
        }
        break;
    }
    return !done_;
}

LloydResult runLloydTeleport(const VSAInput& in, VSAState& s, const LloydParams& lp, const TeleportParams& tp)
{
    RefineJob job(RefineJob::Kind::Teleport, lp, tp);
    while (job.step(in, s)) {}
    return {job.iterations(), job.lastRunConverged(), job.teleports()};
}

void initFarthestSeeds(const VSAInput& in, VSAState& s, int k, uint64_t rngSeed)
{
    const size_t nF = in.mesh.n_faces();
    std::vector<int> candidates;
    for (size_t f = 0; f < nF; ++f)
        if (!in.faces.degenerate[f]) candidates.push_back(int(f));
    if (candidates.empty())
        for (size_t f = 0; f < nF; ++f) candidates.push_back(int(f));
    k = std::max(1, std::min<int>(k, int(candidates.size())));

    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(nF, kInf);
    std::vector<char> isSeed(nF, 0);
    std::vector<int> seeds;

    using Item = std::pair<double, int>;   // (distance, face), min-heap
    auto addSeed = [&](int f) {
        seeds.push_back(f);
        isSeed[f] = 1;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
        dist[f] = 0.0;
        q.push({0.0, f});
        while (!q.empty()) {
            const auto [d, u] = q.top();
            q.pop();
            if (d > dist[u]) continue;
            for (auto nb : in.mesh.ff_range(TriMesh::FaceHandle(u))) {
                const double nd = d + (in.faces.centroid[nb.idx()] - in.faces.centroid[u]).norm();
                if (nd < dist[nb.idx()]) { dist[nb.idx()] = nd; q.push({nd, nb.idx()}); }
            }
        }
    };

    std::mt19937_64 rng(rngSeed);
    addSeed(candidates[std::uniform_int_distribution<size_t>(0, candidates.size() - 1)(rng)]);
    while (int(seeds.size()) < k) {
        int best = -1;
        double bestD = -1.0;
        for (int f : candidates)
            if (!isSeed[f] && dist[f] > bestD) { bestD = dist[f]; best = f; }
        if (best < 0) break;
        addSeed(best);
    }
    setSeeds(in, s, seeds);
}

LloydResult growToK(const VSAInput& in, VSAState& s, int k, const LloydParams& lp)
{
    RefineJob job(RefineJob::Kind::Grow, lp, {}, k);
    while (job.step(in, s)) {}
    return {job.iterations(), job.lastRunConverged(), 0};
}
