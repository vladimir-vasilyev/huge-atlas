#include "core/Partition.h"

#include "core/ParallelUtil.h"
#include "core/Smoothing.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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
    struct Best { int face = -1; double err = 0.0; };
    constexpr size_t kChunk = 1 << 16;
    const size_t chunks = (prev.size() + kChunk - 1) / kChunk;
    // per chunk and proxy: best regular and best degenerate face (strictly smaller error wins, so the
    // lowest face index wins ties; merging chunks in order keeps that rule)
    std::vector<Best> part(2 * k * std::max<size_t>(chunks, 1));
    forChunks(poolOf(s), prev.size(), kChunk, [&](size_t c, size_t b, size_t e) {
        Best* regular = &part[2 * k * c];
        Best* degenerate = regular + k;
        for (size_t f = b; f < e; ++f) {
            const int r = prev[f];
            if (r < 0 || size_t(r) >= k) continue;
            const double err = metricError(s.metric, in.moments[f], s.proxies[r]);
            Best& slot = in.faces.degenerate[f] ? degenerate[r] : regular[r];
            if (slot.face < 0 || err < slot.err) { slot.face = int(f); slot.err = err; }
        }
    });
    std::vector<int> best(k, -1);
    for (size_t r = 0; r < k; ++r) {
        Best reg, deg;
        for (size_t c = 0; c < chunks; ++c) {
            const Best& a = part[2 * k * c + r];
            const Best& d = part[2 * k * c + k + r];
            if (a.face >= 0 && (reg.face < 0 || a.err < reg.err)) reg = a;
            if (d.face >= 0 && (deg.face < 0 || d.err < deg.err)) deg = d;
        }
        best[r] = reg.face >= 0 ? reg.face : deg.face;
    }
    return best;
}

// Quantized error: one bucket per power of two (finer buckets give the same quality but far less
// parallelism, coarser ones lose quality); errors below 1e-14 share bucket 0.
constexpr int kBucketCount = 91;

int bucketOf(double e)
{
    if (!(e > 1e-14)) return 0;
    uint64_t bits;
    std::memcpy(&bits, &e, sizeof bits);
    const int ex = int((bits >> 52) & 0x7ff) - 976;   // 2^-47 ~ 7e-15
    return std::max(0, std::min(kBucketCount - 1, ex + 1));
}

// Parallel flooding with a bucket queue (delta-stepping): faces wait in buckets of quantized error; all
// faces of the lowest bucket are claimed at once, each by the cheapest proxy among its labeled neighbors,
// then the neighbors they reach are queued. Equals the sequential priority queue up to the bucket width
// (ties inside a bucket are claimed simultaneously). A face always joins a region it touches, so regions
// stay connected. The result depends neither on the thread count nor on scheduling.
void floodBuckets(const VSAInput& in, VSAState& s)
{
    const TriMesh& m = in.mesh;
    ThreadPool& pool = poolOf(s);
    const size_t nF = m.n_faces();

    std::vector<std::vector<int>> buckets(kBucketCount);
    int lowest = kBucketCount;   // no non-empty bucket below this index
    std::vector<int> candProxy(nF, -1);
    std::vector<uint32_t> stamp(nF, 0);
    uint32_t round = 0;

    struct Entry { int bucket, face; };
    std::vector<std::vector<Entry>> out;
    auto generate = [&](const std::vector<int>& claimed) {
        const size_t chunks = (claimed.size() + 2047) / 2048;
        out.assign(std::max<size_t>(chunks, 1), {});
        forChunks(pool, claimed.size(), 2048, [&](size_t c, size_t b, size_t e) {
            auto& mine = out[c];
            mine.clear();
            for (size_t i = b; i < e; ++i) {
                const int f = claimed[i];
                const int p = s.label[f];
                for (auto nb : m.ff_range(TriMesh::FaceHandle(f))) {
                    const int u = nb.idx();
                    if (s.label[u] >= 0) continue;
                    mine.push_back({bucketOf(metricError(s.metric, in.moments[u], s.proxies[p])), u});
                }
            }
        });
        for (const auto& list : out)
            for (const Entry& en : list) {
                buckets[en.bucket].push_back(en.face);
                lowest = std::min(lowest, en.bucket);
            }
    };

    std::vector<int> seedsLabeled;
    for (size_t f = 0; f < nF; ++f)
        if (s.label[f] >= 0) seedsLabeled.push_back(int(f));
    generate(seedsLabeled);

    std::vector<int> work;
    while (lowest < kBucketCount) {
        if (buckets[lowest].empty()) { ++lowest; continue; }
        work.clear();
        ++round;
        for (int u : buckets[lowest])
            if (s.label[u] < 0 && stamp[u] != round) { stamp[u] = round; work.push_back(u); }
        buckets[lowest].clear();
        if (work.empty()) continue;

        // cheapest labeled neighbor for every face of the batch, then claim them all
        forChunks(pool, work.size(), 1024, [&](size_t, size_t b, size_t e) {
            for (size_t i = b; i < e; ++i) {
                const int u = work[i];
                double be = 0.0;
                int bp = -1;
                for (auto nb : m.ff_range(TriMesh::FaceHandle(u))) {
                    const int p = s.label[nb.idx()];
                    if (p < 0) continue;
                    const double err = metricError(s.metric, in.moments[u], s.proxies[p]);
                    if (bp < 0 || err < be || (err == be && p < bp)) { be = err; bp = p; }
                }
                candProxy[u] = bp;
            }
        });
        forChunks(pool, work.size(), 4096, [&](size_t, size_t b, size_t e) {
            for (size_t i = b; i < e; ++i) s.label[work[i]] = candProxy[work[i]];
        });
        generate(work);
    }
}

} // namespace

void recomputeRegions(const VSAInput& in, VSAState& s)
{
    const size_t k = s.proxies.size();
    constexpr size_t kChunk = 1 << 16;
    const size_t chunks = (s.label.size() + kChunk - 1) / kChunk;
    s.regions.assign(k, Moments());
    if (chunks <= 1) {
        for (size_t f = 0; f < s.label.size(); ++f)
            if (s.label[f] >= 0) s.regions[s.label[f]] += in.moments[f];
        return;
    }
    std::vector<Moments> part(k * chunks);
    forChunks(poolOf(s), s.label.size(), kChunk, [&](size_t c, size_t b, size_t e) {
        Moments* mine = &part[k * c];
        for (size_t f = b; f < e; ++f)
            if (s.label[f] >= 0) mine[s.label[f]] += in.moments[f];
    });
    // merge in chunk order: the sum does not depend on the thread count
    forChunks(poolOf(s), k, 64, [&](size_t, size_t b, size_t e) {
        for (size_t r = b; r < e; ++r)
            for (size_t c = 0; c < chunks; ++c) s.regions[r] += part[k * c + r];
    });
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
    const bool rounds = nF >= s.parallelMinFaces;
    for (int i = 0; i < k0; ++i) {
        if (seeds[i] < 0 || s.label[seeds[i]] >= 0) continue;
        if (rounds) s.label[seeds[i]] = i;
        else claim(in, s, q, seeds[i], i);
    }
    if (rounds) floodBuckets(in, s);
    else runQueue(in, s, q);

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
    smoothLabels(in, s);
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
