#include "param/HorizonPacker.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

Eigen::Vector2d rotated(const Eigen::Vector2d& p, double a)
{
    const double c = std::cos(a), s = std::sin(a);
    return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

bool isPowerOfTwo(int v) { return v > 0 && (v & (v - 1)) == 0; }

int log2i(int v)
{
    int l = 0;
    while ((1 << (l + 1)) <= v) ++l;
    return l;
}

// A chart prepared for packing: boundary in "3D area units" (UV scaled so that its area equals the 3D area).
struct Chart {
    int    input = -1;
    double scale = 1.0;          // UV -> 3D-area units
    double area3D = 0.0;
    double diameter = 0.0;
    double baseAngle = 0.0;      // rotation that puts the longest diameter vertical
    std::vector<Segment2> boundary;
};

bool validInput(const AtlasInput& a)
{
    return a.uv && !a.uv->empty() && a.uvArea > 0.0 && a.area3D > 0.0;
}

std::vector<Segment2> boundaryOf(const AtlasInput& a, double s)
{
    std::vector<Segment2> segs;
    if (a.boundaryEdges) {
        for (const auto& e : *a.boundaryEdges) segs.push_back({(*a.uv)[e.first] * s, (*a.uv)[e.second] * s});
        if (!segs.empty()) return segs;
    }
    if (a.mesh && a.mesh->n_vertices() == a.uv->size()) {
        for (auto h : a.mesh->halfedges()) {
            if (!a.mesh->is_boundary(h)) continue;
            segs.push_back({(*a.uv)[a.mesh->from_vertex_handle(h).idx()] * s, (*a.uv)[a.mesh->to_vertex_handle(h).idx()] * s});
        }
        if (!segs.empty()) return segs;
    }
    // no mesh: the convex hull is a conservative stand-in for the boundary
    std::vector<Eigen::Vector2d> pts = *a.uv;
    for (auto& p : pts) p *= s;
    const std::vector<Eigen::Vector2d> hull = convexHull(pts);
    for (size_t i = 0; i < hull.size(); ++i) segs.push_back({hull[i], hull[(i + 1) % hull.size()]});
    if (hull.size() == 1) segs.push_back({hull[0], hull[0]});
    return segs;
}

// The pool for the hole search: the shared one, or a private one of the requested size.
ThreadPool* poolFor(const HorizonOptions& o, std::unique_ptr<ThreadPool>& own)
{
    if (!o.fillHoles || o.threads == 1) return nullptr;
    if (o.threads <= 0) return &ThreadPool::shared();
    own = std::make_unique<ThreadPool>(o.threads);
    return own.get();
}

std::vector<Chart> prepare(const std::vector<AtlasInput>& in, const HorizonOptions& opt)
{
    std::vector<Chart> charts;
    for (size_t i = 0; i < in.size(); ++i) {
        if (!validInput(in[i])) continue;
        Chart c;
        c.input = int(i);
        c.scale = std::sqrt(in[i].area3D / in[i].uvArea);
        c.area3D = in[i].area3D;
        c.boundary = boundaryOf(in[i], c.scale);

        std::vector<Eigen::Vector2d> pts;
        for (const auto& sg : c.boundary) { pts.push_back(sg.first); pts.push_back(sg.second); }
        const std::vector<Eigen::Vector2d> hull = convexHull(pts);
        Eigen::Vector2d d(0, 1);
        double best = -1.0;
        for (size_t a = 0; a < hull.size(); ++a)
            for (size_t b = a + 1; b < hull.size(); ++b)
                if ((hull[b] - hull[a]).squaredNorm() > best) { best = (hull[b] - hull[a]).squaredNorm(); d = hull[b] - hull[a]; }
        c.diameter = best > 0 ? std::sqrt(best) : 0.0;
        c.baseAngle = best > 0 ? 0.5 * kPi - std::atan2(d.y(), d.x()) : 0.0;
        charts.push_back(std::move(c));
    }
    std::stable_sort(charts.begin(), charts.end(), [&](const Chart& a, const Chart& b) {
        return opt.sortByDiameter ? a.diameter > b.diameter : a.area3D > b.area3D;
    });
    return charts;
}

struct Placed {
    double theta = 0.0;
    int    tx = 0, ty = 0;   // integer translation in texels, applied after scaling and rotation
};

struct Outcome {
    bool fits = false;
    std::vector<Placed> placed;   // parallel to the chart order
    std::vector<int> horizon;
};

std::vector<Segment2> toTexels(const Chart& c, double tau, double theta)
{
    std::vector<Segment2> out;
    out.reserve(c.boundary.size());
    for (const auto& s : c.boundary) out.push_back({tau * rotated(s.first, theta), tau * rotated(s.second, theta)});
    return out;
}

} // namespace

int mipReach(int level)
{
    const int s = 1 << std::max(0, level);
    return (3 * s + 1) / 2 - 1;   // ceil(1.5 * 2^level) - 1
}

int chartMargin(int maxMipLevel, int extraTexels) { return mipReach(maxMipLevel) + std::max(0, extraTexels); }
int chartGutter(int maxMipLevel, int extraTexels) { return 2 * chartMargin(maxMipLevel, extraTexels); }

HorizonProfile computeProfile(const std::vector<Segment2>& boundary, int dilation)
{
    HorizonProfile prof;
    if (boundary.empty()) return prof;
    dilation = std::max(0, dilation);

    double xmin = 1e300, xmax = -1e300;
    for (const auto& s : boundary) {
        xmin = std::min({xmin, s.first.x(), s.second.x()});
        xmax = std::max({xmax, s.first.x(), s.second.x()});
    }
    const int c0 = int(std::floor(xmin - kEps));
    const int c1 = std::max(c0, int(std::ceil(xmax + kEps)) - 1);
    const int nb = c1 - c0 + 1;

    std::vector<double> bmin(nb, std::numeric_limits<double>::infinity()), bmax(nb, -std::numeric_limits<double>::infinity());
    for (const auto& sg : boundary) {
        Eigen::Vector2d a = sg.first, b = sg.second;
        if (a.x() > b.x()) std::swap(a, b);
        const int jlo = std::max(c0, int(std::floor(a.x() - kEps)));
        const int jhi = std::min(c1, std::max(jlo, int(std::ceil(b.x() + kEps)) - 1));
        const double dx = b.x() - a.x();
        for (int j = jlo; j <= jhi; ++j) {
            const double x0 = std::max(a.x(), double(j)), x1 = std::min(b.x(), double(j + 1));
            double ya, yb;
            if (dx > 1e-12 && x0 <= x1) {
                ya = a.y() + (x0 - a.x()) / dx * (b.y() - a.y());
                yb = a.y() + (x1 - a.x()) / dx * (b.y() - a.y());
            } else {   // vertical segment, or one that only touches the column
                ya = std::min(a.y(), b.y());
                yb = std::max(a.y(), b.y());
            }
            bmin[j - c0] = std::min({bmin[j - c0], ya, yb});
            bmax[j - c0] = std::max({bmax[j - c0], ya, yb});
        }
    }
    // a column no segment reached (cannot happen for a closed chart, but be safe): copy a neighbour
    for (int j = 1; j < nb; ++j)
        if (bmin[j] > bmax[j]) { bmin[j] = bmin[j - 1]; bmax[j] = bmax[j - 1]; }
    for (int j = nb - 2; j >= 0; --j)
        if (bmin[j] > bmax[j]) { bmin[j] = bmin[j + 1]; bmax[j] = bmax[j + 1]; }

    std::vector<int> lo(nb), hi(nb);
    for (int j = 0; j < nb; ++j) {
        lo[j] = int(std::floor(bmin[j] - kEps));
        hi[j] = int(std::ceil(bmax[j] + kEps));
        if (hi[j] <= lo[j]) hi[j] = lo[j] + 1;
    }

    // square dilation: window min of lo and window max of hi, then grow by `dilation` rows
    const int W = nb + 2 * dilation;
    prof.firstColumn = c0 - dilation;
    prof.lo.assign(W, 0);
    prof.hi.assign(W, 0);
    std::deque<int> dqMin, dqMax;   // body column indices, monotone
    int next = c0;                  // next body column (absolute index) to enter the window
    for (int k = 0; k < W; ++k) {
        const int c = c0 - dilation + k;
        const int wl = std::max(c0, c - dilation), wr = std::min(c1, c + dilation);
        for (; next <= wr; ++next) {
            while (!dqMin.empty() && lo[dqMin.back() - c0] >= lo[next - c0]) dqMin.pop_back();
            dqMin.push_back(next);
            while (!dqMax.empty() && hi[dqMax.back() - c0] <= hi[next - c0]) dqMax.pop_back();
            dqMax.push_back(next);
        }
        while (!dqMin.empty() && dqMin.front() < wl) dqMin.pop_front();
        while (!dqMax.empty() && dqMax.front() < wl) dqMax.pop_front();
        prof.lo[k] = lo[dqMin.front() - c0] - dilation;
        prof.hi[k] = hi[dqMax.front() - c0] + dilation;
    }
    return prof;
}

Placement bestPlacement(const HorizonProfile& p, const std::vector<int>& h, int xBegin, int xEnd, int yLimit)
{
    Placement best;
    const int W = p.width();
    if (W == 0 || xEnd - xBegin < W) return best;
    const int maxHi = *std::max_element(p.hi.begin(), p.hi.end());
    for (int u = xBegin; u + W <= xEnd; ++u) {
        int v = std::numeric_limits<int>::min();
        for (int k = 0; k < W; ++k) v = std::max(v, h[u + k] - p.lo[k]);
        const int top = v + maxHi;
        if (top > yLimit) continue;
        long long waste = 0;
        for (int k = 0; k < W; ++k) {
            waste += v + p.lo[k] - h[u + k];
            if (best.ok && waste > best.waste) break;   // already worse than the best so far
        }
        if (!best.ok || waste < best.waste || (waste == best.waste && top < best.top)) {
            best.ok = true;
            best.u = u;
            best.v = v;
            best.waste = waste;
            best.top = top;
        }
    }
    return best;
}

void HorizonOccupancy::add(int x, int lo, int hi)
{
    auto& c = cols_[x];
    auto it = std::lower_bound(c.begin(), c.end(), std::make_pair(lo, hi));
    c.insert(it, {lo, hi});
    int prev = floor_, gap = 0;   // the tallest gap under the horizon
    for (const auto& iv : c) {
        gap = std::max(gap, iv.first - prev);
        prev = std::max(prev, iv.second);
    }
    maxGap_[x] = gap;
}

namespace {

using Range = std::pair<int, int>;   // closed range of feasible vertical shifts

// feasible shifts v of one profile column (rows [lo, hi)) in the gaps of an atlas column
void feasibleShifts(const std::vector<std::pair<int, int>>& occupied, int floor, int lo, int hi, std::vector<Range>& out)
{
    out.clear();
    const int height = hi - lo;
    int prev = floor;
    auto gap = [&](int a, int b) {
        if (b - a >= height) out.push_back({a - lo, b - hi});   // lo + v >= a and hi + v <= b
    };
    for (const auto& iv : occupied) {
        if (iv.first > prev) gap(prev, iv.first);
        prev = std::max(prev, iv.second);
    }
    // above the last interval is the open space over the horizon: not a hole
}

// out = a intersect b (both sorted, disjoint)
void intersect(const std::vector<Range>& a, const std::vector<Range>& b, std::vector<Range>& out)
{
    out.clear();
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        const int lo = std::max(a[i].first, b[j].first), hi = std::min(a[i].second, b[j].second);
        if (lo <= hi) out.push_back({lo, hi});
        if (a[i].second < b[j].second) ++i; else ++j;
    }
}

} // namespace

namespace {

struct HoleScratch { std::vector<Range> feasible, column, next; };

// The best hole position among u in [uBegin, uEnd] (inclusive); lowest v, then lowest u.
Placement searchHoles(const HorizonProfile& p, const HorizonOccupancy& occ, int uBegin, int uEnd, int tallest,
                      int tallestHeight, int maxHi, HoleScratch& sc, const std::vector<char>* allowed = nullptr)
{
    Placement best;
    const int W = p.width();
    for (int u = uBegin; u <= uEnd; ++u) {
        if (allowed && !(*allowed)[u]) continue;
        if (occ.maxGap(u + tallest) < tallestHeight || occ.maxGap(u) < p.hi[0] - p.lo[0] ||
            occ.maxGap(u + W - 1) < p.hi[W - 1] - p.lo[W - 1])
            continue;
        feasibleShifts(occ.column(u), occ.floor(), p.lo[0], p.hi[0], sc.feasible);
        for (int k = 1; k < W && !sc.feasible.empty(); ++k) {
            feasibleShifts(occ.column(u + k), occ.floor(), p.lo[k], p.hi[k], sc.column);
            intersect(sc.feasible, sc.column, sc.next);
            sc.feasible.swap(sc.next);
        }
        if (sc.feasible.empty()) continue;
        const int v = sc.feasible.front().first;                  // the lowest feasible shift
        if (!best.ok || v < best.v) {                             // u ascends: ties keep the smaller u
            best.ok = true;
            best.u = u;
            best.v = v;
            best.top = v + maxHi;
            best.waste = 0;
        }
    }
    return best;
}

} // namespace

namespace {

// Everything about a profile that the hole search needs, and whether a search can succeed at all.
struct HoleQuery {
    const HorizonProfile* profile = nullptr;
    int tallest = 0, tallestHeight = 0, maxHi = 0;
    int uBegin = 0, uLast = -1;
    bool viable = false;
    std::vector<char> allowed;   // per position: some free region can hold the whole profile (empty = no filter)
};

HoleQuery makeHoleQuery(const HorizonProfile& p, const HorizonOccupancy& occ, int xBegin, int xEnd, bool filter = false)
{
    HoleQuery q;
    const int W = p.width();
    if (W == 0 || xEnd - xBegin < W) return q;
    q.profile = &p;
    q.maxHi = *std::max_element(p.hi.begin(), p.hi.end());
    for (int k = 0; k < W; ++k)
        if (p.hi[k] - p.lo[k] > q.tallestHeight) { q.tallestHeight = p.hi[k] - p.lo[k]; q.tallest = k; }
    // cheap rejection: the tallest column of the chart needs a gap at least as tall, somewhere
    bool anyGap = false;
    for (int x = xBegin; x < xEnd && !anyGap; ++x) anyGap = occ.maxGap(x) >= q.tallestHeight;
    q.uBegin = xBegin;
    q.uLast = xEnd - W;
    q.viable = anyGap;
    if (anyGap && filter && q.uLast >= q.uBegin) {
        // Every column of the chart needs a gap at least as tall as the column in the atlas column it lands on: a position
        // where one column finds none is out. One pass of comparisons per chart column over all positions.
        q.allowed.assign(size_t(xEnd), 1);
        for (int k = 0; k < W; ++k) {
            const int need = p.hi[k] - p.lo[k];
            for (int u = q.uBegin; u <= q.uLast; ++u) q.allowed[u] &= char(occ.maxGap(u + k) >= need);
        }
    }
    return q;
}

// Splits [uBegin, uLast] into ranges for the threads; one range when the work is small.
int holeChunks(const HoleQuery& q, int threads)
{
    const int positions = q.uLast - q.uBegin + 1;
    if (threads <= 1 || positions < 256) return 1;
    return std::min(positions / 64, threads * 4);
}

void chunkRange(const HoleQuery& q, int chunks, int c, int& b, int& e)
{
    const int positions = q.uLast - q.uBegin + 1;
    b = q.uBegin + int(int64_t(positions) * c / chunks);
    e = q.uBegin + int(int64_t(positions) * (c + 1) / chunks) - 1;
}

// (v, u) order; the earlier of two equal candidates wins
bool betterHole(const Placement& a, const Placement& b) { return a.ok && (!b.ok || a.v < b.v || (a.v == b.v && a.u < b.u)); }

} // namespace

Placement bestHolePlacement(const HorizonProfile& p, const HorizonOccupancy& occ, int xBegin, int xEnd, ThreadPool* pool)
{
    const HoleQuery q = makeHoleQuery(p, occ, xBegin, xEnd);
    if (!q.viable) return Placement();
    const int chunks = holeChunks(q, pool ? pool->size() : 1);
    if (chunks == 1) {
        HoleScratch sc;
        return searchHoles(p, occ, q.uBegin, q.uLast, q.tallest, q.tallestHeight, q.maxHi, sc);
    }
    // each range of positions is a different set of holes: search them concurrently, merge by (v, u)
    std::vector<Placement> found(chunks);
    pool->parallelFor(chunks, [&](int c) {
        int b, e;
        chunkRange(q, chunks, c, b, e);
        HoleScratch sc;
        found[c] = searchHoles(p, occ, b, e, q.tallest, q.tallestHeight, q.maxHi, sc);
    });
    Placement best;
    for (const Placement& f : found)
        if (betterHole(f, best)) best = f;
    return best;
}

namespace {

Outcome packAt(const std::vector<Chart>& order, double tau, const HorizonOptions& o, int margin, ThreadPool* pool = nullptr)
{
    Outcome out;
    const int R = o.resolution, B = o.atlasBorder;
    const int xBegin = B, xEnd = R - B, yLimit = R - B;
    out.horizon.assign(R, B);
    out.placed.resize(order.size());
    HorizonOccupancy occ(R, B);

    const int rotations = std::max(1, o.rotations);
    const int holeRotations = o.fillHoles ? std::max(0, o.holeRotations) : 0;
    const int candidates = std::max(rotations, holeRotations);
    const int threads = pool ? pool->size() : 1;

    struct Candidate {
        HorizonProfile horizonProfile, holeProfile;
        double thetaH = 0.0, thetaO = 0.0;
        Placement horizonPlacement;
        HoleQuery query;
    };
    for (size_t i = 0; i < order.size(); ++i) {
        // stage 1, per orientation (independent): the profiles and the horizon placement
        std::vector<Candidate> cand(candidates);
        auto stage1 = [&](int k) {
            Candidate& c = cand[k];
            c.thetaH = order[i].baseAngle + 2.0 * kPi * k / rotations;
            c.thetaO = order[i].baseAngle + 2.0 * kPi * k / std::max(1, holeRotations);
            const bool wantHorizon = k < rotations, wantHole = k < holeRotations;
            if (wantHorizon) {
                c.horizonProfile = computeProfile(toTexels(order[i], tau, c.thetaH), margin);
                if (c.horizonProfile.width() > 0 && c.horizonProfile.width() <= xEnd - xBegin)
                    c.horizonPlacement = bestPlacement(c.horizonProfile, out.horizon, xBegin, xEnd, yLimit);
            }
            if (wantHole) {
                // the same orientation: reuse the profile
                c.holeProfile = (wantHorizon && c.thetaH == c.thetaO) ? c.horizonProfile
                                                                       : computeProfile(toTexels(order[i], tau, c.thetaO), margin);
                if (c.holeProfile.width() > 0 && c.holeProfile.width() <= xEnd - xBegin)
                    c.query = makeHoleQuery(c.holeProfile, occ, xBegin, xEnd, o.holeFilter);
            }
        };
        if (pool && candidates > 1) pool->parallelFor(candidates, stage1);
        else for (int k = 0; k < candidates; ++k) stage1(k);

        // the horizon placement: least lost space, then lowest top, earliest orientation
        int bestK = -1;
        for (int k = 0; k < rotations; ++k) {
            const Placement& pl = cand[k].horizonPlacement;
            if (!pl.ok) continue;
            if (bestK < 0 || pl.waste < cand[bestK].horizonPlacement.waste ||
                (pl.waste == cand[bestK].horizonPlacement.waste && pl.top < cand[bestK].horizonPlacement.top))
                bestK = k;
        }

        // stage 2: the holes. Every (orientation, range of positions) is a task: different threads look into different holes
        struct Task { int k, c, chunks; };
        std::vector<Task> tasks;
        for (int k = 0; k < holeRotations; ++k)
            if (cand[k].query.viable) {
                const int chunks = holeChunks(cand[k].query, threads);
                for (int c = 0; c < chunks; ++c) tasks.push_back({k, c, chunks});
            }
        std::vector<Placement> found(tasks.size());
        auto stage2 = [&](int t) {
            const Task& task = tasks[t];
            const HoleQuery& q = cand[task.k].query;
            int b, e;
            chunkRange(q, task.chunks, task.c, b, e);
            HoleScratch sc;
            found[t] = searchHoles(*q.profile, occ, b, e, q.tallest, q.tallestHeight, q.maxHi, sc, q.allowed.empty() ? nullptr : &q.allowed);
        };
        if (pool && tasks.size() > 1) pool->parallelFor(int(tasks.size()), stage2);
        else for (size_t t = 0; t < tasks.size(); ++t) stage2(int(t));

        Placement bestHole;
        int holeK = -1;
        for (size_t t = 0; t < tasks.size(); ++t)    // tasks ascend in (orientation, position): ties keep the earlier
            if (betterHole(found[t], bestHole)) { bestHole = found[t]; holeK = tasks[t].k; }

        // a hole is free space already lost to the horizon: using it never raises the horizon
        const bool useHole = bestHole.ok;
        if (!useHole && bestK < 0) return out;   // does not fit at this density
        const Placement& pl = useHole ? bestHole : cand[bestK].horizonPlacement;
        const HorizonProfile& prof = useHole ? cand[holeK].holeProfile : cand[bestK].horizonProfile;
        for (int k = 0; k < prof.width(); ++k) {
            occ.add(pl.u + k, prof.lo[k] + pl.v, prof.hi[k] + pl.v);
            out.horizon[pl.u + k] = std::max(out.horizon[pl.u + k], pl.v + prof.hi[k]);
        }
        out.placed[i] = {useHole ? cand[holeK].thetaO : cand[bestK].thetaH, pl.u - prof.firstColumn, pl.v};
    }
    out.fits = true;
    return out;
}

} // namespace

bool horizonFits(const std::vector<AtlasInput>& inputs, const HorizonOptions& o, double tau)
{
    const std::vector<Chart> order = prepare(inputs, o);
    std::unique_ptr<ThreadPool> own;
    return packAt(order, tau, o, chartMargin(o.maxMipLevel, o.extraTexels), poolFor(o, own)).fits;
}

AtlasResult packHorizon(const std::vector<AtlasInput>& in, const HorizonOptions& o)
{
    AtlasResult res;
    res.uv.resize(in.size());
    res.rects.assign(in.size(), {0, 0, 0, 0});
    res.resolution = o.resolution;
    res.maxMipLevel = o.maxMipLevel;
    auto fail = [&](const char* msg) { res.ok = false; res.message = msg; return res; };

    if (!isPowerOfTwo(o.resolution) || o.resolution < 16) return fail("resolution must be a power of two (at least 16)");
    if (o.maxMipLevel < 0 || o.maxMipLevel > log2i(o.resolution) - 2) return fail("max mip level is too large for this resolution");
    const int margin = chartMargin(o.maxMipLevel, o.extraTexels);
    res.margin = margin;
    res.gutter = 2 * margin;
    if (2 * margin + 2 * o.atlasBorder >= o.resolution) return fail("gutter does not fit in the texture");

    const std::vector<Chart> order = prepare(in, o);
    if (order.empty()) return fail("nothing to pack");

    double totalArea = 0.0;
    for (const Chart& c : order) totalArea += c.area3D;
    const double usable = double(o.resolution - 2 * o.atlasBorder);
    const double tauHi = usable / std::sqrt(totalArea);   // 100% fill: no density can be higher

    // largest feasible density. Without holes first (fast, and it is a lower bound): halve until the charts
    // fit, then bisect. With holes the density can only go up, so that search is confined to a narrow interval.
    HorizonOptions plain = o;
    plain.fillHoles = false;
    double lo = tauHi, hi = tauHi;
    bool feasible = packAt(order, lo, plain, margin).fits;
    for (int i = 0; !feasible && i < 12; ++i) {
        hi = lo;
        lo *= 0.5;
        feasible = packAt(order, lo, plain, margin).fits;
    }
    if (!feasible) return fail("the charts do not fit: the gutter is too large for this resolution");
    if (lo < hi) {
        for (int i = 0; i < o.searchIterations && (hi - lo) > o.densityTolerance * lo; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (packAt(order, mid, plain, margin).fits) lo = mid;
            else hi = mid;
        }
    }
    std::unique_ptr<ThreadPool> ownPool;
    ThreadPool* pool = poolFor(o, ownPool);
    bool useHoles = false;
    if (o.fillHoles && o.holeRotations > 0 && packAt(order, lo, o, margin, pool).fits) {   // never worse than without holes
        useHoles = true;
        double up = std::min(tauHi, lo * 1.6);
        if (up > lo && packAt(order, up, o, margin, pool).fits) {
            lo = up;
        } else {
            for (int i = 0; i < std::max(4, o.searchIterations - 5) && up > lo && (up - lo) > o.densityTolerance * lo; ++i) {
                const double mid = 0.5 * (lo + up);
                if (packAt(order, mid, o, margin, pool).fits) lo = mid;
                else up = mid;
            }
        }
    }
    const double tau = lo;
    const Outcome fin = packAt(order, tau, useHoles ? o : plain, margin, useHoles ? pool : nullptr);

    const double R = double(o.resolution);
    double used = 0.0;
    for (size_t i = 0; i < order.size(); ++i) {
        const Chart& c = order[i];
        const Placed& pl = fin.placed[i];
        std::vector<Eigen::Vector2d>& out = res.uv[c.input];
        out.reserve(in[c.input].uv->size());
        std::array<double, 4> box = {1e300, 1e300, -1e300, -1e300};
        for (const auto& p : *in[c.input].uv) {
            const Eigen::Vector2d q = tau * rotated(c.scale * p, pl.theta) + Eigen::Vector2d(pl.tx, pl.ty);
            out.push_back(q / R);
            box = {std::min(box[0], q.x() / R), std::min(box[1], q.y() / R), std::max(box[2], q.x() / R), std::max(box[3], q.y() / R)};
        }
        res.rects[c.input] = box;
        used += c.area3D;
    }
    res.texelsPerUnit = tau;
    res.efficiency = used * tau * tau / (R * R);
    res.side = R;
    res.scale = 1.0 / R;
    res.horizon.resize(o.resolution);
    for (int x = 0; x < o.resolution; ++x) res.horizon[x] = float(fin.horizon[x] / R);
    return res;
}
