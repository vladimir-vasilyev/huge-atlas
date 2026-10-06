#include "param/HorizonPacker.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
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

Outcome packAt(const std::vector<Chart>& order, double tau, const HorizonOptions& o, int margin)
{
    Outcome out;
    const int R = o.resolution, B = o.atlasBorder;
    const int xBegin = B, xEnd = R - B, yLimit = R - B;
    out.horizon.assign(R, B);
    out.placed.resize(order.size());

    const int rotations = std::max(1, o.rotations);
    for (size_t i = 0; i < order.size(); ++i) {
        Placement bestPl;
        HorizonProfile bestProf;
        double bestTheta = 0.0;
        for (int k = 0; k < rotations; ++k) {
            const double theta = order[i].baseAngle + 2.0 * kPi * k / rotations;
            const HorizonProfile prof = computeProfile(toTexels(order[i], tau, theta), margin);
            if (prof.width() == 0 || prof.width() > xEnd - xBegin) continue;
            const Placement pl = bestPlacement(prof, out.horizon, xBegin, xEnd, yLimit);
            if (!pl.ok) continue;
            if (!bestPl.ok || pl.waste < bestPl.waste || (pl.waste == bestPl.waste && pl.top < bestPl.top)) {
                bestPl = pl;
                bestProf = prof;
                bestTheta = theta;
            }
        }
        if (!bestPl.ok) return out;   // does not fit at this density
        for (int k = 0; k < bestProf.width(); ++k) out.horizon[bestPl.u + k] = bestPl.v + bestProf.hi[k];
        out.placed[i] = {bestTheta, bestPl.u - bestProf.firstColumn, bestPl.v};
    }
    out.fits = true;
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

bool horizonFits(const std::vector<AtlasInput>& inputs, const HorizonOptions& o, double tau)
{
    const std::vector<Chart> order = prepare(inputs, o);
    return packAt(order, tau, o, chartMargin(o.maxMipLevel, o.extraTexels)).fits;
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

    // largest feasible density: halve until the charts fit, then bisect
    double lo = tauHi, hi = tauHi;
    bool feasible = packAt(order, lo, o, margin).fits;
    for (int i = 0; !feasible && i < 12; ++i) {
        hi = lo;
        lo *= 0.5;
        feasible = packAt(order, lo, o, margin).fits;
    }
    if (!feasible) return fail("the charts do not fit: the gutter is too large for this resolution");
    if (lo < hi) {
        for (int i = 0; i < o.searchIterations; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (packAt(order, mid, o, margin).fits) lo = mid;
            else hi = mid;
        }
    }
    const double tau = lo;
    const Outcome fin = packAt(order, tau, o, margin);

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
