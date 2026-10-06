#include "param/Injectivity.h"

#include "param/LinABF.h"
#include "param/Predicates.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Seg {
    int halfedge;
    int from, to;               // patch vertices
    double a[2], b[2];
};

bool onSegment(const double p[2], const double q[2], const double r[2])
{
    // r is collinear with p-q: is it within the bounding box?
    return std::min(p[0], q[0]) <= r[0] && r[0] <= std::max(p[0], q[0]) &&
           std::min(p[1], q[1]) <= r[1] && r[1] <= std::max(p[1], q[1]);
}

// Do the closed segments share a point? (Exact.) For segments with a common endpoint only a
// collinear overlap counts.
bool segmentsConflict(const Seg& s, const Seg& t)
{
    const int shared = (s.from == t.from || s.from == t.to) + (s.to == t.from || s.to == t.to);
    if (shared >= 2) return true;   // the same two vertices: a duplicated edge

    const int o1 = orient2dSign(s.a, s.b, t.a), o2 = orient2dSign(s.a, s.b, t.b);
    const int o3 = orient2dSign(t.a, t.b, s.a), o4 = orient2dSign(t.a, t.b, s.b);

    if (shared == 1) {
        // adjacent edges: they conflict only if collinear and pointing the same way from the shared vertex
        if (o1 != 0 || o2 != 0) return false;
        const double* sp = (s.from == t.from || s.from == t.to) ? s.a : s.b;     // shared point
        const double* so = (sp == s.a) ? s.b : s.a;
        const double* to = (t.from == s.from || t.from == s.to) ? t.b : t.a;
        const double d1[2] = {so[0] - sp[0], so[1] - sp[1]}, d2[2] = {to[0] - sp[0], to[1] - sp[1]};
        return d1[0] * d2[0] + d1[1] * d2[1] > 0.0;
    }

    if (o1 * o2 < 0 && o3 * o4 < 0) return true;                        // proper crossing
    if (o1 == 0 && onSegment(s.a, s.b, t.a)) return true;               // touching / collinear overlap
    if (o2 == 0 && onSegment(s.a, s.b, t.b)) return true;
    if (o3 == 0 && onSegment(t.a, t.b, s.a)) return true;
    if (o4 == 0 && onSegment(t.a, t.b, s.b)) return true;
    return false;
}

} // namespace

AngleReport checkAngles(const TriMesh& m, const std::vector<double>& angles, double foldDelta)
{
    AngleReport r;
    for (size_t c = 0; c < angles.size(); ++c)
        if (!(angles[c] > 0.0 && angles[c] < kPi)) r.badCorners.push_back(int(c));

    for (auto v : m.vertices()) {
        if (!m.is_boundary(v)) continue;
        double sum = 0.0;
        for (auto h : m.vih_range(v)) {
            const int c = cornerIndex(m, h);
            if (c >= 0) sum += angles[c];
        }
        if (sum > 2.0 * kPi - foldDelta + 1e-9) {   // tolerance: the active set pins sums exactly at the limit
            r.foldVertices.push_back(v.idx());
            r.foldSums.push_back(sum);
        }
    }
    return r;
}

std::vector<std::pair<int, int>> boundaryCrossings(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv)
{
    std::vector<Seg> segs;
    for (auto h : m.halfedges()) {
        if (!m.is_boundary(h)) continue;
        Seg s;
        s.halfedge = h.idx();
        s.from = m.from_vertex_handle(h).idx();
        s.to = m.to_vertex_handle(h).idx();
        s.a[0] = uv[s.from].x(); s.a[1] = uv[s.from].y();
        s.b[0] = uv[s.to].x();   s.b[1] = uv[s.to].y();
        segs.push_back(s);
    }
    std::vector<std::pair<int, int>> out;
    if (segs.size() < 2) return out;

    // uniform grid over the segment bounding boxes
    double lo[2] = {segs[0].a[0], segs[0].a[1]}, hi[2] = {lo[0], lo[1]}, total = 0.0;
    for (const Seg& s : segs) {
        for (const double* p : {s.a, s.b})
            for (int k = 0; k < 2; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
        total += std::hypot(s.b[0] - s.a[0], s.b[1] - s.a[1]);
    }
    const double cell = std::max(total / double(segs.size()), 1e-12);
    const int nx = std::max(1, std::min(1024, int((hi[0] - lo[0]) / cell) + 1));
    const int ny = std::max(1, std::min(1024, int((hi[1] - lo[1]) / cell) + 1));
    const double sx = nx / std::max(hi[0] - lo[0], 1e-300), sy = ny / std::max(hi[1] - lo[1], 1e-300);
    auto cellOf = [&](double v, double l, double s, int n) { return std::max(0, std::min(n - 1, int((v - l) * s))); };

    std::vector<std::vector<int>> grid(size_t(nx) * ny);
    for (size_t i = 0; i < segs.size(); ++i) {
        const Seg& s = segs[i];
        const int x0 = cellOf(std::min(s.a[0], s.b[0]), lo[0], sx, nx), x1 = cellOf(std::max(s.a[0], s.b[0]), lo[0], sx, nx);
        const int y0 = cellOf(std::min(s.a[1], s.b[1]), lo[1], sy, ny), y1 = cellOf(std::max(s.a[1], s.b[1]), lo[1], sy, ny);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) grid[size_t(y) * nx + x].push_back(int(i));
    }

    std::unordered_set<uint64_t> tested;
    for (const auto& bucket : grid)
        for (size_t i = 0; i < bucket.size(); ++i)
            for (size_t j = i + 1; j < bucket.size(); ++j) {
                const int a = std::min(bucket[i], bucket[j]), b = std::max(bucket[i], bucket[j]);
                if (!tested.insert((uint64_t(a) << 32) | uint64_t(b)).second) continue;
                if (segmentsConflict(segs[a], segs[b]))
                    out.push_back({segs[a].halfedge, segs[b].halfedge});
            }
    std::sort(out.begin(), out.end());
    return out;
}

UVReport checkUV(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv)
{
    UVReport r;
    for (auto f : m.faces()) {
        Eigen::Vector2d p[3];
        for (auto h : m.fh_range(f)) p[cornerIndex(m, h) % 3] = uv[m.to_vertex_handle(h).idx()];
        // exact orientation: a triangle is flipped or degenerate unless it is strictly counter-clockwise
        const double a[2] = {p[0].x(), p[0].y()}, b[2] = {p[1].x(), p[1].y()}, c[2] = {p[2].x(), p[2].y()};
        if (orient2dSign(a, b, c) <= 0) r.flippedFaces.push_back(f.idx());
    }
    r.crossings = boundaryCrossings(m, uv);
    return r;
}
