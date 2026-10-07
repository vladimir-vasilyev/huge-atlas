#include "param/HierarchyMap.h"

#include "param/ThreadPool.h"

#include <algorithm>
#include <cmath>

namespace {

struct V3 {
    double x, y, z;
};
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 load(const FlatMesh& m, uint32_t v) { return {m.pos[3 * size_t(v)], m.pos[3 * size_t(v) + 1], m.pos[3 * size_t(v) + 2]}; }

// Closest point of triangle abc to p (Ericson, Real-Time Collision Detection): barycentrics of the point.
void closestOnTriangle(V3 p, V3 a, V3 b, V3 c, double& u, double& v, double& w)
{
    const V3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return; }
    const V3 bp = p - b;
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return; }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double t = d1 / (d1 - d3);
        u = 1 - t; v = t; w = 0; return;
    }
    const V3 cp = p - c;
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return; }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double t = d2 / (d2 - d6);
        u = 1 - t; v = 0; w = t; return;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        u = 0; v = 1 - t; w = t; return;
    }
    const double denom = 1.0 / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    u = 1 - v - w;
}

} // namespace

TriangleGrid::TriangleGrid(const FlatMesh& mesh, const std::vector<uint32_t>* faces) : mesh_(mesh)
{
    const size_t n = faces ? faces->size() : mesh.faceCount();
    auto faceAt = [&](size_t i) { return faces ? (*faces)[i] : uint32_t(i); };
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    double extentSum = 0.0;
    for (size_t i = 0; i < n; ++i) {
        float tl[3] = {1e30f, 1e30f, 1e30f}, th[3] = {-1e30f, -1e30f, -1e30f};
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c) {
                const float x = mesh.pos[3 * size_t(mesh.idx[3 * size_t(faceAt(i)) + k]) + c];
                tl[c] = std::min(tl[c], x); th[c] = std::max(th[c], x);
            }
        for (int c = 0; c < 3; ++c) { lo[c] = std::min(lo[c], tl[c]); hi[c] = std::max(hi[c], th[c]); }
        extentSum += std::max({th[0] - tl[0], th[1] - tl[1], th[2] - tl[2]});
    }
    for (int c = 0; c < 3; ++c) lo_[c] = n ? lo[c] : 0.f;
    cell_ = n ? float(extentSum / double(n)) : 1.f;
    double ext = 0.0;
    for (int c = 0; c < 3; ++c) ext = std::max(ext, double(hi[c] - lo[c]));
    // keep the grid at a few cells per triangle at most
    while (n && true) {
        double cells = 1.0;
        for (int c = 0; c < 3; ++c) cells *= std::floor(double(hi[c] - lo[c]) / cell_) + 1.0;
        if (cells <= 4.0 * double(n) + 64.0) break;
        cell_ *= 1.5f;
    }
    cell_ = std::max(cell_, 1e-20f);
    for (int c = 0; c < 3; ++c) dim_[c] = n ? int(std::floor((hi[c] - lo[c]) / cell_)) + 1 : 1;
    const size_t cells = size_t(dim_[0]) * dim_[1] * dim_[2];

    auto range = [&](size_t i, int* a, int* b) {
        float tl[3] = {1e30f, 1e30f, 1e30f}, th[3] = {-1e30f, -1e30f, -1e30f};
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c) {
                const float x = mesh.pos[3 * size_t(mesh.idx[3 * size_t(faceAt(i)) + k]) + c];
                tl[c] = std::min(tl[c], x); th[c] = std::max(th[c], x);
            }
        for (int c = 0; c < 3; ++c) {
            a[c] = std::min(dim_[c] - 1, std::max(0, int((tl[c] - lo_[c]) / cell_)));
            b[c] = std::min(dim_[c] - 1, std::max(0, int((th[c] - lo_[c]) / cell_)));
        }
    };
    start_.assign(cells + 1, 0);
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<size_t> fill;
        if (pass == 1) {
            for (size_t c = 0; c < cells; ++c) start_[c + 1] += start_[c];
            items_.resize(start_[cells]);
            fill.assign(start_.begin(), start_.end() - 1);
        }
        for (size_t i = 0; i < n; ++i) {
            int a[3], b[3];
            range(i, a, b);
            for (int z = a[2]; z <= b[2]; ++z)
                for (int y = a[1]; y <= b[1]; ++y)
                    for (int x = a[0]; x <= b[0]; ++x) {
                        const size_t c = (size_t(z) * dim_[1] + y) * dim_[0] + x;
                        if (pass == 0) ++start_[c + 1];
                        else items_[fill[c]++] = faceAt(i);
                    }
        }
    }
}

TriangleGrid::Hit TriangleGrid::closest(const float pf[3]) const
{
    Hit best;
    if (items_.empty()) return best;
    double bestD = 1e300;
    const V3 p{pf[0], pf[1], pf[2]};
    int c0[3];
    for (int c = 0; c < 3; ++c)
        c0[c] = std::min(dim_[c] - 1, std::max(0, int(std::floor((pf[c] - lo_[c]) / cell_))));
    const int maxRing = std::max({dim_[0], dim_[1], dim_[2]});
    for (int r = 0; r <= maxRing; ++r) {
        // cells at Chebyshev distance exactly r from c0
        const int x0 = c0[0] - r, x1 = c0[0] + r, y0 = c0[1] - r, y1 = c0[1] + r, z0 = c0[2] - r, z1 = c0[2] + r;
        for (int z = std::max(0, z0); z <= std::min(dim_[2] - 1, z1); ++z)
            for (int y = std::max(0, y0); y <= std::min(dim_[1] - 1, y1); ++y) {
                const bool edgeZY = (z == z0 || z == z1 || y == y0 || y == y1);
                const int step = edgeZY ? 1 : std::max(1, x1 - x0);   // interior rows: only the two end cells
                for (int x = x0; x <= x1; x += step) {
                    if (x < 0 || x >= dim_[0]) continue;
                    const size_t c = (size_t(z) * dim_[1] + y) * dim_[0] + x;
                    for (size_t i = start_[c]; i < start_[c + 1]; ++i) {
                        const uint32_t f = items_[i];
                        double u, v, w;
                        const V3 a = load(mesh_, mesh_.idx[3 * size_t(f)]), b = load(mesh_, mesh_.idx[3 * size_t(f) + 1]),
                                 cc = load(mesh_, mesh_.idx[3 * size_t(f) + 2]);
                        closestOnTriangle(p, a, b, cc, u, v, w);
                        const V3 q{u * a.x + v * b.x + w * cc.x, u * a.y + v * b.y + w * cc.y, u * a.z + v * b.z + w * cc.z};
                        const V3 d = p - q;
                        const double d2 = dot(d, d);
                        if (d2 < bestD || (d2 == bestD && f < best.face)) {
                            bestD = d2;
                            best = {f, float(u), float(v), float(w), d2};
                        }
                    }
                }
            }
        // everything not yet visited lies outside the scanned block of cells: at least this far from p
        if (best.face != UINT32_MAX) {
            double bound = 1e300;
            for (int c = 0; c < 3; ++c) {
                if (c0[c] - r > 0) bound = std::min(bound, double(pf[c]) - (double(lo_[c]) + double(c0[c] - r) * cell_));
                if (c0[c] + r + 1 < dim_[c]) bound = std::min(bound, double(lo_[c]) + double(c0[c] + r + 1) * cell_ - double(pf[c]));
            }
            if (bound >= 1e300 || (bound > 0 && bestD <= bound * bound)) break;
        }
    }
    return best;
}

FineToCoarse mapVertices(const FlatMesh& fine, const FlatMesh& coarse, ThreadPool* pool)
{
    FineToCoarse m;
    const size_t n = fine.vertexCount();
    m.face.assign(n, UINT32_MAX);
    m.dist.assign(n, 0.f);
    const TriangleGrid grid(coarse);
    ThreadPool& tp = pool ? *pool : ThreadPool::shared();
    const size_t chunk = 4096, chunks = (n + chunk - 1) / chunk;
    tp.parallelFor(int(chunks), [&](int c) {
        const size_t b = size_t(c) * chunk, e = std::min(n, b + chunk);
        for (size_t v = b; v < e; ++v) {
            const TriangleGrid::Hit h = grid.closest(&fine.pos[3 * v]);
            m.face[v] = h.face;
            m.dist[v] = float(std::sqrt(h.dist2));
        }
    });
    return m;
}

std::vector<int> transferLabels(const FlatMesh& fine, const std::vector<int>& coarseLabel, const FineToCoarse& map,
                                ThreadPool* pool)
{
    const size_t nF = fine.faceCount();
    std::vector<int> label(nF, -1);
    ThreadPool& tp = pool ? *pool : ThreadPool::shared();
    const size_t chunk = 1 << 14, chunks = (nF + chunk - 1) / chunk;
    tp.parallelFor(int(chunks), [&](int c) {
        const size_t b = size_t(c) * chunk, e = std::min(nF, b + chunk);
        for (size_t f = b; f < e; ++f) {
            int l[3];
            int nearest = 0;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = fine.idx[3 * f + k];
                l[k] = map.face[v] == UINT32_MAX ? -1 : coarseLabel[map.face[v]];
                if (map.dist[v] < map.dist[fine.idx[3 * f + nearest]]) nearest = k;
            }
            if (l[0] == l[1] || l[0] == l[2]) label[f] = l[0];
            else if (l[1] == l[2]) label[f] = l[1];
            else label[f] = l[nearest];
        }
    });
    return label;
}
