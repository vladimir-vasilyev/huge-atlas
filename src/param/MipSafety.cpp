#include "param/MipSafety.h"

#include "param/FlatAdjacency.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace {

struct Sets {
    std::vector<int> parent;
    explicit Sets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int x)
    {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void unite(int a, int b) { parent[find(a)] = find(b); }
};

// does the triangle t (texel coordinates) touch the unit square with corner (x0, y0)? (separating axis test)
bool touches(const double tx[3], const double ty[3], double x0, double y0)
{
    const double eps = 1e-9;
    const double xmin = std::min({tx[0], tx[1], tx[2]}), xmax = std::max({tx[0], tx[1], tx[2]});
    const double ymin = std::min({ty[0], ty[1], ty[2]}), ymax = std::max({ty[0], ty[1], ty[2]});
    if (xmax < x0 - eps || xmin > x0 + 1 + eps || ymax < y0 - eps || ymin > y0 + 1 + eps) return false;
    const double sx[4] = {x0, x0 + 1, x0 + 1, x0}, sy[4] = {y0, y0, y0 + 1, y0 + 1};
    for (int e = 0; e < 3; ++e) {
        const double dx = tx[(e + 1) % 3] - tx[e], dy = ty[(e + 1) % 3] - ty[e];
        const double nx = -dy, ny = dx, len = std::sqrt(nx * nx + ny * ny);
        if (len < 1e-300) continue;
        double tmin = 1e300, tmax = -1e300, smin = 1e300, smax = -1e300;
        for (int k = 0; k < 3; ++k) { const double p = (nx * tx[k] + ny * ty[k]) / len; tmin = std::min(tmin, p); tmax = std::max(tmax, p); }
        for (int k = 0; k < 4; ++k) { const double p = (nx * sx[k] + ny * sy[k]) / len; smin = std::min(smin, p); smax = std::max(smax, p); }
        if (tmax < smin - eps || smax < tmin - eps) return false;
    }
    return true;
}

inline int floorDiv(int a, int b) { return int(std::floor(double(a) / double(b))); }

} // namespace

int mipLevelForGap(int gap)
{
    int best = -1;
    for (int L = 0; L < 20; ++L) {
        const long long reach = (long long)std::ceil(1.5 * double(1LL << L)) - 1;
        if (2 * reach <= gap) best = L; else break;
    }
    return best;
}

MipSafetyResult measureMipSafety(const TriMesh& mesh, const std::vector<float>& uv, int R, bool wrap, int maxLevelToTry,
                                 bool stopAtFirstConflict, int gapLimit)
{
    const FlatMesh flat = flatFromTriMesh(mesh);
    return measureMipSafety(flat, buildFaceNeighbors(flat), uv, R, wrap, maxLevelToTry, stopAtFirstConflict, gapLimit);
}

MipSafetyResult measureMipSafety(const FlatMesh& m, const std::vector<int32_t>& nbr, const std::vector<float>& uv, int R, bool wrap,
                                 int maxLevelToTry, bool stopAtFirstConflict, int gapLimit)
{
    MipSafetyResult res;
    res.resolution = R;
    res.wrap = wrap;
    int top = 0;
    while ((1 << (top + 1)) <= R) ++top;
    res.maxLevel = top;
    const size_t nF = m.faceCount();
    if (R < 1 || uv.size() != nF * 6) return res;
    auto hasUV = [&](size_t f) { return std::isfinite(uv[f * 6]) && std::isfinite(uv[f * 6 + 1]); };

    // charts: faces joined through interior edges whose uv match at both ends
    Sets ds(nF);
    for (size_t f = 0; f < nF; ++f)
        for (int k = 0; k < 3; ++k) {
            const int g = nbr[3 * f + k];
            if (g < 0 || size_t(g) < f || !hasUV(f) || !hasUV(size_t(g))) continue;
            bool same = true;
            for (int slot : {k, (k + 1) % 3}) {
                const uint32_t vi = m.idx[3 * f + slot];
                int slotG = -1;
                for (int j = 0; j < 3; ++j) if (m.idx[3 * size_t(g) + j] == vi) slotG = j;
                if (slotG < 0 || std::abs(double(uv[f * 6 + slot * 2]) - uv[size_t(g) * 6 + slotG * 2]) > 1e-6 ||
                    std::abs(double(uv[f * 6 + slot * 2 + 1]) - uv[size_t(g) * 6 + slotG * 2 + 1]) > 1e-6) { same = false; break; }
            }
            if (same) ds.unite(int(f), g);
        }
    for (size_t f = 0; f < nF; ++f) if (hasUV(f) && ds.find(int(f)) == int(f)) ++res.charts;

    // texels touched by each chart's triangles (-1 none, -2 several charts)
    std::vector<int> owner(size_t(R) * R, -1);
    for (size_t f = 0; f < nF; ++f) {
        if (!hasUV(f)) continue;
        double tx[3], ty[3];
        for (int k = 0; k < 3; ++k) { tx[k] = double(uv[f * 6 + 2 * k]) * R; ty[k] = double(uv[f * 6 + 2 * k + 1]) * R; }
        const int c = ds.find(int(f));
        const int x0 = int(std::floor(std::min({tx[0], tx[1], tx[2]}))) - 1, x1 = int(std::ceil(std::max({tx[0], tx[1], tx[2]}))) + 1;
        const int y0 = int(std::floor(std::min({ty[0], ty[1], ty[2]}))) - 1, y1 = int(std::ceil(std::max({ty[0], ty[1], ty[2]}))) + 1;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                if (!touches(tx, ty, x, y)) continue;
                if (x < 0 || y < 0 || x >= R || y >= R) { ++res.outsideTexels; continue; }
                int& o = owner[size_t(y) * R + x];
                if (o == -1) o = c;
                else if (o != c && o != -2) { o = -2; ++res.bodyOverlapTexels; }
            }
    }

    // level by level: the level-m texels a sample inside a body texel reads (2 x 2, up to 3 x 3 at level 0) belong to one chart only
    res.safeLevel = -1;
    for (int lv = 0; lv <= std::min(top, maxLevelToTry); ++lv) {
        const int S = 1 << lv, N = std::max(1, R / S);
        std::vector<int> own(size_t(N) * N, -1);
        std::vector<char> bad(nF, 0);
        long long conflicts = 0;
        for (int ty = 0; ty < R; ++ty)
            for (int tx = 0; tx < R; ++tx) {
                const int c = owner[size_t(ty) * R + tx];
                if (c == -1) continue;
                const int xa = floorDiv(2 * tx - S, 2 * S), xb = floorDiv(2 * tx + 1 - S, 2 * S) + 1;
                const int ya = floorDiv(2 * ty - S, 2 * S), yb = floorDiv(2 * ty + 1 - S, 2 * S) + 1;
                for (int iy = ya; iy <= yb; ++iy)
                    for (int ix = xa; ix <= xb; ++ix) {
                        int px = ix, py = iy;
                        if (wrap) { px = ((px % N) + N) % N; py = ((py % N) + N) % N; }
                        else { px = std::max(0, std::min(N - 1, px)); py = std::max(0, std::min(N - 1, py)); }
                        int& o = own[size_t(py) * N + px];
                        if (c == -2) { if (o != -2) { if (o >= 0) bad[o] = 1; o = -2; ++conflicts; } continue; }   // a texel of two charts: always a conflict
                        if (o == -1) o = c;
                        else if (o != c) { if (o != -2) { bad[o] = 1; o = -2; ++conflicts; } bad[c] = 1; }
                    }
            }
        res.conflicts.push_back(conflicts);
        res.conflictCharts.push_back(int(std::count(bad.begin(), bad.end(), char(1))));
        if (conflicts > 0 && stopAtFirstConflict) break;
        if (conflicts == 0 && res.safeLevel == lv - 1) res.safeLevel = lv;
    }

    // gaps between facing charts: nearest-chart labels grown over the free texels (8-neighbour BFS, Chebyshev distance)
    if (gapLimit > 0) {
        std::vector<int> label(owner);                  // -1 free, -2 several charts (treated as a chart of its own: gap 0 to anything)
        std::vector<unsigned char> dist(size_t(R) * R, 0);
        std::vector<int> frontier, next;
        for (size_t i = 0; i < label.size(); ++i) if (label[i] != -1) frontier.push_back(int(i));
        for (int d = 1; d <= std::min(gapLimit, 250) && !frontier.empty(); ++d) {
            next.clear();
            for (int i : frontier) {
                const int x = i % R, y = i / R;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= R || ny >= R) continue;
                        const size_t j = size_t(ny) * R + nx;
                        if (label[j] != -1) continue;
                        label[j] = label[i];
                        dist[j] = (unsigned char)d;
                        next.push_back(int(j));
                    }
            }
            frontier.swap(next);
        }
        std::vector<long long> hist(512, 0);
        for (int y = 0; y < R; ++y)
            for (int x = 0; x < R; ++x) {
                const size_t i = size_t(y) * R + x;
                if (label[i] == -1) continue;
                for (int k = 0; k < 2; ++k) {
                    const int nx = x + (k == 0), ny = y + (k == 1);
                    if (nx >= R || ny >= R) continue;
                    const size_t j = size_t(ny) * R + nx;
                    if (label[j] == -1 || label[j] == label[i]) continue;
                    ++hist[std::min<int>(511, dist[i] + dist[j])];
                    ++res.gapPairs;
                }
            }
        auto pct = [&](double q) {
            if (res.gapPairs == 0) return -1;
            long long acc = 0;
            for (int g = 0; g < 512; ++g) { acc += hist[g]; if (double(acc) >= q * double(res.gapPairs)) return g; }
            return 511;
        };
        for (int g = 0; g < 512; ++g) if (hist[g]) { res.gapMin = g; break; }
        res.gapP01 = pct(0.01); res.gapP05 = pct(0.05); res.gapP50 = pct(0.5);
        res.levelByGapP05 = res.gapP05 >= 0 ? mipLevelForGap(res.gapP05) : -1;
        res.levelByGapMin = res.gapMin >= 0 ? mipLevelForGap(res.gapMin) : -1;
        res.levelByGapP01 = res.gapP01 >= 0 ? mipLevelForGap(res.gapP01) : -1;
    }
    return res;
}
