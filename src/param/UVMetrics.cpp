#include "param/UVMetrics.h"

#include "param/FlatAdjacency.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace {

struct DisjointSets {
    std::vector<int> parent;
    explicit DisjointSets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int x)
    {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void unite(int a, int b) { parent[find(a)] = find(b); }
};

struct Weighted { double value, weight; };

double weightedPercentile(std::vector<Weighted>& v, double q)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end(), [](const Weighted& a, const Weighted& b) { return a.value < b.value; });
    double total = 0.0;
    for (const auto& w : v) total += w.weight;
    double acc = 0.0;
    for (const auto& w : v) {
        acc += w.weight;
        if (acc >= q * total) return w.value;
    }
    return v.back().value;
}

} // namespace

UVLayoutStats analyzeUVLayout(const TriMesh& mesh, const std::vector<float>& uv, int R)
{
    const FlatMesh flat = flatFromTriMesh(mesh);
    return analyzeUVLayout(flat, buildFaceNeighbors(flat), uv, R);
}

UVLayoutStats analyzeUVLayout(const FlatMesh& m, const std::vector<int32_t>& nbr, const std::vector<float>& uv, int R)
{
    UVLayoutStats st;
    const size_t nFaces = m.faceCount();
    auto point = [&](size_t f, int k) {
        const float* q = &m.pos[3 * size_t(m.idx[3 * f + k])];
        return Vec3(q[0], q[1], q[2]);
    };
    st.faces = nFaces;
    st.resolution = R;
    if (uv.size() != nFaces * 6) { st.facesWithoutUV = st.faces; return st; }

    auto hasUV = [&](size_t f) {
        for (int k = 0; k < 6; ++k) if (std::isnan(uv[f * 6 + k])) return false;
        return true;
    };

    // the orientation of the layout: a layout that is mirrored as a whole (negative total signed area) is
    // measured as its mirror image, and only the faces against the majority count as flipped
    double signedTotal = 0.0;
    for (size_t f = 0; f < nFaces; ++f) {
        if (!hasUV(f)) continue;
        const float* q = &uv[f * 6];
        signedTotal += 0.5 * (double(q[2] - q[0]) * double(q[5] - q[1]) - double(q[3] - q[1]) * double(q[4] - q[0]));
    }
    st.mirrored = signedTotal < 0.0;
    const double sgn = st.mirrored ? -1.0 : 1.0;

    // areas and the global scale
    std::vector<double> area3(nFaces, 0.0), areaUV(nFaces, 0.0);
    double total3 = 0.0, totalUV = 0.0;
    for (size_t f = 0; f < nFaces; ++f) {
        Vec3 p[3];
        for (int k = 0; k < 3; ++k) p[k] = point(f, k);
        area3[f] = 0.5 * ((p[1] - p[0]) % (p[2] - p[0])).norm();
        if (!hasUV(f)) { ++st.facesWithoutUV; continue; }
        const float* q = &uv[f * 6];
        areaUV[f] = sgn * 0.5 * (double(q[2] - q[0]) * double(q[5] - q[1]) - double(q[3] - q[1]) * double(q[4] - q[0]));
        if (areaUV[f] <= 0.0) ++st.flipped;
        total3 += area3[f];
        totalUV += std::abs(areaUV[f]);
    }
    const double scale = (totalUV > 0.0 && total3 > 0.0) ? std::sqrt(total3 / totalUV) : 1.0;

    // distortion per face
    std::vector<Weighted> l2, conf, density;
    double l2Sq = 0.0, wSum = 0.0;
    for (size_t i = 0; i < nFaces; ++i) {
        if (!hasUV(i) || area3[i] <= 0.0 || areaUV[i] <= 0.0) continue;
        Vec3 p[3];
        for (int k = 0; k < 3; ++k) p[k] = point(i, k);
        const Vec3 e1 = p[1] - p[0], e2 = p[2] - p[0];
        const double l1 = e1.norm(), cross = (e1 % e2).norm();
        if (l1 <= 0.0 || cross <= 0.0) continue;
        const double x2 = (e1 | e2) / l1, y2 = cross / l1;
        const float* q = &uv[i * 6];
        // a mirrored layout is measured as its mirror image (u -> -u)
        const double d1x = sgn * double(q[2] - q[0]) * scale, d1y = double(q[3] - q[1]) * scale;
        const double d2x = sgn * double(q[4] - q[0]) * scale, d2y = double(q[5] - q[1]) * scale;
        const double ax = d1x / l1, ay = d1y / l1;                                   // first column of J
        const double bx = (-x2 * d1x + l1 * d2x) / (l1 * y2), by = (-x2 * d1y + l1 * d2y) / (l1 * y2);   // second column
        const double a = ax, c = ay, b = bx, d = by;
        const double E = 0.5 * (a + d), F = 0.5 * (a - d), G = 0.5 * (c + b), H = 0.5 * (c - b);
        const double Q = std::hypot(E, H), Rr = std::hypot(F, G);
        const double s1 = Q + Rr, s2 = Q - Rr;
        if (s2 <= 1e-12) continue;
        const double l2v = std::sqrt(0.5 * (s1 * s1 + s2 * s2));
        l2.push_back({l2v, area3[i]});
        conf.push_back({s1 / s2, area3[i]});
        density.push_back({s1 * s2, area3[i]});
        l2Sq += area3[i] * l2v * l2v;
        wSum += area3[i];
        st.l2Max = std::max(st.l2Max, l2v);
    }
    if (wSum > 0.0) st.l2Mean = std::sqrt(l2Sq / wSum);
    st.l2P50 = weightedPercentile(l2, 0.50);
    st.l2P95 = weightedPercentile(l2, 0.95);
    st.l2P99 = weightedPercentile(l2, 0.99);
    st.conformalP50 = weightedPercentile(conf, 0.50);
    st.conformalP95 = weightedPercentile(conf, 0.95);
    st.densityP05 = weightedPercentile(density, 0.05);
    st.densityP50 = weightedPercentile(density, 0.50);
    st.densityP95 = weightedPercentile(density, 0.95);

    // charts and seams
    DisjointSets ds(nFaces);
    for (size_t f = 0; f < nFaces; ++f)
        for (int k = 0; k < 3; ++k) {
            const int g = nbr[3 * f + k];
            if (g < 0 || size_t(g) < f) continue;   // border / non-manifold edge, or seen from the other face
            const uint32_t va = m.idx[3 * f + k], vb = m.idx[3 * f + (k + 1) % 3];
            const double len = (point(f, k) - point(f, (k + 1) % 3)).norm();
            st.edgeLength += len;
            bool same = hasUV(f) && hasUV(size_t(g));
            if (same)
                for (int slot : {k, (k + 1) % 3}) {
                    const uint32_t vi = m.idx[3 * f + slot];
                    int slotG = -1;
                    for (int j = 0; j < 3; ++j) if (m.idx[3 * size_t(g) + j] == vi) slotG = j;
                    if (slotG < 0 || std::abs(double(uv[f * 6 + slot * 2]) - uv[size_t(g) * 6 + slotG * 2]) > 1e-6 ||
                        std::abs(double(uv[f * 6 + slot * 2 + 1]) - uv[size_t(g) * 6 + slotG * 2 + 1]) > 1e-6) { same = false; break; }
                }
            (void)va; (void)vb;
            if (same) ds.unite(int(f), g);
            else st.seamLength += len;
        }
    for (size_t f = 0; f < nFaces; ++f)
        if (hasUV(f) && ds.find(int(f)) == int(f)) ++st.charts;

    // texture coverage by the texel centers
    if (R > 0) {
        std::vector<unsigned char> count(size_t(R) * R, 0);
        for (size_t f = 0; f < nFaces; ++f) {
            if (!hasUV(f)) continue;
            const float* q = &uv[f * 6];
            double tx[3], ty[3];
            for (int k = 0; k < 3; ++k) { tx[k] = double(q[k * 2]) * R; ty[k] = double(q[k * 2 + 1]) * R; }
            const double ar = 0.5 * ((tx[1] - tx[0]) * (ty[2] - ty[0]) - (ty[1] - ty[0]) * (tx[2] - tx[0]));
            if (std::abs(ar) < 1e-12) continue;
            const int x0 = std::max(0, int(std::floor(std::min({tx[0], tx[1], tx[2]})))),
                      x1 = std::min(R - 1, int(std::ceil(std::max({tx[0], tx[1], tx[2]}))));
            const int y0 = std::max(0, int(std::floor(std::min({ty[0], ty[1], ty[2]})))),
                      y1 = std::min(R - 1, int(std::ceil(std::max({ty[0], ty[1], ty[2]}))));
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const double px = x + 0.5 + 1e-7, py = y + 0.5 + 2.3e-7;   // never exactly on an edge: no texel counts twice by accident
                    const double b0 = 0.5 * ((tx[1] - px) * (ty[2] - py) - (ty[1] - py) * (tx[2] - px)) / ar;
                    const double b1 = 0.5 * ((tx[2] - px) * (ty[0] - py) - (ty[2] - py) * (tx[0] - px)) / ar;
                    const double b2 = 1.0 - b0 - b1;
                    if (b0 < 0.0 || b1 < 0.0 || b2 < 0.0) continue;
                    unsigned char& c = count[size_t(y) * R + x];
                    if (c < 255) ++c;
                }
        }
        size_t covered = 0, overlapped = 0;
        for (unsigned char c : count) { covered += c >= 1; overlapped += c >= 2; }
        st.covered = double(covered) / (double(R) * R);
        st.overlap = covered ? double(overlapped) / double(covered) : 0.0;
    }
    return st;
}
