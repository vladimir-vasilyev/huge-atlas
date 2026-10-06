#pragma once

// Independent checks of an atlas: exact texel coverage (separating axis test) and the mip-level
// guarantee. Shares no code with the packer or with the mask rasterizer.

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace atlascheck {

inline bool triangleTouchesSquare(const Eigen::Vector2d t[3], double x0, double y0)
{
    const double eps = 1e-9;
    // bounding boxes
    double xmin = std::min({t[0].x(), t[1].x(), t[2].x()}), xmax = std::max({t[0].x(), t[1].x(), t[2].x()});
    double ymin = std::min({t[0].y(), t[1].y(), t[2].y()}), ymax = std::max({t[0].y(), t[1].y(), t[2].y()});
    if (xmax < x0 - eps || xmin > x0 + 1 + eps || ymax < y0 - eps || ymin > y0 + 1 + eps) return false;
    // the three edge normals of the triangle
    const Eigen::Vector2d sq[4] = {{x0, y0}, {x0 + 1, y0}, {x0 + 1, y0 + 1}, {x0, y0 + 1}};
    for (int e = 0; e < 3; ++e) {
        const Eigen::Vector2d d = t[(e + 1) % 3] - t[e];
        const Eigen::Vector2d n(-d.y(), d.x());
        const double len = n.norm();
        if (len < 1e-300) continue;
        double tmin = 1e300, tmax = -1e300, smin = 1e300, smax = -1e300;
        for (int k = 0; k < 3; ++k) { const double p = n.dot(t[k]) / len; tmin = std::min(tmin, p); tmax = std::max(tmax, p); }
        for (int k = 0; k < 4; ++k) { const double p = n.dot(sq[k]) / len; smin = std::min(smin, p); smax = std::max(smax, p); }
        if (tmax < smin - eps || smax < tmin - eps) return false;
    }
    return true;
}

// 1 where the texel (column x, row y) touches a triangle of the chart; uv in [0, 1].
inline std::vector<unsigned char> bodyMask(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv, int R)
{
    std::vector<unsigned char> mask(size_t(R) * R, 0);
    for (auto f : m.faces()) {
        Eigen::Vector2d t[3];
        int k = 0;
        for (auto v : m.fv_range(f)) t[k++] = uv[v.idx()] * double(R);
        const int x0 = std::max(0, int(std::floor(std::min({t[0].x(), t[1].x(), t[2].x()}))) - 1);
        const int x1 = std::min(R - 1, int(std::ceil(std::max({t[0].x(), t[1].x(), t[2].x()}))) + 1);
        const int y0 = std::max(0, int(std::floor(std::min({t[0].y(), t[1].y(), t[2].y()}))) - 1);
        const int y1 = std::min(R - 1, int(std::ceil(std::max({t[0].y(), t[1].y(), t[2].y()}))) + 1);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                if (triangleTouchesSquare(t, x, y)) mask[size_t(y) * R + x] = 1;
    }
    return mask;
}

inline int floorDiv(int a, int b) { return int(std::floor(double(a) / double(b))); }

struct Report {
    int conflicts = 0;     // level-m texels read by samples of two different charts
    int outside = 0;       // chart texels outside the texture
};

// For every level m <= maxLevel: the level-m texels that a bilinear sample inside chart A reads
// (all samples inside its body texels) must not be read by a sample of another chart. With wrap the
// texel indices are taken modulo the level size.
inline Report checkMipGuarantee(const std::vector<std::vector<unsigned char>>& bodies, int R, int maxLevel, bool wrap)
{
    Report rep;
    for (const auto& b : bodies) (void)b;
    for (int m = 0; m <= maxLevel; ++m) {
        const int S = 1 << m, N = R / S;
        std::vector<int> owner(size_t(N) * N, -1);
        for (size_t c = 0; c < bodies.size(); ++c) {
            for (int ty = 0; ty < R; ++ty)
                for (int tx = 0; tx < R; ++tx) {
                    if (!bodies[c][size_t(ty) * R + tx]) continue;
                    const int xa = floorDiv(2 * tx - S, 2 * S), xb = floorDiv(2 * tx + 1 - S, 2 * S) + 1;
                    const int ya = floorDiv(2 * ty - S, 2 * S), yb = floorDiv(2 * ty + 1 - S, 2 * S) + 1;
                    for (int iy = ya; iy <= yb; ++iy)
                        for (int ix = xa; ix <= xb; ++ix) {
                            int cx = ix, cy = iy;
                            if (wrap) { cx = ((ix % N) + N) % N; cy = ((iy % N) + N) % N; }
                            else if (cx < 0 || cy < 0 || cx >= N || cy >= N) continue;   // clamped: the edge texel is the chart's own
                            int& o = owner[size_t(cy) * N + cx];
                            if (o == -1) o = int(c);
                            else if (o != int(c)) { ++rep.conflicts; o = -2; }
                            else if (o == -2) ++rep.conflicts;
                        }
                }
        }
    }
    return rep;
}

} // namespace atlascheck
