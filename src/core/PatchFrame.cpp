#include "core/PatchFrame.h"

#include <algorithm>
#include <cmath>

PatchFrame computePatchFrame(const std::vector<PatchTriangle>& tris, size_t maxPoints)
{
    PatchFrame f;
    if (tris.empty()) return f;
    double g[3] = {0, 0, 0}, nsum[3] = {0, 0, 0}, area = 0;
    std::vector<double> triArea(tris.size()), triNormal(tris.size() * 3);
    for (size_t t = 0; t < tris.size(); ++t) {
        const auto& p = tris[t].p;
        const double e1[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]}, e2[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]};
        double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const double twice = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double a = 0.5 * twice;
        triArea[t] = a;
        if (twice > 0) for (int k = 0; k < 3; ++k) n[k] /= twice;
        for (int k = 0; k < 3; ++k) {
            triNormal[3 * t + k] = n[k];
            nsum[k] += a * n[k];
            g[k] += a * (p[0][k] + p[1][k] + p[2][k]) / 3.0;
        }
        area += a;
    }
    if (area <= 0) return f;
    for (double& v : g) v /= area;
    // the triangle whose centre is nearest to the centre of mass: a point that is on the patch even when the patch is curved
    size_t best = 0;
    double bestD = 1e300;
    for (size_t t = 0; t < tris.size(); ++t) {
        const auto& p = tris[t].p;
        double d = 0;
        for (int k = 0; k < 3; ++k) { const double c = (p[0][k] + p[1][k] + p[2][k]) / 3.0 - g[k]; d += c * c; }
        if (d < bestD) { bestD = d; best = t; }
    }
    for (int k = 0; k < 3; ++k) f.centre[k] = (tris[best].p[0][k] + tris[best].p[1][k] + tris[best].p[2][k]) / 3.0;
    const double len = std::sqrt(nsum[0] * nsum[0] + nsum[1] * nsum[1] + nsum[2] * nsum[2]) / area;   // 1 for a flat patch, 0 for a closed ring
    if (len > 0.2) for (int k = 0; k < 3; ++k) f.normal[k] = nsum[k] / (len * area);
    else for (int k = 0; k < 3; ++k) f.normal[k] = triNormal[3 * best + k];
    f.area = area;
    const size_t stride = std::max<size_t>(1, tris.size() * 3 / std::max<size_t>(1, maxPoints));
    for (size_t t = 0; t < tris.size(); t += stride)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 3; ++k) f.points.push_back(float(tris[t].p[c][k]));
    f.ok = true;
    return f;
}
