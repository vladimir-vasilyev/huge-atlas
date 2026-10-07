#include "param/FlatBake.h"

#include "param/MipChain.h"
#include "param/ThreadPool.h"

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

void chartColor(int index, unsigned char out[3])
{
    const float h = std::fmod(float(index) * 0.61803399f, 1.0f) * 6.0f;
    const float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
    const float c = 0.85f, m = 0.1f;
    float r = 0, g = 0, b = 0;
    switch (int(h)) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    out[0] = (unsigned char)std::min(255.0f, (r + m) * 255.0f);
    out[1] = (unsigned char)std::min(255.0f, (g + m) * 255.0f);
    out[2] = (unsigned char)std::min(255.0f, (b + m) * 255.0f);
}

} // namespace

std::vector<unsigned char> bakeAtlasFlat(const FlatMesh& mesh, const FlatResult& result, const std::vector<float>& sourceUV,
                                         const std::vector<uint8_t>& faceMaterial, int textureMaterial, const BakeSource* source,
                                         const BakeOptions& opt)
{
    const int R = opt.resolution;
    const size_t texels = size_t(R) * R;
    const size_t nF = mesh.faceCount();
    std::vector<unsigned char> rgba(texels * 4, 0);
    std::vector<unsigned char> filled(texels, 0);
    for (size_t i = 0; i < texels; ++i) {
        rgba[i * 4] = opt.background[0];
        rgba[i * 4 + 1] = opt.background[1];
        rgba[i * 4 + 2] = opt.background[2];
    }
    const bool textured = source && source->rgba && source->width > 0 && source->height > 0 && sourceUV.size() == nF * 6;
    std::unique_ptr<MipChain> mips;
    if (textured) mips = std::make_unique<MipChain>(*source);
    ThreadPool& pool = ThreadPool::shared();

    // 1. the texels whose centre is inside an atlas triangle; the patches in parallel, the biggest first
    std::vector<int> order(result.patches.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = int(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return result.patches[a].faces.size() != result.patches[b].faces.size() ? result.patches[a].faces.size() > result.patches[b].faces.size() : a < b;
    });
    pool.parallelFor(int(order.size()), [&](int oi) {
        const int ci = order[oi];
        unsigned char flat[3];
        chartColor(ci, flat);
        for (uint32_t g : result.patches[ci].faces) {
            Eigen::Vector2d T[3], S[3];
            bool ok = true;
            for (int k = 0; k < 3; ++k) {
                const float u = result.cornerUV[6 * size_t(g) + 2 * k], v = result.cornerUV[6 * size_t(g) + 2 * k + 1];
                if (!(u == u) || !(v == v)) { ok = false; break; }
                T[k] = Eigen::Vector2d(u, v) * double(R);
            }
            if (!ok) continue;
            const bool useTexture = textured && (faceMaterial.empty() || faceMaterial[g] == textureMaterial);
            bool haveSource = useTexture;
            if (useTexture)
                for (int k = 0; k < 3; ++k) {
                    const float u = sourceUV[6 * size_t(g) + 2 * k], v = sourceUV[6 * size_t(g) + 2 * k + 1];
                    if (!(u == u) || !(v == v)) { haveSource = false; break; }
                    S[k] = Eigen::Vector2d(u, v);
                }
            const double area = 0.5 * ((T[1] - T[0]).x() * (T[2] - T[0]).y() - (T[1] - T[0]).y() * (T[2] - T[0]).x());
            if (std::abs(area) < 1e-12) continue;
            double lod = 0.0;
            if (haveSource) {
                const double sArea = 0.5 * std::abs((S[1] - S[0]).x() * (S[2] - S[0]).y() - (S[1] - S[0]).y() * (S[2] - S[0]).x()) *
                                     double(source->width) * source->height;
                lod = 0.5 * std::log2(std::max(1.0, sArea / std::abs(area)));
            }
            const int x0 = std::max(0, int(std::floor(std::min({T[0].x(), T[1].x(), T[2].x()})))),
                      x1 = std::min(R - 1, int(std::ceil(std::max({T[0].x(), T[1].x(), T[2].x()}))));
            const int y0 = std::max(0, int(std::floor(std::min({T[0].y(), T[1].y(), T[2].y()})))),
                      y1 = std::min(R - 1, int(std::ceil(std::max({T[0].y(), T[1].y(), T[2].y()}))));
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const Eigen::Vector2d p(x + 0.5, y + 0.5);
                    const double b0 = 0.5 * ((T[1] - p).x() * (T[2] - p).y() - (T[1] - p).y() * (T[2] - p).x()) / area;
                    const double b1 = 0.5 * ((T[2] - p).x() * (T[0] - p).y() - (T[2] - p).y() * (T[0] - p).x()) / area;
                    const double b2 = 1.0 - b0 - b1;
                    if (b0 < -1e-9 || b1 < -1e-9 || b2 < -1e-9) continue;
                    unsigned char* px = &rgba[(size_t(y) * R + x) * 4];
                    if (haveSource) {
                        const Eigen::Vector2d s = b0 * S[0] + b1 * S[1] + b2 * S[2];
                        double c[3];
                        mips->trilinear(s.x(), s.y(), lod, c);
                        for (int ch = 0; ch < 3; ++ch) px[ch] = (unsigned char)std::max(0.0, std::min(255.0, c[ch] + 0.5));
                    } else if (!textured) {
                        for (int ch = 0; ch < 3; ++ch) px[ch] = flat[ch];
                    }
                    filled[size_t(y) * R + x] = 1;
                }
        }
    });

    // 2. alpha = the covered texels grown by one (a texel a chart only touches counts)
    std::vector<unsigned char> occupancy(texels, 0);
    pool.parallelFor(R, [&](int y) {
        for (int x = 0; x < R; ++x) {
            bool any = false;
            for (int dy = -1; dy <= 1 && !any; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && ny >= 0 && nx < R && ny < R && filled[size_t(ny) * R + nx]) { any = true; break; }
                }
            occupancy[size_t(y) * R + x] = any ? 255 : 0;
        }
    });

    // 3. extrude the colours outward by margin + 1 texels, axis neighbours first
    static const int kNeighbours[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    std::vector<unsigned char> nextRgba = rgba, nextFilled = filled;
    for (int pass = 0; pass < opt.margin + 1; ++pass) {
        std::vector<char> changed(R, 0);
        pool.parallelFor(R, [&](int y) {
            for (int x = 0; x < R; ++x) {
                const size_t at = size_t(y) * R + x;
                if (filled[at]) continue;
                for (const auto& d : kNeighbours) {
                    const int nx = x + d[0], ny = y + d[1];
                    if (nx < 0 || ny < 0 || nx >= R || ny >= R || !filled[size_t(ny) * R + nx]) continue;
                    const size_t from = size_t(ny) * R + nx;
                    for (int ch = 0; ch < 3; ++ch) nextRgba[at * 4 + ch] = rgba[from * 4 + ch];
                    nextFilled[at] = 1;
                    changed[y] = 1;
                    break;
                }
            }
        });
        bool any = false;
        for (char c : changed) any = any || c;
        if (!any) break;
        rgba.swap(nextRgba);
        filled.swap(nextFilled);
        nextRgba = rgba;        // the buffers stay equal between passes
        nextFilled = filled;
    }
    for (size_t i = 0; i < texels; ++i) rgba[i * 4 + 3] = occupancy[i];
    return rgba;
}
