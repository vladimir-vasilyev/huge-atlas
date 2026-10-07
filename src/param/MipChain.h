#pragma once

// RGBA8 mip chain of a source texture with bilinear and trilinear lookups (wrapping), shared by the bakers.

#include "param/AtlasBake.h"

#include <algorithm>
#include <cmath>
#include <vector>

struct MipChain {
    std::vector<std::vector<unsigned char>> level;   // RGBA8
    std::vector<int> w, h;

    explicit MipChain(const BakeSource& src)
    {
        level.emplace_back(src.rgba, src.rgba + size_t(src.width) * src.height * 4);
        w.push_back(src.width);
        h.push_back(src.height);
        while (w.back() > 1 || h.back() > 1) {
            const int pw = w.back(), ph = h.back(), nw = std::max(1, pw / 2), nh = std::max(1, ph / 2);
            std::vector<unsigned char> next(size_t(nw) * nh * 4);
            const std::vector<unsigned char>& prev = level.back();
            for (int y = 0; y < nh; ++y)
                for (int x = 0; x < nw; ++x)
                    for (int c = 0; c < 4; ++c) {
                        int sum = 0, n = 0;
                        for (int dy = 0; dy < 2; ++dy)
                            for (int dx = 0; dx < 2; ++dx) {
                                const int sx = std::min(pw - 1, 2 * x + dx), sy = std::min(ph - 1, 2 * y + dy);
                                sum += prev[(size_t(sy) * pw + sx) * 4 + c];
                                ++n;
                            }
                        next[(size_t(y) * nw + x) * 4 + c] = (unsigned char)((sum + n / 2) / n);
                    }
            level.push_back(std::move(next));
            w.push_back(nw);
            h.push_back(nh);
        }
    }

    // bilinear sample of one level, wrapping at the borders
    void bilinear(int l, double u, double v, double out[3]) const
    {
        const int W = w[l], H = h[l];
        const double x = u * W - 0.5, y = v * H - 0.5;
        const double fx = std::floor(x), fy = std::floor(y);
        const double tx = x - fx, ty = y - fy;
        auto texel = [&](int ix, int iy, int c) {
            ix = ((ix % W) + W) % W;
            iy = ((iy % H) + H) % H;
            return double(level[l][(size_t(iy) * W + ix) * 4 + c]);
        };
        for (int c = 0; c < 3; ++c) {
            const double a = texel(int(fx), int(fy), c), b = texel(int(fx) + 1, int(fy), c);
            const double d = texel(int(fx), int(fy) + 1, c), e = texel(int(fx) + 1, int(fy) + 1, c);
            out[c] = (a * (1 - tx) + b * tx) * (1 - ty) + (d * (1 - tx) + e * tx) * ty;
        }
    }

    // trilinear: lod in levels (0 = full resolution)
    void trilinear(double u, double v, double lod, double out[3]) const
    {
        const int top = int(level.size()) - 1;
        lod = std::max(0.0, std::min(double(top), lod));
        const int l0 = int(std::floor(lod)), l1 = std::min(top, l0 + 1);
        const double f = lod - l0;
        double a[3], b[3];
        bilinear(l0, u, v, a);
        if (f <= 0.0 || l1 == l0) { for (int c = 0; c < 3; ++c) out[c] = a[c]; return; }
        bilinear(l1, u, v, b);
        for (int c = 0; c < 3; ++c) out[c] = a[c] * (1 - f) + b[c] * f;
    }
};
