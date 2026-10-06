#include "param/AtlasMask.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

constexpr double kEps = 1e-9;

using Poly = std::vector<Eigen::Vector2d>;

// Sutherland-Hodgman against x >= c (keepAbove) or x <= c.
Poly clipX(const Poly& in, double c, bool keepAbove)
{
    Poly out;
    for (size_t i = 0; i < in.size(); ++i) {
        const Eigen::Vector2d& a = in[i];
        const Eigen::Vector2d& b = in[(i + 1) % in.size()];
        const bool ina = keepAbove ? a.x() >= c : a.x() <= c;
        const bool inb = keepAbove ? b.x() >= c : b.x() <= c;
        if (ina) out.push_back(a);
        if (ina != inb) {
            const double t = (c - a.x()) / (b.x() - a.x());
            out.push_back(a + t * (b - a));
        }
    }
    return out;
}

void hueRgb(int index, unsigned char out[3])
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

void putBE32(std::vector<unsigned char>& v, uint32_t x)
{
    v.push_back(x >> 24); v.push_back((x >> 16) & 255); v.push_back((x >> 8) & 255); v.push_back(x & 255);
}

void chunk(std::vector<unsigned char>& file, const char type[4], const std::vector<unsigned char>& data)
{
    putBE32(file, uint32_t(data.size()));
    std::vector<unsigned char> body(type, type + 4);
    body.insert(body.end(), data.begin(), data.end());
    file.insert(file.end(), body.begin(), body.end());
    putBE32(file, uint32_t(crc32(0L, body.data(), uInt(body.size()))));
}

// Marks the texels of one chart; chartId < 0 only marks the mask.
void rasterizeChart(const MaskChart& c, int R, std::vector<unsigned char>& mask, std::vector<int>* owner, int chartId)
{
    if (!c.mesh || !c.uv || c.uv->size() != c.mesh->n_vertices()) return;
    for (auto f : c.mesh->faces()) {
        Poly tri;
        for (auto v : c.mesh->fv_range(f)) tri.push_back((*c.uv)[v.idx()] * double(R));
        double xmin = 1e300, xmax = -1e300;
        for (const auto& p : tri) { xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x()); }
        const int j0 = std::max(0, int(std::floor(xmin - kEps)));
        const int j1 = std::min(R - 1, int(std::ceil(xmax + kEps)) - 1);
        for (int j = j0; j <= j1; ++j) {
            Poly poly = clipX(clipX(tri, double(j) - kEps, true), double(j + 1) + kEps, false);
            if (poly.empty()) continue;
            double ymin = 1e300, ymax = -1e300;
            for (const auto& p : poly) { ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y()); }
            const int r0 = std::max(0, int(std::floor(ymin - kEps)));
            const int r1 = std::min(R - 1, int(std::ceil(ymax + kEps)) - 1);
            for (int r = r0; r <= r1; ++r) {
                mask[size_t(r) * R + j] = 255;
                if (owner) (*owner)[size_t(r) * R + j] = chartId;
            }
        }
    }
}

} // namespace

std::vector<unsigned char> rasterizeCoverage(const std::vector<MaskChart>& charts, int R)
{
    std::vector<unsigned char> mask(size_t(R) * R, 0);
    for (const MaskChart& c : charts) rasterizeChart(c, R, mask, nullptr, -1);
    return mask;
}

bool writePNG(const std::string& path, int w, int h, const std::vector<unsigned char>& rgba, std::string* error)
{
    auto fail = [&](const char* msg) { if (error) *error = msg; return false; };
    if (w <= 0 || h <= 0 || rgba.size() != size_t(w) * h * 4) return fail("bad image size");

    std::vector<unsigned char> raw;
    raw.reserve(size_t(h) * (1 + size_t(w) * 4));
    for (int y = 0; y < h; ++y) {
        raw.push_back(0);   // filter: none
        raw.insert(raw.end(), rgba.begin() + size_t(y) * w * 4, rgba.begin() + size_t(y + 1) * w * 4);
    }
    uLongf cap = compressBound(uLong(raw.size()));
    std::vector<unsigned char> packed(cap);
    if (compress2(packed.data(), &cap, raw.data(), uLong(raw.size()), 6) != Z_OK) return fail("compression failed");
    packed.resize(cap);

    std::vector<unsigned char> file = {137, 80, 78, 71, 13, 10, 26, 10};
    std::vector<unsigned char> ihdr;
    putBE32(ihdr, uint32_t(w));
    putBE32(ihdr, uint32_t(h));
    ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});   // 8 bit, RGBA, deflate, no filter method, no interlace
    chunk(file, "IHDR", ihdr);
    chunk(file, "IDAT", packed);
    chunk(file, "IEND", {});

    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return fail("cannot open the file");
    const bool ok = std::fwrite(file.data(), 1, file.size(), f) == file.size();
    std::fclose(f);
    return ok ? true : fail("write failed");
}

bool writeAtlasPNG(const std::string& path, const std::vector<MaskChart>& charts, int R, bool flipV, std::string* error)
{
    std::vector<unsigned char> mask(size_t(R) * R, 0);
    std::vector<int> owner(size_t(R) * R, -1);
    for (size_t i = 0; i < charts.size(); ++i) rasterizeChart(charts[i], R, mask, &owner, int(i));

    std::vector<unsigned char> rgba(size_t(R) * R * 4, 0);
    for (int row = 0; row < R; ++row) {
        const int v = flipV ? R - 1 - row : row;   // atlas row for this image row
        for (int x = 0; x < R; ++x) {
            unsigned char* px = &rgba[(size_t(row) * R + x) * 4];
            const size_t at = size_t(v) * R + x;
            if (mask[at]) hueRgb(owner[at], px);
            px[3] = mask[at];
        }
    }
    return writePNG(path, R, R, rgba, error);
}
