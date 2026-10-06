#include "param/AtlasBake.h"

#include "param/AtlasMask.h"

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

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

} // namespace

std::vector<unsigned char> bakeAtlasTexture(const TriMesh& src, const std::vector<BakeChart>& charts, const BakeSource* source,
                                            const std::vector<float>& cornerUV, const std::vector<int>& faceMaterial,
                                            int textureMaterial, const BakeOptions& opt)
{
    const int R = opt.resolution;
    std::vector<unsigned char> rgba(size_t(R) * R * 4, 0);
    std::vector<unsigned char> filled(size_t(R) * R, 0);   // 1: the texel has a colour (baked or extruded)
    for (size_t i = 0; i < size_t(R) * R; ++i) {
        rgba[i * 4] = opt.background[0];
        rgba[i * 4 + 1] = opt.background[1];
        rgba[i * 4 + 2] = opt.background[2];
    }

    const bool textured = source && source->rgba && source->width > 0 && source->height > 0 && cornerUV.size() == src.n_faces() * 6;
    const MipChain* mips = nullptr;
    std::unique_ptr<MipChain> mipStorage;
    if (textured) {
        mipStorage = std::make_unique<MipChain>(*source);
        mips = mipStorage.get();
    }

    // 1. the texels whose centre is inside an atlas triangle
    for (size_t ci = 0; ci < charts.size(); ++ci) {
        const Patch& patch = *charts[ci].patch;
        const std::vector<Eigen::Vector2d>& uv = *charts[ci].uv;
        unsigned char chartColor[3];
        hueRgb(int(ci), chartColor);
        for (auto lf : patch.mesh.faces()) {
            const int g = patch.faceToGlobal[lf.idx()];
            const bool useTexture = textured && (faceMaterial.empty() || faceMaterial[g] == textureMaterial);

            // atlas corners (texel units) and the source uv of the same corners, matched by vertex identity
            Eigen::Vector2d T[3], S[3];
            int gv[3], k = 0;
            for (auto v : patch.mesh.fv_range(lf)) {
                T[k] = uv[v.idx()] * double(R);
                gv[k] = patch.vertexToGlobal[v.idx()];
                ++k;
            }
            if (useTexture) {
                int slot = 0;
                for (auto v : src.fv_range(TriMesh::FaceHandle(g))) {
                    for (int c = 0; c < 3; ++c)
                        if (gv[c] == v.idx()) S[c] = Eigen::Vector2d(cornerUV[size_t(g) * 6 + slot * 2], cornerUV[size_t(g) * 6 + slot * 2 + 1]);
                    ++slot;
                }
            }

            const double area = 0.5 * ((T[1] - T[0]).x() * (T[2] - T[0]).y() - (T[1] - T[0]).y() * (T[2] - T[0]).x());
            if (std::abs(area) < 1e-12) continue;
            double lod = 0.0;
            if (useTexture) {   // source texels per atlas texel, from the areas
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
                    // barycentric coordinates
                    const double b0 = 0.5 * ((T[1] - p).x() * (T[2] - p).y() - (T[1] - p).y() * (T[2] - p).x()) / area;
                    const double b1 = 0.5 * ((T[2] - p).x() * (T[0] - p).y() - (T[2] - p).y() * (T[0] - p).x()) / area;
                    const double b2 = 1.0 - b0 - b1;
                    const double eps = -1e-9;
                    if (b0 < eps || b1 < eps || b2 < eps) continue;
                    unsigned char* px = &rgba[(size_t(y) * R + x) * 4];
                    if (useTexture) {
                        const Eigen::Vector2d s = b0 * S[0] + b1 * S[1] + b2 * S[2];
                        double c[3];
                        mips->trilinear(s.x(), s.y(), lod, c);
                        for (int ch = 0; ch < 3; ++ch) px[ch] = (unsigned char)std::max(0.0, std::min(255.0, c[ch] + 0.5));
                    } else if (!textured) {
                        for (int ch = 0; ch < 3; ++ch) px[ch] = chartColor[ch];   // no source: a flat colour per chart
                    }   // else: a face of another material keeps the background colour
                    filled[size_t(y) * R + x] = 1;
                }
        }
    }

    // 2. extrude the colours outward by margin + 1 texels (the +1 colours the texels a chart only touches)
    std::vector<unsigned char> nextFilled = filled;
    std::vector<unsigned char> nextRgb(rgba);
    // the neighbours in the order of preference: along the axes first (the nearest texel), then the diagonals
    static const int kNeighbours[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    for (int pass = 0; pass < opt.margin + 1; ++pass) {
        bool any = false;
        for (int y = 0; y < R; ++y)
            for (int x = 0; x < R; ++x) {
                const size_t at = size_t(y) * R + x;
                if (filled[at]) continue;
                for (const auto& d : kNeighbours) {
                    const int nx = x + d[0], ny = y + d[1];
                    if (nx < 0 || ny < 0 || nx >= R || ny >= R || !filled[size_t(ny) * R + nx]) continue;
                    const size_t from = size_t(ny) * R + nx;
                    for (int ch = 0; ch < 3; ++ch) nextRgb[at * 4 + ch] = rgba[from * 4 + ch];
                    nextFilled[at] = 1;
                    any = true;
                    break;
                }
            }
        if (!any) break;
        rgba = nextRgb;
        filled = nextFilled;
    }

    // 3. alpha = occupancy: the texels that a chart touches
    std::vector<MaskChart> mc;
    for (const BakeChart& c : charts) mc.push_back({&c.patch->mesh, c.uv});
    const std::vector<unsigned char> mask = rasterizeCoverage(mc, R);
    for (size_t i = 0; i < size_t(R) * R; ++i) rgba[i * 4 + 3] = mask[i];
    return rgba;
}

std::vector<float> cornerUVFromAtlas(const TriMesh& src, const std::vector<BakeChart>& charts)
{
    std::vector<float> out(src.n_faces() * 6, std::nanf(""));
    for (const BakeChart& c : charts) {
        const Patch& patch = *c.patch;
        for (auto lf : patch.mesh.faces()) {
            const int g = patch.faceToGlobal[lf.idx()];
            for (auto v : patch.mesh.fv_range(lf)) {
                const int gv = patch.vertexToGlobal[v.idx()];
                int slot = 0;
                for (auto sv : src.fv_range(TriMesh::FaceHandle(g))) {   // match the corner by the vertex of the source
                    if (sv.idx() == gv) {
                        out[size_t(g) * 6 + slot * 2] = float((*c.uv)[v.idx()].x());
                        out[size_t(g) * 6 + slot * 2 + 1] = float((*c.uv)[v.idx()].y());
                    }
                    ++slot;
                }
            }
        }
    }
    return out;
}

RetextureReport measureRetexture(const TriMesh& mesh, const std::vector<float>& oldUV, const BakeSource& oldTex,
                                 const std::vector<int>& faceMaterial, int textureMaterial,
                                 const std::vector<float>& newUV, const std::vector<unsigned char>& newRgba, int R,
                                 size_t faceStride)
{
    RetextureReport rep;
    if (!oldTex.rgba || oldUV.size() != mesh.n_faces() * 6 || newUV.size() != mesh.n_faces() * 6 ||
        newRgba.size() != size_t(R) * R * 4)
        return rep;
    const MipChain mips(oldTex);
    BakeSource nt{R, R, newRgba.data()};
    // the new texture is only looked up bilinearly: a chain with the one level
    auto bilinearNew = [&](double u, double v, double out[3]) {
        const double x = u * R - 0.5, y = v * R - 0.5;
        const double fx = std::floor(x), fy = std::floor(y), tx = x - fx, ty = y - fy;
        auto texel = [&](int ix, int iy, int c) {
            ix = std::max(0, std::min(R - 1, ix));   // clamp: the extruded border makes this safe
            iy = std::max(0, std::min(R - 1, iy));
            return double(newRgba[(size_t(iy) * R + ix) * 4 + c]);
        };
        for (int c = 0; c < 3; ++c) {
            const double a = texel(int(fx), int(fy), c), b = texel(int(fx) + 1, int(fy), c);
            const double d = texel(int(fx), int(fy) + 1, c), e = texel(int(fx) + 1, int(fy) + 1, c);
            out[c] = (a * (1 - tx) + b * tx) * (1 - ty) + (d * (1 - tx) + e * tx) * ty;
        }
    };
    (void)nt;

    static const double kBary[3][3] = {{1.0 / 3, 1.0 / 3, 1.0 / 3}, {0.6, 0.2, 0.2}, {0.2, 0.2, 0.6}};
    double sq = 0.0, sqFull = 0.0;
    for (size_t f = 0; f < mesh.n_faces(); f += std::max<size_t>(1, faceStride)) {
        if (!faceMaterial.empty() && faceMaterial[f] != textureMaterial) continue;
        bool ok = true;
        for (int k = 0; k < 6; ++k) ok = ok && !std::isnan(oldUV[f * 6 + k]) && !std::isnan(newUV[f * 6 + k]);
        if (!ok) continue;
        const float *o = &oldUV[f * 6], *n = &newUV[f * 6];
        const double oArea = 0.5 * std::abs(double(o[2] - o[0]) * double(o[5] - o[1]) - double(o[3] - o[1]) * double(o[4] - o[0])) *
                             double(oldTex.width) * oldTex.height;
        const double nArea = 0.5 * std::abs(double(n[2] - n[0]) * double(n[5] - n[1]) - double(n[3] - n[1]) * double(n[4] - n[0])) *
                             double(R) * R;
        if (oArea <= 0.0 || nArea <= 0.0) continue;
        const double lod = 0.5 * std::log2(std::max(1.0, oArea / nArea));
        for (const auto& b : kBary) {
            const double ou = b[0] * o[0] + b[1] * o[2] + b[2] * o[4], ov = b[0] * o[1] + b[1] * o[3] + b[2] * o[5];
            const double nu = b[0] * n[0] + b[1] * n[2] + b[2] * n[4], nv = b[0] * n[1] + b[1] * n[3] + b[2] * n[5];
            double cOld[3], cOldFull[3], cNew[3];
            mips.trilinear(ou, ov, lod, cOld);
            mips.trilinear(ou, ov, 0.0, cOldFull);
            bilinearNew(nu, nv, cNew);
            for (int c = 0; c < 3; ++c) {
                sq += (cOld[c] - cNew[c]) * (cOld[c] - cNew[c]);
                sqFull += (cOldFull[c] - cNew[c]) * (cOldFull[c] - cNew[c]);
            }
            ++rep.samples;
        }
    }
    if (rep.samples) {
        rep.rmse = std::sqrt(sq / (3.0 * rep.samples));
        rep.rmseFull = std::sqrt(sqFull / (3.0 * rep.samples));
        rep.psnr = rep.rmse > 0.0 ? 20.0 * std::log10(255.0 / rep.rmse) : 99.0;
        rep.psnrFull = rep.rmseFull > 0.0 ? 20.0 * std::log10(255.0 / rep.rmseFull) : 99.0;
    }
    return rep;
}
