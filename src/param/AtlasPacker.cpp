#include "param/AtlasPacker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector2d rotate(const Eigen::Vector2d& p, double a)
{
    const double c = std::cos(a), s = std::sin(a);
    return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

struct Box { double x0, y0, x1, y1; double w() const { return x1 - x0; } double h() const { return y1 - y0; } };

Box boundsOf(const std::vector<Eigen::Vector2d>& pts, double angle)
{
    Box b{1e300, 1e300, -1e300, -1e300};
    for (const auto& p : pts) {
        const Eigen::Vector2d q = rotate(p, -angle);
        b.x0 = std::min(b.x0, q.x()); b.x1 = std::max(b.x1, q.x());
        b.y0 = std::min(b.y0, q.y()); b.y1 = std::max(b.y1, q.y());
    }
    return b;
}

struct Item {
    int input;
    double angle = 0.0, s = 1.0;
    Box box{};                 // bounds of the rotated, scaled patch
    double x = 0.0, y = 0.0;   // lower-left corner of the padded cell
};

// Shelf packing of padded cells into rows of width W. Returns the used height; fills x, y.
double shelfPack(std::vector<Item>& items, double g, double W)
{
    double x = 0.0, y = 0.0, rowH = 0.0;
    for (Item& it : items) {
        const double cw = it.box.w() + g, ch = it.box.h() + g;
        if (x > 0.0 && x + cw > W) { y += rowH; x = 0.0; rowH = 0.0; }
        it.x = x;
        it.y = y;
        x += cw;
        rowH = std::max(rowH, ch);
    }
    return y + rowH;
}

} // namespace

std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> pts)
{
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
    });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    if (pts.size() < 3) return pts;
    auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<Eigen::Vector2d> h(2 * pts.size());
    size_t k = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
        h[k++] = pts[i];
    }
    for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross(h[k - 2], h[k - 1], pts[i - 1]) <= 0) --k;
        h[k++] = pts[i - 1];
    }
    h.resize(k - 1);
    return h;
}

double minAreaRectangleAngle(const std::vector<Eigen::Vector2d>& hull)
{
    if (hull.size() < 2) return 0.0;
    double best = 0.0, bestArea = 1e300;
    for (size_t i = 0; i < hull.size(); ++i) {
        const Eigen::Vector2d e = hull[(i + 1) % hull.size()] - hull[i];
        if (e.squaredNorm() == 0.0) continue;
        const double a = std::atan2(e.y(), e.x());
        const Box b = boundsOf(hull, a);
        if (b.w() * b.h() < bestArea) { bestArea = b.w() * b.h(); best = a; }
    }
    return best;
}

AtlasResult packAtlas(const std::vector<AtlasInput>& in, const AtlasOptions& opt)
{
    AtlasResult res;
    res.uv.resize(in.size());
    res.rects.assign(in.size(), {0, 0, 0, 0});

    std::vector<Item> items;
    double totalArea = 0.0;
    for (size_t i = 0; i < in.size(); ++i) {
        const AtlasInput& a = in[i];
        if (!a.uv || a.uv->empty() || a.uvArea <= 0.0 || a.area3D <= 0.0) continue;
        Item it;
        it.input = int(i);
        it.s = std::sqrt(a.area3D / a.uvArea);
        std::vector<Eigen::Vector2d> pts = *a.uv;
        for (auto& p : pts) p *= it.s;
        if (opt.rotate) {
            const std::vector<Eigen::Vector2d> hull = convexHull(pts);
            it.angle = minAreaRectangleAngle(hull);
            Box b = boundsOf(pts, it.angle);
            if (b.w() < b.h()) { it.angle += 0.5 * kPi; }   // wide rectangles pack better on shelves
        }
        it.box = boundsOf(pts, it.angle);
        items.push_back(it);
        totalArea += a.area3D;
    }
    if (items.empty()) return res;

    const double g = opt.gutter * std::sqrt(totalArea);
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        return a.box.h() != b.box.h() ? a.box.h() > b.box.h() : a.input < b.input;
    });

    // the shelf width that gives the squarest atlas
    double widest = 0.0, cells = 0.0;
    for (const Item& it : items) { widest = std::max(widest, it.box.w() + g); cells += (it.box.w() + g) * (it.box.h() + g); }
    double bestSide = 1e300, bestW = 0.0;
    for (double f = 0.7; f <= 3.0; f += 0.05) {
        const double W = std::max(widest, f * std::sqrt(cells));
        std::vector<Item> trial = items;
        const double H = shelfPack(trial, g, W);
        double used = 0.0;
        for (const Item& it : trial) used = std::max(used, it.x + it.box.w() + g);
        const double side = std::max(used, H);
        if (side < bestSide) { bestSide = side; bestW = W; }
    }
    shelfPack(items, g, bestW);

    res.side = bestSide;
    res.scale = 1.0 / bestSide;
    res.efficiency = totalArea / (bestSide * bestSide);
    for (const Item& it : items) {
        const AtlasInput& a = in[it.input];
        std::vector<Eigen::Vector2d>& out = res.uv[it.input];
        out.reserve(a.uv->size());
        for (const auto& p : *a.uv) {
            const Eigen::Vector2d q = rotate(p * it.s, -it.angle);
            out.push_back(Eigen::Vector2d(q.x() - it.box.x0 + it.x + 0.5 * g, q.y() - it.box.y0 + it.y + 0.5 * g) * res.scale);
        }
        res.rects[it.input] = {(it.x + 0.5 * g) * res.scale, (it.y + 0.5 * g) * res.scale,
                               (it.x + 0.5 * g + it.box.w()) * res.scale, (it.y + 0.5 * g + it.box.h()) * res.scale};
    }
    return res;
}

int exportAtlasOBJ(const std::string& path, const TriMesh& source, const std::vector<const Patch*>& patches,
                   const std::vector<const std::vector<Eigen::Vector2d>*>& atlasUV, const std::string& comment)
{
    std::ofstream f(path);
    if (!f) return -1;
    f << "# VSA + LinABF atlas\n";
    if (!comment.empty()) f << "# " << comment << "\n";
    char buf[128];
    for (auto v : source.vertices()) {
        const Vec3 p = source.point(v);
        std::snprintf(buf, sizeof(buf), "v %.9g %.9g %.9g\n", p[0], p[1], p[2]);
        f << buf;
    }
    std::vector<int> offset(patches.size(), -1);
    int next = 1;
    for (size_t i = 0; i < patches.size(); ++i) {
        if (!atlasUV[i] || atlasUV[i]->size() != patches[i]->mesh.n_vertices()) continue;
        offset[i] = next;
        for (const auto& t : *atlasUV[i]) {
            std::snprintf(buf, sizeof(buf), "vt %.9g %.9g\n", t.x(), t.y());
            f << buf;
        }
        next += int(atlasUV[i]->size());
    }
    int faces = 0;
    for (size_t i = 0; i < patches.size(); ++i) {
        if (offset[i] < 0) continue;
        const Patch& p = *patches[i];
        for (auto lf : p.mesh.faces()) {
            const auto gf = TriMesh::FaceHandle(p.faceToGlobal[lf.idx()]);
            f << "f";
            auto sv = source.cfv_begin(gf);
            for (auto pv = p.mesh.cfv_begin(lf); pv != p.mesh.cfv_end(lf); ++pv, ++sv)
                f << " " << sv->idx() + 1 << "/" << offset[i] + pv->idx();
            f << "\n";
            ++faces;
        }
    }
    return f.good() ? faces : -1;
}
