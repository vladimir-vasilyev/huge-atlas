#include "TestUtil.h"
#include "param/AtlasPacker.h"
#include "param/ParamPipeline.h"

#include <fstream>
#include <random>
#include <sstream>

static const double kPi = 3.14159265358979323846;

static double hullArea(const std::vector<Eigen::Vector2d>& pts)
{
    const std::vector<Eigen::Vector2d> h = convexHull(pts);
    double a = 0.0;
    for (size_t i = 0; i < h.size(); ++i) {
        const auto& p = h[i];
        const auto& q = h[(i + 1) % h.size()];
        a += p.x() * q.y() - q.x() * p.y();
    }
    return 0.5 * a;
}

static void testHullAndRectangle()
{
    // a 4 x 1 rectangle rotated by 30 degrees, with interior and edge points
    std::vector<Eigen::Vector2d> pts;
    const double a = kPi / 6;
    for (double x = 0; x <= 4.0 + 1e-12; x += 0.5)
        for (double y = 0; y <= 1.0 + 1e-12; y += 0.25)
            pts.push_back(Eigen::Vector2d(std::cos(a) * x - std::sin(a) * y, std::sin(a) * x + std::cos(a) * y));
    const std::vector<Eigen::Vector2d> hull = convexHull(pts);
    CHECK(hull.size() >= 4);   // edge points are collinear only up to rounding, so extra hull vertices are possible
    CHECK_NEAR(hullArea(pts), 4.0, 1e-12);

    const double ang = minAreaRectangleAngle(hull);
    double lo[2] = {1e9, 1e9}, hi[2] = {-1e9, -1e9};
    for (const auto& p : pts) {
        const Eigen::Vector2d q(std::cos(-ang) * p.x() - std::sin(-ang) * p.y(), std::sin(-ang) * p.x() + std::cos(-ang) * p.y());
        for (int k = 0; k < 2; ++k) { lo[k] = std::min(lo[k], q[k]); hi[k] = std::max(hi[k], q[k]); }
    }
    CHECK_NEAR((hi[0] - lo[0]) * (hi[1] - lo[1]), 4.0, 1e-9);   // the minimum rectangle is the rectangle itself
}

static void testPacking()
{
    std::mt19937 rng(8);
    std::uniform_real_distribution<double> U(0.2, 3.0), A(0.0, kPi);
    std::vector<std::vector<Eigen::Vector2d>> uvs;
    std::vector<double> area3(0);
    std::vector<AtlasInput> in;
    for (int i = 0; i < 25; ++i) {
        // a randomly rotated random rectangle (as a point set with a few interior points)
        const double w = U(rng), h = U(rng), ang = A(rng), scale = U(rng);
        std::vector<Eigen::Vector2d> pts;
        for (double x : {0.0, 0.5, 1.0})
            for (double y : {0.0, 0.5, 1.0}) {
                const Eigen::Vector2d p(x * w, y * h);
                pts.push_back(scale * Eigen::Vector2d(std::cos(ang) * p.x() - std::sin(ang) * p.y(),
                                                       std::sin(ang) * p.x() + std::cos(ang) * p.y()));
            }
        uvs.push_back(pts);
        area3.push_back(U(rng));
    }
    for (size_t i = 0; i < uvs.size(); ++i) in.push_back({&uvs[i], hullArea(uvs[i]), area3[i]});

    AtlasOptions opt;
    const AtlasResult r = packAtlas(in, opt);
    CHECK(r.efficiency > 0.0 && r.efficiency <= 1.0);
    CHECK(r.efficiency > 0.25);                       // shelf packing of random rectangles: not absurdly loose

    double density = -1.0;
    for (size_t i = 0; i < in.size(); ++i) {
        CHECK(r.uv[i].size() == uvs[i].size());
        const auto& rc = r.rects[i];
        CHECK(rc[0] >= 0.0 && rc[1] >= 0.0 && rc[2] <= 1.0 && rc[3] <= 1.0 && rc[2] > rc[0] && rc[3] > rc[1]);
        for (const auto& p : r.uv[i])
            CHECK(p.x() >= rc[0] - 1e-9 && p.x() <= rc[2] + 1e-9 && p.y() >= rc[1] - 1e-9 && p.y() <= rc[3] + 1e-9);

        // same texel density everywhere: atlas area per 3D area is one constant
        const double d = hullArea(r.uv[i]) / area3[i];
        if (density < 0) density = d;
        CHECK_NEAR(d / density, 1.0, 1e-9);
        // the patch is not distorted: its hull area is the input area times a uniform factor squared
        CHECK_NEAR(hullArea(r.uv[i]) / hullArea(uvs[i]), (area3[i] / in[i].uvArea) * r.scale * r.scale, 1e-9);
    }
    // the rectangles do not touch each other: the gutter separates them
    const double gap = opt.gutter * std::sqrt([&] { double s = 0; for (double a : area3) s += a; return s; }()) * r.scale;
    for (size_t i = 0; i < in.size(); ++i)
        for (size_t j = i + 1; j < in.size(); ++j) {
            const auto &a = r.rects[i], &b = r.rects[j];
            const bool separated = a[2] + gap - 1e-9 <= b[0] || b[2] + gap - 1e-9 <= a[0] ||
                                   a[3] + gap - 1e-9 <= b[1] || b[3] + gap - 1e-9 <= a[1];
            CHECK(separated);
        }

    // invalid inputs are skipped
    std::vector<AtlasInput> bad = {{nullptr, 1.0, 1.0}, {&uvs[0], 0.0, 1.0}, in[0]};
    const AtlasResult rb = packAtlas(bad);
    CHECK(rb.uv[0].empty() && rb.uv[1].empty() && !rb.uv[2].empty());
    CHECK(packAtlas({}).uv.empty());

    // without rotation the patches keep their orientation: the same points before and after, up to translation and scale
    AtlasOptions norot;
    norot.rotate = false;
    const AtlasResult rn = packAtlas({in[3]}, norot);
    const double f = std::sqrt(area3[3] / in[3].uvArea) * rn.scale;
    for (size_t k = 1; k < uvs[3].size(); ++k)
        CHECK(((rn.uv[0][k] - rn.uv[0][0]) - f * (uvs[3][k] - uvs[3][0])).norm() < 1e-9);
}

static void testExportObj()
{
    TriMesh cube;
    addCube(cube, 0.0);
    std::vector<int> label(12);
    for (int f = 0; f < 12; ++f) label[f] = f / 2;
    PatchOptions po;
    po.minFaces = 1;
    const ExtractionResult ex = extractPatches(cube, label, po);
    CHECK(ex.patches.size() == 6);

    std::vector<PatchParam> params;
    std::vector<AtlasInput> in;
    for (const Patch& p : ex.patches) params.push_back(parameterizePatch(cube, p));
    for (const PatchParam& r : params) {
        CHECK(r.injective);
        in.push_back({&r.uv, r.distortion.scale * r.distortion.scale * 0.0 + [&] {
                          double a = 0; for (auto f : r.patch.mesh.faces()) {
                              Eigen::Vector2d q[3];
                              for (auto h : r.patch.mesh.fh_range(f)) q[cornerIndex(r.patch.mesh, h) % 3] = r.uv[r.patch.mesh.to_vertex_handle(h).idx()];
                              a += signedArea(q[0], q[1], q[2]); }
                          return a; }(),
                      4.0});
    }
    const AtlasResult atlas = packAtlas(in);
    std::vector<const Patch*> patches;
    std::vector<const std::vector<Eigen::Vector2d>*> uvs;
    for (size_t i = 0; i < params.size(); ++i) { patches.push_back(&params[i].patch); uvs.push_back(&atlas.uv[i]); }

    const std::string path = "/tmp/vsa_test_atlas.obj";
    CHECK(exportAtlasOBJ(path, cube, patches, uvs) == 12);

    std::ifstream f(path);
    std::string line;
    int nv = 0, nvt = 0, nf = 0;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") ++nv;
        else if (tag == "vt") {
            double u, v;
            ss >> u >> v;
            CHECK(u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0);
            ++nvt;
        } else if (tag == "f") {
            ++nf;
            std::string c;
            int count = 0;
            while (ss >> c) {
                int vi = 0, ti = 0;
                CHECK(std::sscanf(c.c_str(), "%d/%d", &vi, &ti) == 2);
                CHECK(vi >= 1 && vi <= 8 && ti >= 1 && ti <= 24);
                ++count;
            }
            CHECK(count == 3);
        }
    }
    CHECK(nv == 8 && nvt == 24 && nf == 12);   // seams: 24 texture vertices for 8 positions
    std::remove(path.c_str());

    // a patch without UV is skipped
    std::vector<const std::vector<Eigen::Vector2d>*> some = uvs;
    some[2] = nullptr;
    CHECK(exportAtlasOBJ(path, cube, patches, some) == 10);
    std::remove(path.c_str());
    CHECK(exportAtlasOBJ("/nonexistent_dir/x.obj", cube, patches, uvs) == -1);
}

int main()
{
    testHullAndRectangle();
    testPacking();
    testExportObj();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
