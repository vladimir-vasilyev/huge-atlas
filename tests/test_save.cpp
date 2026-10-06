#include "TestUtil.h"
#include "param/AtlasBake.h"
#include "param/AtlasMask.h"
#include "param/AtlasPacker.h"
#include "param/HorizonPacker.h"
#include "param/MeshWriters.h"
#include "param/UVMetrics.h"
#include "param/ParamPipeline.h"

#include <fstream>
#include <sstream>

// ---- a source texture and a mesh that uses it -------------------------------------------------------
// 64 x 64 gradient: red grows with u, green with v, blue is constant. Row 0 is v = 0.
static std::vector<unsigned char> gradientTexture(int n)
{
    std::vector<unsigned char> px(size_t(n) * n * 4);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            unsigned char* p = &px[(size_t(y) * n + x) * 4];
            p[0] = (unsigned char)std::lround(255.0 * (x + 0.5) / n);
            p[1] = (unsigned char)std::lround(255.0 * (y + 0.5) / n);
            p[2] = 128;
            p[3] = 255;
        }
    return px;
}

struct Square {
    TriMesh mesh;
    Patch patch;
    std::vector<float> cornerUV;              // the source uv: u = x, v = y of the corners
    std::vector<Eigen::Vector2d> atlasUV;     // the square placed at [0.25, 0.75]^2 of the atlas
};

static Square makeSquare()
{
    Square s;
    const auto a = s.mesh.add_vertex(Vec3(0, 0, 0)), b = s.mesh.add_vertex(Vec3(1, 0, 0)),
               c = s.mesh.add_vertex(Vec3(1, 1, 0)), d = s.mesh.add_vertex(Vec3(0, 1, 0));
    s.mesh.add_face(a, b, c);
    s.mesh.add_face(a, c, d);
    std::vector<int> faces = {0, 1};
    buildPatch(s.mesh, faces, {}, s.patch);
    for (auto f : s.mesh.faces())
        for (auto v : s.mesh.fv_range(f)) {
            const auto p = s.mesh.point(v);
            s.cornerUV.push_back(float(p[0]));
            s.cornerUV.push_back(float(p[1]));
        }
    for (auto v : s.patch.mesh.vertices()) {
        const auto p = s.patch.mesh.point(v);
        s.atlasUV.push_back({0.25 + 0.5 * p[0], 0.25 + 0.5 * p[1]});
    }
    return s;
}

static void testBakeGradient()
{
    Square sq = makeSquare();
    const std::vector<unsigned char> tex = gradientTexture(64);
    BakeSource src{64, 64, tex.data()};
    BakeOptions opt;
    opt.resolution = 64;
    opt.margin = 2;
    const std::vector<BakeChart> charts = {{&sq.patch, &sq.atlasUV}};
    const std::vector<unsigned char> out = bakeAtlasTexture(sq.mesh, charts, &src, sq.cornerUV, {}, 0, opt);
    CHECK(out.size() == 64 * 64 * 4);

    // inside (a few texels away from the border): the colour of the source at the same point
    int checked = 0, wrong = 0;
    for (int y = 18; y < 46; ++y)
        for (int x = 18; x < 46; ++x) {
            const double u = ((x + 0.5) / 64 - 0.25) / 0.5, v = ((y + 0.5) / 64 - 0.25) / 0.5;
            const unsigned char* p = &out[(size_t(y) * 64 + x) * 4];
            ++checked;
            wrong += std::abs(p[0] - 255.0 * u) > 4.0 || std::abs(p[1] - 255.0 * v) > 4.0 || p[2] != 128 || p[3] != 255;
        }
    CHECK(checked > 700 && wrong == 0);

    // the alpha channel is the occupancy mask
    const std::vector<unsigned char> mask = rasterizeCoverage({{&sq.patch.mesh, &sq.atlasUV}}, 64);
    for (size_t i = 0; i < mask.size(); ++i) CHECK(out[i * 4 + 3] == mask[i]);

    // the chart border is extruded into the gutter: the texels around it repeat the nearest chart texel
    // (here the square covers texels 16..47, so column 15 and 14 are outside, with alpha 0)
    for (int y = 20; y < 44; ++y) {
        const unsigned char* edge = &out[(size_t(y) * 64 + 16) * 4];
        for (int dx = 1; dx <= 2; ++dx) {
            const unsigned char* g = &out[(size_t(y) * 64 + 16 - dx) * 4];
            if (dx == 2) CHECK(g[3] == 0);   // column 15 only touches the chart (alpha 255), column 14 is a pure gutter
            CHECK(g[0] == edge[0] && g[1] == edge[1] && g[2] == edge[2]);
        }
        // beyond margin + 1 texels the background is left alone
        const unsigned char* far = &out[(size_t(y) * 64 + 16 - 5) * 4];
        CHECK(far[0] == 128 && far[1] == 128 && far[2] == 128 && far[3] == 0);
    }
}

static void testBakeDownsampling()
{
    // a fine black and white checker on a large texture: the atlas is much smaller, so the mip chain
    // has to average it instead of aliasing
    const int n = 512;
    std::vector<unsigned char> tex(size_t(n) * n * 4, 255);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x)
            if ((x + y) % 2) for (int c = 0; c < 3; ++c) tex[(size_t(y) * n + x) * 4 + c] = 0;
    Square sq = makeSquare();
    BakeSource src{n, n, tex.data()};
    BakeOptions opt;
    opt.resolution = 64;
    opt.margin = 1;
    const std::vector<unsigned char> out =
        bakeAtlasTexture(sq.mesh, {{&sq.patch, &sq.atlasUV}}, &src, sq.cornerUV, {}, 0, opt);
    int outliers = 0;
    for (int y = 20; y < 44; ++y)
        for (int x = 20; x < 44; ++x) outliers += std::abs(int(out[(size_t(y) * 64 + x) * 4]) - 127) > 40;
    CHECK(outliers == 0);   // a medium grey, not black / white noise
}

static void testBakeMaterialsAndNoSource()
{
    Square sq = makeSquare();
    const std::vector<unsigned char> tex = gradientTexture(64);
    BakeSource src{64, 64, tex.data()};
    BakeOptions opt;
    opt.resolution = 64;
    opt.margin = 2;
    opt.background[0] = 10; opt.background[1] = 20; opt.background[2] = 30;
    const std::vector<BakeChart> charts = {{&sq.patch, &sq.atlasUV}};

    // the second triangle uses another material: it keeps the background colour
    const std::vector<int> material = {0, 1};
    const std::vector<unsigned char> out = bakeAtlasTexture(sq.mesh, charts, &src, sq.cornerUV, material, 0, opt);
    // the first triangle (0,0)(1,0)(1,1) is below the diagonal: texel (40, 24) is inside it, (24, 40) in the second
    const unsigned char* a = &out[(size_t(24) * 64 + 40) * 4];
    const unsigned char* b = &out[(size_t(40) * 64 + 24) * 4];
    CHECK(a[0] != 10 && a[2] == 128);
    CHECK(b[0] == 10 && b[1] == 20 && b[2] == 30 && b[3] == 255);

    // no source: one flat colour per chart, alpha still the mask
    const std::vector<unsigned char> flat = bakeAtlasTexture(sq.mesh, charts, nullptr, {}, {}, 0, opt);
    const unsigned char* p = &flat[(size_t(30) * 64 + 30) * 4];
    const unsigned char* q = &flat[(size_t(26) * 64 + 38) * 4];
    CHECK(p[3] == 255 && p[0] == q[0] && p[1] == q[1] && p[2] == q[2]);
    CHECK(!(p[0] == 10 && p[1] == 20 && p[2] == 30));
}

static void testLayoutStats()
{
    Square sq = makeSquare();
    const std::vector<float>& identity = sq.cornerUV;   // u = x, v = y: an isometry of the unit square

    UVLayoutStats a = analyzeUVLayout(sq.mesh, identity, 16);
    CHECK(a.faces == 2 && a.flipped == 0 && a.facesWithoutUV == 0);
    CHECK(a.charts == 1 && a.seamLength == 0.0 && std::abs(a.edgeLength - std::sqrt(2.0)) < 1e-12);
    CHECK(std::abs(a.l2Mean - 1.0) < 1e-9 && std::abs(a.conformalP50 - 1.0) < 1e-9 && std::abs(a.densityP50 - 1.0) < 1e-9);
    CHECK(a.covered > 0.999 && a.overlap == 0.0);          // the unit square fills the whole texture, no texel twice

    // the two triangles apart in uv: two charts, the diagonal is a seam
    std::vector<float> apart = identity;
    for (int k = 0; k < 3; ++k) apart[6 + k * 2] += 0.0f;   // keep the second triangle's u
    for (size_t k = 6; k < 12; k += 2) { apart[k] = apart[k] * 0.5f + 2.0f; apart[k + 1] = apart[k + 1] * 0.5f + 2.0f; }
    UVLayoutStats b = analyzeUVLayout(sq.mesh, apart, 16);
    CHECK(b.charts == 2 && std::abs(b.seamLength - std::sqrt(2.0)) < 1e-12 && std::abs(b.seamRatio() - 1.0) < 1e-12);

    // both triangles on the same uv region: the texels are covered twice
    std::vector<float> same(12);
    for (int k = 0; k < 6; ++k) same[k] = same[6 + k] = identity[k];
    UVLayoutStats c = analyzeUVLayout(sq.mesh, same, 32);
    CHECK(c.overlap > 0.99);

    // a layout that is mirrored as a whole is not an error: it is measured like the original
    std::vector<float> mirror = identity;
    for (size_t k = 0; k < mirror.size(); k += 2) mirror[k] = 1.0f - mirror[k];
    const UVLayoutStats mirrored = analyzeUVLayout(sq.mesh, mirror, 16);
    CHECK(mirrored.mirrored && mirrored.flipped == 0 && !a.mirrored);
    CHECK(std::abs(mirrored.l2Mean - a.l2Mean) < 1e-9 && std::abs(mirrored.conformalP50 - 1.0) < 1e-9);
    CHECK(mirrored.covered > 0.999);

    // one face against the rest: flipped (here the second face is 3 times the first one, so it is the majority)
    std::vector<float> oneFlipped = identity;
    for (size_t k = 6; k < 12; k += 2) oneFlipped[k] = 1.0f - oneFlipped[k];   // mirrors only the second triangle
    const UVLayoutStats mixed = analyzeUVLayout(sq.mesh, oneFlipped, 16);
    CHECK(mixed.flipped == 1);

    // stretched along u by 2: conformal 2 (after the area normalization), density still 1, covered half of the u range
    std::vector<float> stretched = identity;
    for (size_t k = 0; k < stretched.size(); k += 2) stretched[k] *= 0.5f;   // the uv square is twice as narrow
    UVLayoutStats d = analyzeUVLayout(sq.mesh, stretched, 16);
    CHECK(std::abs(d.conformalP50 - 2.0) < 1e-6 && std::abs(d.densityP50 - 1.0) < 1e-6);
    CHECK(d.covered > 0.45 && d.covered < 0.55);

    // a face without uv
    std::vector<float> partial = identity;
    for (int k = 6; k < 12; ++k) partial[k] = std::nanf("");
    UVLayoutStats e = analyzeUVLayout(sq.mesh, partial, 16);
    CHECK(e.facesWithoutUV == 1 && e.charts == 1);
    CHECK(analyzeUVLayout(sq.mesh, {}, 16).facesWithoutUV == 2);
}

static void testRetextureReport()
{
    Square sq = makeSquare();
    const std::vector<unsigned char> tex = gradientTexture(64);
    BakeSource src{64, 64, tex.data()};
    BakeOptions opt;
    opt.resolution = 64;
    opt.margin = 2;
    const std::vector<BakeChart> charts = {{&sq.patch, &sq.atlasUV}};
    const std::vector<unsigned char> baked = bakeAtlasTexture(sq.mesh, charts, &src, sq.cornerUV, {}, 0, opt);
    const std::vector<float> newUV = cornerUVFromAtlas(sq.mesh, charts);
    CHECK(newUV.size() == 12 && !std::isnan(newUV[0]));
    CHECK(analyzeUVLayout(sq.mesh, newUV, 64).charts == 1);

    // the corners of the new uv are the atlas uv of the same vertices
    int slot = 0;
    for (auto v : sq.mesh.fv_range(TriMesh::FaceHandle(0))) {
        const auto p = sq.mesh.point(v);
        CHECK(std::abs(newUV[slot * 2] - (0.25 + 0.5 * p[0])) < 1e-6 && std::abs(newUV[slot * 2 + 1] - (0.25 + 0.5 * p[1])) < 1e-6);
        ++slot;
    }

    const RetextureReport good = measureRetexture(sq.mesh, sq.cornerUV, src, {}, 0, newUV, baked, 64);
    CHECK(good.samples == 6);
    CHECK(good.rmse < 3.0 && good.psnr > 38.0);

    // a wrong mapping (the uv shifted) is clearly worse
    std::vector<float> wrong = newUV;
    for (size_t k = 0; k < wrong.size(); k += 2) wrong[k] += 0.1f;
    const RetextureReport bad = measureRetexture(sq.mesh, sq.cornerUV, src, {}, 0, wrong, baked, 64);
    CHECK(bad.rmse > 10.0 * good.rmse);

    // faces of another material are not compared; no samples at all gives an empty report
    CHECK(measureRetexture(sq.mesh, sq.cornerUV, src, {1, 1}, 0, newUV, baked, 64).samples == 0);
}

// ---- file writers ---------------------------------------------------------------------------------
struct Cube {
    TriMesh mesh;
    ExtractionResult ex;
    std::vector<PatchParam> params;
    std::vector<std::vector<Eigen::Vector2d>> atlasUV;
};

static Cube makeCubeAtlas()
{
    Cube c;
    addCube(c.mesh, 0.0);
    std::vector<int> label(12);
    for (int f = 0; f < 12; ++f) label[f] = f / 2;
    PatchOptions po;
    po.minFaces = 1;
    c.ex = extractPatches(c.mesh, label, po);
    std::vector<AtlasInput> in;
    for (const Patch& p : c.ex.patches) c.params.push_back(parameterizePatch(c.mesh, p));
    for (const PatchParam& r : c.params) {
        double ua = 0;
        for (auto f : r.patch.mesh.faces()) {
            Eigen::Vector2d q[3];
            for (auto h : r.patch.mesh.fh_range(f)) q[cornerIndex(r.patch.mesh, h) % 3] = r.uv[r.patch.mesh.to_vertex_handle(h).idx()];
            ua += signedArea(q[0], q[1], q[2]);
        }
        in.push_back({&r.uv, ua, 4.0, &r.patch.mesh});
    }
    HorizonOptions o;
    o.resolution = 256;
    o.maxMipLevel = 1;
    const AtlasResult r = packHorizon(in, o);
    c.atlasUV = r.uv;
    return c;
}

static std::string slurp(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

static void testObjAndMtl()
{
    Cube c = makeCubeAtlas();
    std::vector<const Patch*> patches;
    std::vector<const std::vector<Eigen::Vector2d>*> uv;
    for (size_t i = 0; i < c.params.size(); ++i) { patches.push_back(&c.params[i].patch); uv.push_back(&c.atlasUV[i]); }

    const std::string path = "/tmp/vsa_test_save.obj";
    CHECK(exportAtlasOBJ(path, c.mesh, patches, uv, "atlas test", "vsa_test_save_atlas.png") == 12);
    const std::string obj = slurp(path), mtl = slurp("/tmp/vsa_test_save.mtl");
    CHECK(obj.find("mtllib vsa_test_save.mtl") != std::string::npos);
    CHECK(obj.find("usemtl atlas") != std::string::npos);
    CHECK(mtl.find("newmtl atlas") != std::string::npos && mtl.find("map_Kd vsa_test_save_atlas.png") != std::string::npos);
    int nv = 0, nvt = 0, nf = 0;
    std::istringstream is(obj);
    std::string line;
    while (std::getline(is, line)) {
        if (line.rfind("v ", 0) == 0) ++nv;
        else if (line.rfind("vt ", 0) == 0) ++nvt;
        else if (line.rfind("f ", 0) == 0) ++nf;
    }
    CHECK(nv == 8 && nvt == 24 && nf == 12);
    std::remove(path.c_str());
    std::remove("/tmp/vsa_test_save.mtl");

    // without a texture there is no material
    CHECK(exportAtlasOBJ(path, c.mesh, patches, uv) == 12);
    CHECK(slurp(path).find("mtllib") == std::string::npos);
    std::remove(path.c_str());
}

template <class T>
static T readLE(const std::string& data, size_t& pos)
{
    T v;
    std::memcpy(&v, data.data() + pos, sizeof(T));
    pos += sizeof(T);
    return v;
}

static void testPly(bool binary)
{
    Cube c = makeCubeAtlas();
    std::vector<const Patch*> patches;
    std::vector<const std::vector<Eigen::Vector2d>*> uv;
    for (size_t i = 0; i < c.params.size(); ++i) { patches.push_back(&c.params[i].patch); uv.push_back(&c.atlasUV[i]); }

    const std::string path = "/tmp/vsa_test_save.ply";
    CHECK(writePlyAtlas(path, patches, uv, "atlas.png", binary, "unit test") == 12);
    const std::string data = slurp(path);
    const size_t end = data.find("end_header\n");
    CHECK(end != std::string::npos);
    const std::string head = data.substr(0, end);
    CHECK(head.find(binary ? "format binary_little_endian 1.0" : "format ascii 1.0") != std::string::npos);
    CHECK(head.find("comment TextureFile atlas.png") != std::string::npos);
    CHECK(head.find("element vertex 24") != std::string::npos && head.find("element face 12") != std::string::npos);
    CHECK(head.find("property float s") != std::string::npos && head.find("property float t") != std::string::npos);

    // read the body back and compare with the patches
    std::vector<std::array<double, 5>> verts;
    std::vector<std::array<int, 3>> faces;
    size_t pos = end + 11;
    if (binary) {
        for (int i = 0; i < 24; ++i) {
            std::array<double, 5> v;
            for (double& x : v) x = readLE<float>(data, pos);
            verts.push_back(v);
        }
        for (int i = 0; i < 12; ++i) {
            CHECK(readLE<unsigned char>(data, pos) == 3);
            std::array<int, 3> f;
            for (int& x : f) x = readLE<int32_t>(data, pos);
            faces.push_back(f);
        }
        CHECK(pos == data.size());
    } else {
        std::istringstream is(data.substr(pos));
        for (int i = 0; i < 24; ++i) { std::array<double, 5> v; for (double& x : v) is >> x; verts.push_back(v); }
        for (int i = 0; i < 12; ++i) { int n; std::array<int, 3> f; is >> n >> f[0] >> f[1] >> f[2]; CHECK(n == 3); faces.push_back(f); }
    }
    size_t k = 0;
    int mismatches = 0;
    for (size_t i = 0; i < patches.size(); ++i)
        for (auto v : patches[i]->mesh.vertices()) {
            const auto p = patches[i]->mesh.point(v);
            const auto& a = verts[k++];
            mismatches += std::abs(a[0] - p[0]) > 1e-5 || std::abs(a[1] - p[1]) > 1e-5 || std::abs(a[2] - p[2]) > 1e-5 ||
                          std::abs(a[3] - c.atlasUV[i][v.idx()].x()) > 1e-5 || std::abs(a[4] - c.atlasUV[i][v.idx()].y()) > 1e-5;
        }
    CHECK(mismatches == 0);
    for (const auto& f : faces) for (int idx : f) CHECK(idx >= 0 && idx < 24);

    // another program can read the geometry (OpenMesh knows PLY; the s, t properties are extra)
    TriMesh back;
    std::string err;
    CHECK(loadMesh(path, back, &err));
    CHECK(back.n_vertices() == 24 && back.n_faces() == 12);

    // a patch without uv is skipped
    std::vector<const std::vector<Eigen::Vector2d>*> some = uv;
    some[1] = nullptr;
    CHECK(writePlyAtlas(path, patches, some, "", binary) == 10);
    CHECK(slurp(path).find("TextureFile") == std::string::npos);
    CHECK(writePlyAtlas("/nonexistent_dir/x.ply", patches, uv, "", binary) == -1);
    std::remove(path.c_str());
}

static void testPlainMeshes()
{
    TriMesh cube;
    addCube(cube, 0.0);
    for (bool binary : {true, false}) {
        const std::string path = "/tmp/vsa_test_plain.ply";
        CHECK(writePlyMesh(path, cube, binary));
        TriMesh back;
        CHECK(loadMesh(path, back));
        CHECK(back.n_vertices() == 8 && back.n_faces() == 12);
        for (size_t i = 0; i < 8; ++i)
            CHECK((back.point(TriMesh::VertexHandle(int(i))) - cube.point(TriMesh::VertexHandle(int(i)))).norm() < 1e-5);
        std::remove(path.c_str());
    }
    const std::string objPath = "/tmp/vsa_test_plain.obj";
    CHECK(writeObjMesh(objPath, cube));
    TriMesh back;
    CHECK(loadMesh(objPath, back));
    CHECK(back.n_vertices() == 8 && back.n_faces() == 12);
    std::remove(objPath.c_str());
    CHECK(!writeObjMesh("/nonexistent_dir/x.obj", cube) && !writePlyMesh("/nonexistent_dir/x.ply", cube, true));
}

int main()
{
    testBakeGradient();
    testBakeDownsampling();
    testBakeMaterialsAndNoSource();
    testLayoutStats();
    testRetextureReport();
    testObjAndMtl();
    testPly(true);
    testPly(false);
    testPlainMeshes();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
