#include "core/Operations.h"
#include "TestUtil.h"
#include "param/ParamPipeline.h"
#include "param/PatchExtractor.h"

static const double kPi = 3.14159265358979323846;

static TriMesh makeGrid(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) v.push_back(m.add_vertex(Vec3(i * 1.0, j * 0.8 + 0.1 * i, 0)));
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// The whole mesh as one patch.
static Patch wholePatch(const TriMesh& m)
{
    std::vector<int> faces(m.n_faces());
    for (size_t i = 0; i < faces.size(); ++i) faces[i] = int(i);
    Patch p;
    CHECK(buildPatch(m, faces, {}, p));
    p.region = 0;
    return p;
}

static TriMesh makeWavyFan(int n)
{
    TriMesh m;
    const auto c = m.add_vertex(Vec3(0, 0, 0));
    std::vector<TriMesh::VertexHandle> ring;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * kPi * i / n;
        ring.push_back(m.add_vertex(Vec3(std::cos(a), std::sin(a), i % 2 ? 1.0 : -1.0)));
    }
    for (int i = 0; i < n - 1; ++i) m.add_face(c, ring[i], ring[i + 1]);
    return m;
}

// A strip of `rows` vertex rows on a cone of half-angle gamma, winding `turns` times around it.
// The cone unfolds to a sector of angle 2 pi turns sin(gamma): above 2 pi the strip overlaps itself.
static TriMesh makeConeStrip(double gamma, double turns, int segments, int rows)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int i = 0; i <= segments; ++i)
        for (int j = 0; j < rows; ++j) {
            const double phi = 2 * kPi * turns * i / segments, rho = 1.0 + 0.3 * j / (rows - 1);
            v.push_back(m.add_vertex(Vec3(rho * std::sin(gamma) * std::cos(phi), rho * std::sin(gamma) * std::sin(phi),
                                          rho * std::cos(gamma))));
        }
    auto id = [rows](int i, int j) { return i * rows + j; };
    for (int i = 0; i < segments; ++i)
        for (int j = 0; j + 1 < rows; ++j) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

static void testDistortion()
{
    const TriMesh g = makeGrid(5);
    std::vector<Eigen::Vector2d> uv;
    for (auto v : g.vertices()) uv.push_back({g.point(v)[0], g.point(v)[1]});

    std::vector<float> per;
    DistortionStats s = computeDistortion(g, uv, &per);
    CHECK(s.flipped == 0 && per.size() == g.n_faces());
    CHECK_NEAR(s.l2Mean, 1.0, 1e-12);
    CHECK_NEAR(s.l2Max, 1.0, 1e-12);
    CHECK_NEAR(s.conformalMax, 1.0, 1e-12);
    CHECK_NEAR(s.areaRatioMin, 1.0, 1e-12);
    CHECK_NEAR(s.areaRatioMax, 1.0, 1e-12);

    // uniform scaling does not matter: the map is normalized to the 3D area
    std::vector<Eigen::Vector2d> big = uv;
    for (auto& p : big) p *= 7.3;
    s = computeDistortion(g, big);
    CHECK_NEAR(s.l2Mean, 1.0, 1e-12);
    CHECK_NEAR(s.scale, 1.0 / 7.3, 1e-12);

    // stretch x by 2: after the area normalization the singular values are sqrt(2) and 1 / sqrt(2)
    std::vector<Eigen::Vector2d> aniso = uv;
    for (auto& p : aniso) p.x() *= 2.0;
    // (the grid is sheared, so compare the invariants rather than the faces)
    s = computeDistortion(g, aniso);
    CHECK(s.flipped == 0);
    CHECK(s.conformalMax > 1.5);
    CHECK(s.areaRatioMin > 0.99 && s.areaRatioMax < 1.01);   // area is preserved by a pure stretch

    // a mirror image flips every triangle
    std::vector<Eigen::Vector2d> mirror = uv;
    for (auto& p : mirror) p.x() = -p.x();
    CHECK(computeDistortion(g, mirror).flipped == int(g.n_faces()));
}

static void testTutte()
{
    const Fixture fx(makeBump(22));
    VSAState s;
    initFarthestSeeds(fx.input(), s, 4, 2);
    runLloyd(fx.input(), s);
    const ExtractionResult ex = extractPatches(fx.mesh, s.label);
    int checked = 0;
    for (const Patch& p : ex.patches) {
        if (!p.isDisk()) continue;
        std::string err;
        const std::vector<Eigen::Vector2d> uv = tutteEmbedding(p.mesh, &err);
        CHECK(uv.size() == p.mesh.n_vertices());
        CHECK(checkUV(p.mesh, uv).ok());                       // injective by construction
        for (auto v : p.mesh.vertices()) {
            const double r = uv[v.idx()].norm();
            if (p.mesh.is_boundary(v)) CHECK_NEAR(r, 1.0, 1e-12);
            else CHECK(r < 1.0);
        }
        ++checked;
    }
    CHECK(checked >= 1);

    TriMesh cube;
    addCube(cube, 0.0);
    Patch cp;
    std::string err;
    CHECK(tutteEmbedding(cube, &err).empty() && !err.empty());   // closed: not a disk
}

static void testCut()
{
    const Fixture fx(makeBump(20));
    const Patch whole = wholePatch(fx.mesh);
    Patch cut;
    std::string err;
    CHECK(cutAtWorstVertex(fx.mesh, whole, 1.0, cut, &err));
    CHECK(cut.cuts == 1 && !cut.cutEdges.empty());
    CHECK(cut.mesh.n_faces() == whole.mesh.n_faces());
    CHECK(cut.mesh.n_vertices() > whole.mesh.n_vertices());       // vertices along the slit are duplicated
    CHECK(cut.topology.kind == PatchKind::Disk);                  // a slit from the interior to the boundary keeps the disk
    CHECK(cut.faceToGlobal == whole.faceToGlobal);

    Patch cut2;
    CHECK(cutAtWorstVertex(fx.mesh, cut, 1.0, cut2, &err));       // cutting again keeps the first slit
    CHECK(cut2.cuts == 2 && cut2.cutEdges.size() > cut.cutEdges.size() && cut2.topology.kind == PatchKind::Disk);

    // a single triangle has no interior vertex
    TriMesh t;
    t.add_face(t.add_vertex(Vec3(0, 0, 0)), t.add_vertex(Vec3(1, 0, 0)), t.add_vertex(Vec3(0, 1, 0)));
    const Patch tp = wholePatch(t);
    Patch none;
    CHECK(!cutAtWorstVertex(t, tp, 1.0, none, &err) && !err.empty());
}

static void testPipelineLadder()
{
    // 1. an easy patch: plain LinABF is enough, the map is isometric
    {
        const TriMesh g = makeGrid(6);
        const PatchParam r = parameterizePatch(g, wholePatch(g));
        CHECK(r.method == ParamMethod::LinABF && r.injective && !r.needsSplit);
        CHECK_NEAR(r.distortion.l2Mean, 1.0, 1e-9);
        CHECK(r.perTriangleL2.size() == r.patch.mesh.n_faces());
    }
    // 2. a fold at a boundary vertex: needs the active set
    {
        const TriMesh w = makeWavyFan(6);
        const PatchParam r = parameterizePatch(w, wholePatch(w));
        CHECK(r.method == ParamMethod::ActiveSet && r.injective);
        CHECK(r.log.find("fold") != std::string::npos);
    }
    // 3. a strip that overlaps itself when unrolled: neither the active set nor a slit helps, Tutte is injective
    //    but strongly stretched, so the patch is reported as needing a split
    {
        const TriMesh c = makeConeStrip(1.05, 1.5, 90, 4);   // 60 degrees: the unrolled angle is 8.2 rad > 2 pi
        const Patch whole = wholePatch(c);
        CHECK(whole.isDisk());
        const PatchParam r = parameterizePatch(c, whole);
        CHECK(r.method == ParamMethod::Tutte);
        CHECK(r.injective);
        CHECK(r.needsSplit);
        CHECK(r.distortion.l2Max > 4.0);
        CHECK(r.log.find("crossings") != std::string::npos);

        ParamOptions noTutte;
        noTutte.allowTutte = false;
        const PatchParam f = parameterizePatch(c, whole, noTutte);
        CHECK(!f.injective && f.needsSplit);
    }
    // 4. the narrow cone unrolls into less than 2 pi: no problem at all
    {
        const TriMesh c = makeConeStrip(0.35, 1.5, 90, 4);   // 20 degrees: 3.2 rad
        const PatchParam r = parameterizePatch(c, wholePatch(c));
        CHECK(r.injective && r.method == ParamMethod::LinABF);
    }
    // 5. non-disk input
    {
        TriMesh cube;
        addCube(cube, 0.0);
        std::vector<int> faces(12);
        for (int i = 0; i < 12; ++i) faces[i] = i;
        Patch p;
        CHECK(buildPatch(cube, faces, {}, p));
        const PatchParam r = parameterizePatch(cube, p);
        CHECK(!r.injective && r.needsSplit && r.method == ParamMethod::None);
    }
}

int main()
{
    testDistortion();
    testTutte();
    testCut();
    testPipelineLadder();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
