#include "TestUtil.h"
#include "param/AnglesToUV.h"
#include "param/Injectivity.h"
#include "param/LinABF.h"
#include "param/Predicates.h"

#include <random>

static const double kPi = 3.14159265358979323846;

// ---- predicates ---------------------------------------------------------------------------------
// Points on the grid k / 2^30 make every orientation determinant an integer / 2^60, computed exactly
// in 128-bit integers, while the products need up to 62 bits: plain doubles cannot represent them.
static int exactSign(const long long a[2], const long long b[2], const long long c[2])
{
    const __int128 d = (__int128)(a[0] - c[0]) * (b[1] - c[1]) - (__int128)(a[1] - c[1]) * (b[0] - c[0]);
    return d > 0 ? 1 : (d < 0 ? -1 : 0);
}

static void testPredicates()
{
    const double inv = 1.0 / double(1 << 30);
    std::mt19937_64 rng(1);
    std::uniform_int_distribution<long long> origin(-(1LL << 29), 1LL << 29), step(-(1LL << 15), 1LL << 15),
        mult(-(1LL << 14), 1LL << 14);
    int naiveWrong = 0, zeros = 0;
    for (int it = 0; it < 20000; ++it) {
        // three points on one line a + k s (exactly collinear), the third nudged by one grid step half of the time
        const long long a[2] = {origin(rng), origin(rng)}, st[2] = {step(rng), step(rng)};
        const long long k1 = mult(rng), k2 = mult(rng);
        const long long b[2] = {a[0] + k1 * st[0], a[1] + k1 * st[1]};
        long long c[2] = {a[0] + k2 * st[0], a[1] + k2 * st[1]};
        if (it % 2) c[1] += (it % 4 == 1 ? 1 : -1);

        const double pa[2] = {a[0] * inv, a[1] * inv}, pb[2] = {b[0] * inv, b[1] * inv}, pc[2] = {c[0] * inv, c[1] * inv};
        const int want = exactSign(a, b, c);
        zeros += want == 0;
        CHECK(orient2dSign(pa, pb, pc) == want);
        const double ex = orient2dExact(pa, pb, pc);
        CHECK((ex > 0 ? 1 : ex < 0 ? -1 : 0) == want);

        const double naive = (pa[0] - pc[0]) * (pb[1] - pc[1]) - (pa[1] - pc[1]) * (pb[0] - pc[0]);
        naiveWrong += (naive > 0 ? 1 : naive < 0 ? -1 : 0) != want;
    }
    CHECK(zeros > 100);        // exactly collinear configurations occur
    CHECK(naiveWrong > 50);    // and the plain double formula does get them wrong

    const double o[2] = {0, 0}, x[2] = {1, 0}, y[2] = {0, 1};
    CHECK(orient2d(o, x, y) > 0 && orient2d(o, y, x) < 0 && orient2d(o, x, x) == 0);
    const double h1[2] = {1e300, 1e300}, h2[2] = {-1e300, -1e300}, h3[2] = {0, 0};
    CHECK(orient2dSign(h1, h2, h3) == 0);
}

// ---- angles -------------------------------------------------------------------------------------
// A fan around vertex 0 whose ring alternates up and down: the wedge angles are about 104 degrees each.
static TriMesh makeWavyFan(int n, bool open)
{
    TriMesh m;
    const auto c = m.add_vertex(Vec3(0, 0, 0));
    std::vector<TriMesh::VertexHandle> ring;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * kPi * i / n;
        ring.push_back(m.add_vertex(Vec3(std::cos(a), std::sin(a), i % 2 ? 1.0 : -1.0)));
    }
    for (int i = 0; i < (open ? n - 1 : n); ++i) m.add_face(c, ring[i], ring[(i + 1) % n]);
    return m;
}

static void testAngleChecks()
{
    const TriMesh wavy = makeWavyFan(6, true);
    const std::vector<double> ang = originalAngles(wavy);
    double sum = 0.0;
    for (auto h : wavy.vih_range(TriMesh::VertexHandle(0)))
        if (cornerIndex(wavy, h) >= 0) sum += ang[cornerIndex(wavy, h)];
    CHECK(sum > 2.8 * kPi && sum < 3.0 * kPi);

    const AngleReport r = checkAngles(wavy, ang, 0.05);
    CHECK(r.badCorners.empty());
    CHECK(r.foldVertices.size() == 1 && r.foldVertices[0] == 0);
    CHECK_NEAR(r.foldSums[0], sum, 1e-12);

    // a closed fan: the center is interior, constraints of LinABF handle it, no fold is reported
    const TriMesh closed = makeWavyFan(6, false);
    CHECK(checkAngles(closed, originalAngles(closed), 0.05).foldVertices.empty());

    // a flat open fan is fine
    TriMesh flat;
    const auto c = flat.add_vertex(Vec3(0, 0, 0));
    std::vector<TriMesh::VertexHandle> ring;
    for (int i = 0; i < 6; ++i) ring.push_back(flat.add_vertex(Vec3(std::cos(i * 0.9), std::sin(i * 0.9), 0)));
    for (int i = 0; i < 5; ++i) flat.add_face(c, ring[i], ring[i + 1]);
    CHECK(checkAngles(flat, originalAngles(flat), 0.05).ok());

    // invalid angles
    std::vector<double> bad = originalAngles(flat);
    bad[0] = 0.0;
    bad[4] = kPi;
    bad[7] = -0.1;
    const AngleReport rb = checkAngles(flat, bad, 0.05);
    CHECK(rb.badCorners == std::vector<int>({0, 4, 7}));
}

// ---- UV -----------------------------------------------------------------------------------------
static TriMesh makeGrid(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) v.push_back(m.add_vertex(Vec3(i, j, 0)));
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

static std::vector<Eigen::Vector2d> planarUV(const TriMesh& m)
{
    std::vector<Eigen::Vector2d> uv;
    for (auto v : m.vertices()) uv.push_back({m.point(v)[0], m.point(v)[1]});
    return uv;
}

static void testUVChecks()
{
    const TriMesh g = makeGrid(4);
    std::vector<Eigen::Vector2d> uv = planarUV(g);
    CHECK(checkUV(g, uv).ok());

    // pull an interior vertex across a neighbour: flipped triangles, no boundary problem
    std::vector<Eigen::Vector2d> flipped = uv;
    flipped[2 * 5 + 2] = Eigen::Vector2d(0.2, 0.2);   // vertex (2, 2) moves close to the corner region
    const UVReport rf = checkUV(g, flipped);
    CHECK(!rf.flippedFaces.empty() && rf.crossings.empty());

    // a degenerate (zero area) triangle counts as flipped
    std::vector<Eigen::Vector2d> degenerate = uv;
    degenerate[6] = degenerate[7];
    CHECK(!checkUV(g, degenerate).flippedFaces.empty());

    // boundary vertex pushed through the opposite side: the boundary crosses itself
    std::vector<Eigen::Vector2d> crossing = uv;
    crossing[0] = Eigen::Vector2d(5.5, 5.5);   // corner (0,0) jumps beyond the far corner
    const UVReport rc = checkUV(g, crossing);
    CHECK(!rc.crossings.empty());

    // a boundary vertex placed exactly on another boundary edge touches it
    std::vector<Eigen::Vector2d> touch = uv;
    touch[0] = Eigen::Vector2d(2.0, 4.0);   // lands on the top side between (1,4) and (3,4)
    CHECK(!boundaryCrossings(g, touch).empty());

    // adjacent boundary edges folded back onto each other overlap along a line
    std::vector<Eigen::Vector2d> spike = uv;
    spike[1] = Eigen::Vector2d(-1.0, 0.0);   // corner (0,0) (vertex 0) sits between (-1,0) and (2,0): collinear, same side? 
    spike[0] = Eigen::Vector2d(3.0, 0.0);    // vertex 0 beyond vertex 2: edges 0-1 and 0-5 ... see below
    // (vertex ids: 0 -> (0,0), 1 -> (1,0), 5 -> (0,1)); edge 0-1 now runs from (3,0) to (-1,0) and passes vertex 2
    CHECK(!boundaryCrossings(g, spike).empty());
}

static void testWavyFanOverlaps()
{
    const TriMesh wavy = makeWavyFan(6, true);
    const std::vector<double> ang = originalAngles(wavy);
    const UVResult r = anglesToUV(wavy, ang);
    CHECK(r.ok);
    // every triangle is positively oriented, the fan nevertheless winds 2.9 pi around the center
    UVStats st = analyzeUV(wavy, r.uv, ang);
    const UVReport rep = checkUV(wavy, r.uv);
    CHECK(rep.flippedFaces.empty());
    CHECK(!rep.crossings.empty());   // only the boundary test sees the global overlap
    (void)st;
}

static void testActiveSetRepairsFold()
{
    const TriMesh wavy = makeWavyFan(6, true);

    // plain LinABF leaves the boundary fan alone: the fold and the overlap stay
    const LinABFResult plain = solveLinABF(wavy);
    CHECK(plain.ok);
    CHECK(!checkAngles(wavy, plain.angles, 0.05).foldVertices.empty());

    ActiveSetParams ap;
    ActiveSetInfo info;
    const LinABFResult fixed = solveLinABFActiveSet(wavy, {}, ap, &info);
    CHECK(fixed.ok && fixed.anglesValid);
    CHECK(info.converged && info.constrainedVertices == 1 && info.rounds >= 1);
    CHECK(checkAngles(wavy, fixed.angles, ap.foldDelta - 1e-9).ok());
    CHECK(fixed.triangleResidual < 1e-10);

    const UVResult uv = anglesToUV(wavy, fixed.angles);
    CHECK(uv.ok);
    CHECK(checkUV(wavy, uv.uv).ok());   // no flips, no boundary crossings any more

    // a mesh that needs nothing is passed through untouched
    const TriMesh g = makeGrid(4);
    ActiveSetInfo ig;
    const LinABFResult rg = solveLinABFActiveSet(g, {}, ap, &ig);
    CHECK(rg.ok && ig.converged && ig.rounds == 0 && ig.fixedCorners == 0 && ig.constrainedVertices == 0);
}

int main()
{
    testPredicates();
    testAngleChecks();
    testUVChecks();
    testWavyFanOverlaps();
    testActiveSetRepairsFold();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
