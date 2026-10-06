#include "TestUtil.h"
#include "param/LinABF.h"
#include "param/PatchExtractor.h"

#include <algorithm>
#include <cmath>
#include <random>

static const double kPi = 3.14159265358979323846;

// Planar fan: center vertex with n spokes, plus an outer ring; interior vertices: the center only.
static TriMesh makeFan(int n, double rOuter, double zBump = 0.0)
{
    TriMesh m;
    const auto c = m.add_vertex(Vec3(0, 0, zBump));
    std::vector<TriMesh::VertexHandle> ring;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * kPi * i / n;
        ring.push_back(m.add_vertex(Vec3(rOuter * std::cos(a), rOuter * std::sin(a), 0)));
    }
    for (int i = 0; i < n; ++i) m.add_face(c, ring[i], ring[(i + 1) % n]);
    return m;
}

// Planar triangulated grid (n x n quads): many interior vertices.
static TriMesh makeFlatGrid(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) v.push_back(m.add_vertex(Vec3(i * 1.0, j * 0.7 + 0.1 * i, 0)));
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

static void testFlatPatchIsUntouched()
{
    for (const TriMesh& m : {makeFlatGrid(6), makeFan(7, 2.0)}) {
        const LinABFResult r = solveLinABF(m);
        CHECK(r.ok && r.anglesValid);
        CHECK(r.fansRescaled == 0);
        const std::vector<double> orig = originalAngles(m);
        double maxE = 0.0;
        for (size_t i = 0; i < orig.size(); ++i) maxE = std::max(maxE, std::abs(r.angles[i] - orig[i]));
        CHECK(maxE < 1e-10);               // the original angles already satisfy every constraint
        CHECK(r.triangleResidual < 1e-12 && r.vertexResidual < 1e-12 && r.wheelResidual < 1e-12);
        CHECK(r.angularError < 1e-11);
        CHECK(r.rows == r.triangles + 2 * r.interiorVertices);
    }
}

// Fig. 4 of the paper: a fan around p1 that sits far above the triangle p2 p3 p4 (angular deficit ~4.56).
static TriMesh makeObtuse()
{
    TriMesh m;
    const auto p1 = m.add_vertex(Vec3(3, 0, 1)), p2 = m.add_vertex(Vec3(1, 1, 0)),
               p3 = m.add_vertex(Vec3(0, 0, 0)), p4 = m.add_vertex(Vec3(1, -1, 0));
    m.add_face(p1, p2, p3);
    m.add_face(p1, p3, p4);
    m.add_face(p1, p4, p2);
    return m;
}

static void testObtuseExample()
{
    const TriMesh m = makeObtuse();
    CHECK(m.n_faces() == 3);

    // the deficit of p1 (vertex 0)
    const std::vector<double> orig = originalAngles(m);
    double atP1 = 0.0;
    for (auto h : m.vih_range(TriMesh::VertexHandle(0))) atP1 += orig[cornerIndex(m, h)];
    CHECK_NEAR(2 * kPi - atP1, 4.56, 0.01);

    LinABFParams prm;
    prm.maxIterations = 1;                     // a single linear step, as in the paper
    const LinABFResult r = solveLinABF(m, prm);
    CHECK(r.ok && r.anglesValid);
    CHECK(r.iterations == 1);
    CHECK(r.fansRescaled == 1);                // the large deficit triggers the exponential-map estimation
    CHECK(r.triangleResidual < 1e-12 && r.vertexResidual < 1e-12);
    CHECK(r.wheelResidual < 1e-2);             // one linear step: the wheel condition holds approximately

    // another linearization step improves the nonlinear wheel residual
    prm.maxIterations = 4;
    const LinABFResult r4 = solveLinABF(m, prm);
    CHECK(r4.ok && r4.anglesValid);
    CHECK(r4.triangleResidual < 1e-12 && r4.vertexResidual < 1e-12);
    CHECK(r4.wheelResidual <= r.wheelResidual + 1e-15);

    // by default it iterates until the wheel condition holds to the tolerance
    const LinABFResult rd = solveLinABF(m);
    CHECK(rd.ok && rd.anglesValid && rd.wheelResidual <= 1e-8);

    // the sum of the three angles at p1 closes the fan
    double s = 0.0;
    for (auto h : m.vih_range(TriMesh::VertexHandle(0))) s += r.angles[cornerIndex(m, h)];
    CHECK_NEAR(s, 2 * kPi, 1e-10);

    // without the exponential-map fix the same system is still solvable but starts further away
    LinABFParams plain;
    plain.deficitThreshold = 100.0;
    const LinABFResult rp = solveLinABF(m, plain);
    CHECK(rp.fansRescaled == 0);
    CHECK(rp.ok);
}

static void testCurvedPatches()
{
    // disk patches of VSA partitions of a bump and of the hemisphere
    TriMesh bump = makeBump(26);
    const Fixture fx(std::move(bump));
    VSAState s;
    initRandomSeeds(fx.input(), s, 6, 3);
    runLloyd(fx.input(), s);
    const ExtractionResult ex = extractPatches(fx.mesh, s.label);

    int solved = 0;
    for (const Patch& p : ex.patches) {
        if (!p.isDisk()) continue;
        const LinABFResult r = solveLinABF(p.mesh);
        CHECK(r.ok);
        if (!r.ok) continue;
        ++solved;
        CHECK(r.anglesValid);
        CHECK(r.triangleResidual < 1e-10 && r.vertexResidual < 1e-10);
        CHECK(r.wheelResidual < 1e-3);
        CHECK(r.rows == r.triangles + 2 * r.interiorVertices);
        // every corner angle is a corner of a triangle: sums per triangle are pi
        for (auto f : p.mesh.faces()) {
            const double t = r.angles[3 * f.idx()] + r.angles[3 * f.idx() + 1] + r.angles[3 * f.idx() + 2];
            CHECK_NEAR(t, kPi, 1e-10);
        }
    }
    CHECK(solved >= 1);
}

static void testClosedMeshIsRankDeficient()
{
    TriMesh cube;
    addCube(cube, 0.0);
    const LinABFResult r = solveLinABF(cube);
    CHECK(!r.ok);
    CHECK(!r.message.empty());
}

static void testSingleTriangle()
{
    TriMesh m;
    const auto a = m.add_vertex(Vec3(0, 0, 0)), b = m.add_vertex(Vec3(2, 0, 0)), c = m.add_vertex(Vec3(0.5, 1, 0.3));
    m.add_face(a, b, c);
    const LinABFResult r = solveLinABF(m);
    CHECK(r.ok && r.anglesValid && r.interiorVertices == 0 && r.rows == 1);
    CHECK_NEAR(r.angles[0] + r.angles[1] + r.angles[2], kPi, 1e-12);
    const std::vector<double> orig = originalAngles(m);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(r.angles[i], orig[i], 1e-10);
}

// Interior vertices of different valences and a spike: the corrected angles must satisfy the constraints.
static bool needsMoreThanOneStep(int count) { return count >= 1; }

static void testRandomPerturbedFans()
{
    int neededMore = 0;   // trials where one linear step is not enough but the default iterations are
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> U(-1, 1);
    for (int trial = 0; trial < 20; ++trial) {
        const int n = 4 + trial % 6;
        TriMesh m = makeFan(n, 1.0 + 0.3 * U(rng), 0.8 * std::abs(U(rng)));   // lifted center: a cone
        for (auto v : m.vertices()) {
            Vec3 pt = m.point(v);
            if (v.idx() > 0) pt += Vec3(0.1 * U(rng), 0.1 * U(rng), 0.05 * U(rng));
            m.set_point(v, pt);
        }
        LinABFParams single;
        single.maxIterations = 1;
        const LinABFResult r1 = solveLinABF(m, single);
        const LinABFResult r = solveLinABF(m);
        CHECK(r.ok && r1.ok);
        if (!r.ok || !r1.ok) continue;
        CHECK(r.triangleResidual < 1e-10 && r.vertexResidual < 1e-10);
        if (r1.anglesValid) CHECK(r1.wheelResidual < 1e-2);
        if (r.anglesValid) {
            CHECK(r.wheelResidual <= 1e-8);
            if (r1.anglesValid && r1.wheelResidual > 1e-8) ++neededMore;
        }
    }
    CHECK(needsMoreThanOneStep(neededMore));
}

int main()
{
    testFlatPatchIsUntouched();
    testObtuseExample();
    testCurvedPatches();
    testClosedMeshIsRankDeficient();
    testSingleTriangle();
    testRandomPerturbedFans();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
