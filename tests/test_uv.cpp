#include "TestUtil.h"
#include "core/Operations.h"
#include "param/AnglesToUV.h"
#include "param/LinABF.h"
#include "param/PatchExtractor.h"

#include <algorithm>

static const double kPi = 3.14159265358979323846;

static TriMesh makeFlatGrid(int n, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(),
                            const Eigen::Vector3d& shift = Eigen::Vector3d::Zero())
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const Eigen::Vector3d p = rot * Eigen::Vector3d(i * 1.0, j * 0.7 + 0.1 * i, 0) + shift;
            v.push_back(m.add_vertex(Vec3(p.x(), p.y(), p.z())));
        }
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// All pairwise distances of the UV points equal those of the 3D points (an isometry up to a rigid motion).
static double maxDistanceError(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv)
{
    double worst = 0.0;
    const int n = int(m.n_vertices());
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const double d3 = (m.point(TriMesh::VertexHandle(i)) - m.point(TriMesh::VertexHandle(j))).norm();
            worst = std::max(worst, std::abs((uv[i] - uv[j]).norm() - d3));
        }
    return worst;
}

static void testCornerOrder()
{
    for (const std::array<double, 3> a : {std::array<double, 3>{0.5, 0.6, kPi - 1.1}, {1.0, 1.0, kPi - 2.0},
                                          {2.5, 0.3, kPi - 2.8}, {0.1, 1.5, kPi - 1.6}}) {
        const std::array<int, 3> o = uvCornerOrder(a[0], a[1], a[2]);
        // a permutation that keeps the cyclic (face) order and puts the largest sine last
        CHECK((o[1] == (o[0] + 1) % 3) && (o[2] == (o[1] + 1) % 3));
        CHECK(std::sin(a[o[2]]) >= std::sin(a[0]) - 1e-15 && std::sin(a[o[2]]) >= std::sin(a[1]) - 1e-15 &&
              std::sin(a[o[2]]) >= std::sin(a[2]) - 1e-15);
        CHECK(std::sin(a[o[1]]) / std::sin(a[o[2]]) <= 1.0 + 1e-15);   // the ratio in the equations is at most 1
    }
}

static void testFlatPatchesAreReproduced()
{
    Eigen::Matrix3d rot = (Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())).toRotationMatrix();
    for (const TriMesh& m : {makeFlatGrid(7), makeFlatGrid(7, rot, Eigen::Vector3d(3, -2, 5))}) {
        const std::vector<double> ang = originalAngles(m);

        const UVResult r = anglesToUV(m, ang);
        CHECK(r.ok);
        CHECK(r.pinnedA != r.pinnedB && r.pinnedA >= 0);
        CHECK(r.maxResidual < 1e-10);
        CHECK(maxDistanceError(m, r.uv) < 1e-9);
        const UVStats s = analyzeUV(m, r.uv, ang);
        CHECK(s.flipped == 0 && s.minArea > 0.0);
        CHECK(s.maxAngleDeviation < 1e-9);

        const UVResult g = anglesToUVGreedy(m, ang);
        CHECK(g.ok && g.maxResidual < 1e-9);
        CHECK(maxDistanceError(m, g.uv) < 1e-9);
    }
}

static void testLinabfAnglesGiveConsistentUV()
{
    // obtuse example: a fan far above its ring; LinABF angles must unfold without flips
    TriMesh m;
    const auto p1 = m.add_vertex(Vec3(3, 0, 1)), p2 = m.add_vertex(Vec3(1, 1, 0)),
               p3 = m.add_vertex(Vec3(0, 0, 0)), p4 = m.add_vertex(Vec3(1, -1, 0));
    m.add_face(p1, p2, p3);
    m.add_face(p1, p3, p4);
    m.add_face(p1, p4, p2);
    const LinABFResult lin = solveLinABF(m);
    CHECK(lin.ok && lin.anglesValid);

    const UVResult r = anglesToUV(m, lin.angles);
    CHECK(r.ok);
    CHECK(r.maxResidual < 1e-8);   // the angles are consistent, so the system has an exact solution
    const UVStats s = analyzeUV(m, r.uv, lin.angles);
    CHECK(s.flipped == 0);
    CHECK(s.maxAngleDeviation < 1e-7);
    // the three triangles tile the neighbourhood of p1: total area equals the area of the outer triangle
    CHECK_NEAR(s.totalArea, std::abs(signedArea(r.uv[1], r.uv[2], r.uv[3])), 1e-9);

    // the raw angles of this fan are inconsistent (deficit 4.56): the least squares cannot honor them
    const UVResult raw = anglesToUV(m, originalAngles(m));
    CHECK(raw.ok && raw.maxResidual > 1e-3);

    const UVResult gr = anglesToUVGreedy(m, lin.angles);
    CHECK(gr.ok && gr.maxResidual < 1e-8);
}

static void testCurvedPatches()
{
    const Fixture fx(makeBump(28));
    VSAState s;
    initFarthestSeeds(fx.input(), s, 5, 1);
    runLloyd(fx.input(), s);
    const ExtractionResult ex = extractPatches(fx.mesh, s.label);

    int checked = 0;
    for (const Patch& p : ex.patches) {
        if (!p.isDisk()) continue;
        const LinABFResult lin = solveLinABF(p.mesh);
        CHECK(lin.ok && lin.anglesValid);
        const UVResult r = anglesToUV(p.mesh, lin.angles);
        CHECK(r.ok);
        CHECK(r.maxResidual < 1e-6);
        const UVStats st = analyzeUV(p.mesh, r.uv, lin.angles);
        CHECK(st.flipped == 0);
        CHECK(st.maxAngleDeviation < 1e-5);
        ++checked;

        const UVResult g = anglesToUVGreedy(p.mesh, lin.angles);
        CHECK(g.ok && g.maxResidual < 1e-5);
    }
    CHECK(checked >= 1);
}

static void testSingleTriangleAndErrors()
{
    TriMesh t;
    const auto a = t.add_vertex(Vec3(0, 0, 0)), b = t.add_vertex(Vec3(2, 0, 0)), c = t.add_vertex(Vec3(0.5, 1, 0.3));
    t.add_face(a, b, c);
    const std::vector<double> ang = originalAngles(t);
    const UVResult r = anglesToUV(t, ang);
    CHECK(r.ok && r.maxResidual < 1e-12);
    CHECK(analyzeUV(t, r.uv, ang).flipped == 0);
    CHECK(maxDistanceError(t, r.uv) < 1e-12);

    TriMesh cube;
    addCube(cube, 0.0);
    const UVResult closed = anglesToUV(cube, originalAngles(cube));
    CHECK(!closed.ok && !closed.message.empty());      // no boundary: nothing to pin

    CHECK(!anglesToUV(t, std::vector<double>{1.0}).ok);   // wrong angle count
}

int main()
{
    testCornerOrder();
    testFlatPatchesAreReproduced();
    testLinabfAnglesGiveConsistentUV();
    testCurvedPatches();
    testSingleTriangleAndErrors();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
