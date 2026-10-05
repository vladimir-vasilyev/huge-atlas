#include "core/MeshPrep.h"
#include "core/Proxy.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <random>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)
#define CHECK_NEAR(a, b, tol) do { double va_ = (a), vb_ = (b); \
    if (!(std::abs(va_ - vb_) <= (tol))) { std::printf("FAIL %s:%d  %s = %.15g, %s = %.15g\n", \
    __FILE__, __LINE__, #a, va_, #b, vb_); ++failures; } } while (0)

using Tri = std::array<EVec3, 3>;

// ---- numeric integration: recursive 4-way subdivision (depth -> 4^depth triangles)
template <class F>
static void subdivide(const Tri& t, int depth, F&& leaf)
{
    if (depth == 0) { leaf(t); return; }
    const EVec3 ab = 0.5 * (t[0] + t[1]), bc = 0.5 * (t[1] + t[2]), ca = 0.5 * (t[2] + t[0]);
    subdivide({t[0], ab, ca}, depth - 1, leaf);
    subdivide({ab, t[1], bc}, depth - 1, leaf);
    subdivide({ca, bc, t[2]}, depth - 1, leaf);
    subdivide({ab, bc, ca}, depth - 1, leaf);
}

static double triArea(const Tri& t) { return 0.5 * (t[1] - t[0]).cross(t[2] - t[0]).norm(); }

// Integral of g(x) over the triangle. useMidpoints: edge-midpoint rule (exact for degree 2),
// otherwise the centroid rule (error O(h^2)).
template <class G>
static double integrate(const Tri& t, int depth, bool useMidpoints, G&& g)
{
    double sum = 0.0;
    subdivide(t, depth, [&](const Tri& s) {
        const double a = triArea(s);
        if (useMidpoints)
            sum += a / 3.0 * (g(0.5 * (s[0] + s[1])) + g(0.5 * (s[1] + s[2])) + g(0.5 * (s[2] + s[0])));
        else
            sum += a * g((s[0] + s[1] + s[2]) / 3.0);
    });
    return sum;
}

static Moments numericMoments(const Tri& t, const EVec3& n, int depth, bool mid)
{
    Moments m;
    m.area = integrate(t, depth, mid, [](const EVec3&) { return 1.0; });
    for (int i = 0; i < 3; ++i) {
        m.first[i] = integrate(t, depth, mid, [i](const EVec3& x) { return x[i]; });
        for (int j = 0; j < 3; ++j)
            m.second(i, j) = integrate(t, depth, mid, [i, j](const EVec3& x) { return x[i] * x[j]; });
    }
    m.areaNormal = m.area * n;
    return m;
}

static double maxDiff(const Moments& a, const Moments& b)
{
    return std::max({std::abs(a.area - b.area), (a.first - b.first).cwiseAbs().maxCoeff(),
                     (a.second - b.second).cwiseAbs().maxCoeff()});
}

static EVec3 unitNormal(const Tri& t) { return (t[1] - t[0]).cross(t[2] - t[0]).normalized(); }

static void testMomentsAgainstQuadrature()
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    auto rnd = [&] { return EVec3(U(rng), U(rng), U(rng)); };

    for (int k = 0; k < 20; ++k) {
        const Tri t = {rnd(), rnd(), rnd()};
        const EVec3 n = unitNormal(t);
        const Moments ex = triangleMoments(t[0], t[1], t[2], n);

        // midpoint rule is exact for quadratics: agreement to rounding
        CHECK(maxDiff(ex, numericMoments(t, n, 1, true)) < 1e-13);

        // centroid rule converges with order 2: error shrinks ~4x per level
        const double e4 = maxDiff(ex, numericMoments(t, n, 4, false));
        const double e5 = maxDiff(ex, numericMoments(t, n, 5, false));
        CHECK(e5 < 2e-3);
        CHECK(e5 < 0.35 * e4);
        CHECK_NEAR((ex.areaNormal - ex.area * n).norm(), 0.0, 1e-15);
    }
}

static void testErrorAgainstQuadrature()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    auto rnd = [&] { return EVec3(U(rng), U(rng), U(rng)); };

    for (int k = 0; k < 20; ++k) {
        const Tri t = {rnd(), rnd(), rnd()};
        const EVec3 n = unitNormal(t);
        const Moments m = triangleMoments(t[0], t[1], t[2], n);
        Proxy p;
        p.X = rnd();
        p.N = rnd().normalized();

        const double l2 = integrate(t, 1, true, [&](const EVec3& x) {
            const double d = p.N.dot(x - p.X);
            return d * d;
        });
        CHECK_NEAR(metricError(Metric::L2, m, p), l2, 1e-13);

        const double l21 = triArea(t) * (n - p.N).squaredNorm();
        CHECK_NEAR(metricError(Metric::L21, m, p), l21, 1e-13);
    }
}

static void testKnownValues()
{
    // right triangle in z = 0, area 1/2
    const Tri t = {EVec3(0, 0, 0), EVec3(1, 0, 0), EVec3(0, 1, 0)};
    const Moments m = triangleMoments(t[0], t[1], t[2], EVec3::UnitZ());
    CHECK_NEAR(m.area, 0.5, 1e-15);

    Proxy p;
    p.X = EVec3(0.3, 0.3, 0.2);
    p.N = EVec3::UnitZ();
    CHECK_NEAR(metricError(Metric::L2, m, p), 0.5 * 0.2 * 0.2, 1e-15);    // h^2 * area
    CHECK_NEAR(metricError(Metric::L21, m, p), 0.0, 1e-15);

    p.N = EVec3::UnitX();                                                  // orthogonal normals
    CHECK_NEAR(metricError(Metric::L21, m, p), 0.5 * 2.0, 1e-15);          // area * |n - N|^2 = area * 2
}

static void testAdditivity()
{
    const Tri a = {EVec3(0, 0, 0), EVec3(1, 0, 0), EVec3(0, 1, 0)};
    const Tri b = {EVec3(1, 0, 0), EVec3(1, 1, 0.3), EVec3(0, 1, 0)};
    const Moments ma = triangleMoments(a[0], a[1], a[2], unitNormal(a));
    const Moments mb = triangleMoments(b[0], b[1], b[2], unitNormal(b));
    const Moments sum = ma + mb;
    CHECK_NEAR(maxDiff(sum - mb, ma), 0.0, 1e-15);
    CHECK_NEAR(maxDiff(sum - ma, mb), 0.0, 1e-15);

    // error of a union equals the sum of the errors (same proxy)
    Proxy p;
    p.X = EVec3(0.2, 0.1, 0.0);
    p.N = EVec3(0.1, 0.2, 1.0).normalized();
    for (Metric me : {Metric::L2, Metric::L21})
        CHECK_NEAR(metricError(me, sum, p), metricError(me, ma, p) + metricError(me, mb, p), 1e-14);
}

// Total error of a region, summing per-triangle errors
static double sumFaceErrors(Metric me, const std::vector<Moments>& fm, const std::vector<int>& faces, const Proxy& p)
{
    double e = 0.0;
    for (int f : faces) e += metricError(me, fm[f], p);
    return e;
}

static void testFitMinimizes()
{
    // a smooth bump: non-trivial region, fitted proxy must beat random perturbations
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    std::vector<Tri> tris;
    const int n = 6;
    auto z = [](double x, double y) { return 0.3 * std::sin(2 * x) * std::cos(y); };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double x0 = -1 + 2.0 * i / n, x1 = -1 + 2.0 * (i + 1) / n;
            const double y0 = -1 + 2.0 * j / n, y1 = -1 + 2.0 * (j + 1) / n;
            const EVec3 p00(x0, y0, z(x0, y0)), p10(x1, y0, z(x1, y0)),
                        p11(x1, y1, z(x1, y1)), p01(x0, y1, z(x0, y1));
            tris.push_back({p00, p10, p11});
            tris.push_back({p00, p11, p01});
        }
    std::vector<Moments> fm;
    std::vector<int> ids;
    Moments region;
    for (const Tri& t : tris) {
        fm.push_back(triangleMoments(t[0], t[1], t[2], unitNormal(t)));
        ids.push_back(int(fm.size()) - 1);
        region += fm.back();
    }

    for (Metric me : {Metric::L2, Metric::L21}) {
        const Proxy best = fitProxy(me, region);
        CHECK_NEAR(best.N.norm(), 1.0, 1e-14);
        CHECK(best.N.dot(region.areaNormal) > 0.0);
        const double eBest = sumFaceErrors(me, fm, ids, best);
        CHECK_NEAR(eBest, metricError(me, region, best), 1e-13);   // O(1) region error matches the sum

        for (int k = 0; k < 200; ++k) {
            Proxy q = best;
            q.N = (best.N + 0.2 * EVec3(U(rng), U(rng), U(rng))).normalized();
            q.X = best.X + 0.2 * EVec3(U(rng), U(rng), U(rng));
            // L21 depends only on N; X is fixed at the centroid for both metrics
            if (me == Metric::L21) q.X = best.X;
            CHECK(sumFaceErrors(me, fm, ids, q) >= eBest - 1e-12);
        }
    }
}

static void testPlanarAndCube(const std::string& cubePath)
{
    // coplanar tilted patch: both metrics fit exactly, error 0
    const EVec3 n = EVec3(1, -2, 3).normalized();
    const EVec3 u = n.cross(EVec3::UnitX()).normalized(), v = n.cross(u);
    const EVec3 o(0.4, -0.2, 0.7);
    const Tri a = {o, o + u, o + v}, b = {o + u, o + u + v, o + v};
    Moments r = triangleMoments(a[0], a[1], a[2], unitNormal(a));
    r += triangleMoments(b[0], b[1], b[2], unitNormal(b));
    for (Metric me : {Metric::L2, Metric::L21}) {
        const Proxy p = fitProxy(me, r);
        CHECK_NEAR((p.N - n).norm(), 0.0, 1e-12);
        CHECK_NEAR(metricError(me, r, p), 0.0, 1e-12);
    }

    // cube: one face is exact; the whole closed surface has zero normal sum (L21 falls back to L2)
    TriMesh m;
    CHECK(loadMesh(cubePath, m));
    normalizeToUnitBox(m);
    const FaceData fd = computeFaceData(m);
    const std::vector<Moments> fm = computeFaceMoments(m, fd);

    Moments all, top;
    for (auto f : m.faces()) {
        all += fm[f.idx()];
        if (fd.normal[f.idx()][2] > 0.99) top += fm[f.idx()];
    }
    CHECK_NEAR(top.area, 1.0, 1e-14);
    for (Metric me : {Metric::L2, Metric::L21}) {
        const Proxy p = fitProxy(me, top);
        CHECK_NEAR((p.N - EVec3::UnitZ()).norm(), 0.0, 1e-12);
        CHECK_NEAR(p.X[2], 0.5, 1e-14);
        CHECK_NEAR(metricError(me, top, p), 0.0, 1e-13);
    }
    CHECK_NEAR(all.areaNormal.norm(), 0.0, 1e-14);
    const Proxy pa = fitProxy(Metric::L21, all);
    CHECK_NEAR(pa.N.norm(), 1.0, 1e-12);
    CHECK(std::isfinite(metricError(Metric::L21, all, pa)));
    CHECK_NEAR(metricError(Metric::L21, all, pa), 2.0 * all.area, 1e-12);   // sum of normals cancels

    // empty region
    const Proxy pe = fitProxy(Metric::L2, Moments());
    CHECK_NEAR(pe.N.norm(), 1.0, 1e-15);
}

int main(int argc, char** argv)
{
    testMomentsAgainstQuadrature();
    testErrorAgainstQuadrature();
    testKnownValues();
    testAdditivity();
    testFitMinimizes();
    testPlanarAndCube(argc > 1 ? argv[1] : "data/cube.obj");

    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
