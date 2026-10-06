#include "core/BoundaryMetrics.h"
#include "core/Smoothing.h"
#include "param/PatchExtractor.h"
#include "TestUtil.h"

#include <random>

// n x n quads; z = slope * |x - n/2|: two planar halves meeting in a crease along x = n/2.
static TriMesh makeCreased(int n, double slope)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) v.push_back(m.add_vertex(Vec3(i, j, slope * std::abs(i - n / 2.0))));
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

static std::vector<int> halves(const TriMesh& m, double xSplit)
{
    std::vector<int> label(m.n_faces());
    for (auto f : m.faces()) {
        double x = 0;
        for (auto v : m.fv_range(f)) x += m.point(v)[0];
        label[f.idx()] = (x / 3.0 < xSplit) ? 0 : 1;
    }
    return label;
}

// Per-region (euler, loops) of the patches built without merging or cutting.
static std::vector<std::pair<int, int>> regionTopology(const TriMesh& m, const std::vector<int>& label)
{
    PatchOptions o;
    o.minFaces = 1;
    o.cutMultiBoundary = false;
    const ExtractionResult r = extractPatches(m, label, o);
    std::vector<std::pair<int, int>> t;
    for (const Patch& p : r.patches) t.push_back({p.topology.euler, p.topology.boundaryLoops});
    return t;
}

static void testStats()
{
    const TriMesh m = makeCreased(12, 0.6);
    const std::vector<int> clean = halves(m, 6.0);
    BoundaryStats s = computeBoundaryStats(m, clean);
    CHECK(s.teeth == 0 && s.spikeVertices == 0 && s.reflexVertices == 0);
    CHECK(s.boundaryEdges == 12);
    CHECK_NEAR(s.perimeter, 12.0 * 1.0, 1e-12);   // the crease edges run along y

    // one tooth: a single triangle on the other side of the border
    std::vector<int> bad = clean;
    bad[2 * (5 * 12 + 5) + 1] = 1;   // upper triangle of quad (5,5), left of the border
    s = computeBoundaryStats(m, bad);
    CHECK(s.teeth >= 1);
    CHECK(s.perimeter > 12.0);
    CHECK(s.spikeVertices + s.reflexVertices >= 1);
}

static void testRepairsNoise()
{
    const Fixture fx(makeCreased(24, 0.6));
    const VSAInput in = fx.input();
    const std::vector<int> clean = halves(fx.mesh, 12.0);

    // two exact proxies (one per plane) and a noisy labeling around the crease
    VSAState s;
    s.metric = Metric::L21;
    setSeeds(in, s, {2 * (10 * 24 + 3), 2 * (10 * 24 + 20)});
    s.label = clean;
    fitProxies(in, s);
    std::mt19937 rng(5);
    std::bernoulli_distribution flip(0.12);   // sparse single-triangle teeth, as flooding produces
    std::vector<int> noisy = clean;
    for (auto f : fx.mesh.faces()) {
        double x = 0;
        for (auto v : fx.mesh.fv_range(f)) x += fx.mesh.point(v)[0];
        if (std::abs(x / 3.0 - 12.0) < 1.0 && flip(rng)) noisy[f.idx()] = 1 - noisy[f.idx()];
    }
    s.label = noisy;
    recomputeRegions(in, s);

    const BoundaryStats before = computeBoundaryStats(fx.mesh, s.label);
    const double eBefore = totalEnergy(s);
    CHECK(before.teeth > 10 && eBefore > 0.0);

    s.smooth.lambda = 0.05;
    const SmoothStats st = smoothLabels(in, s);
    const BoundaryStats after = computeBoundaryStats(fx.mesh, s.label);
    CHECK(st.flips > 0);
    CHECK(after.teeth < before.teeth / 4);
    CHECK(after.perimeter < before.perimeter);
    CHECK(totalEnergy(s) <= eBefore + 1e-12);
    CHECK(allLabeled(s) && regionsConnected(fx.mesh, s));

    int wrongBefore = 0, wrongAfter = 0;
    for (size_t f = 0; f < clean.size(); ++f) {
        wrongBefore += noisy[f] != clean[f];
        wrongAfter += s.label[f] != clean[f];
    }
    CHECK(wrongAfter < wrongBefore / 3);

    // off switch
    VSAState off = s;
    off.label = noisy;
    off.smooth.lambda = 0.0;
    CHECK(smoothLabels(in, off).flips == 0 && off.label == noisy);
}

static void testPerfectPartitionUntouched()
{
    const Fixture fx = makeCubeFixture();
    VSAState s;
    setSeeds(fx.input(), s, {0, 2, 4, 6, 8, 10});
    partition(fx.input(), s);
    fitProxies(fx.input(), s);
    const std::vector<int> before = s.label;
    s.smooth.lambda = 5.0;
    CHECK(smoothLabels(fx.input(), s).flips == 0);
    CHECK(s.label == before);
}

static void testGuardsKeepTopology()
{
    const Fixture fx(makeBump(26));
    const VSAInput in = fx.input();
    for (Metric me : {Metric::L2, Metric::L21})
        for (uint64_t seed = 1; seed <= 4; ++seed) {
            VSAState s;
            s.metric = me;
            initRandomSeeds(in, s, 9, seed);
            runLloyd(in, s);
            const auto topoBefore = regionTopology(fx.mesh, s.label);
            const int k = s.k();

            s.smooth.lambda = 50.0;     // far beyond any sensible value: borders are pulled as straight as possible
            s.smooth.kappa = 1.0;
            const SmoothStats st = smoothLabels(in, s);
            CHECK(st.flips >= 0 && st.perimeterAfter <= st.perimeterBefore + 1e-12);
            CHECK(s.k() == k && allLabeled(s) && regionsConnected(fx.mesh, s));
            // the faces moved, but every region kept its Euler characteristic and its boundary loops
            CHECK(regionTopology(fx.mesh, s.label) == topoBefore);
        }
}

static void testLloydWithSmoothing()
{
    const Fixture fx(makeBump(30));
    const VSAInput in = fx.input();
    int teethPlain = 0, teethSmooth = 0;
    double perimPlain = 0, perimSmooth = 0;
    for (Metric me : {Metric::L2, Metric::L21})
        for (uint64_t seed = 1; seed <= 6; ++seed) {
            VSAState a, b;
            a.metric = b.metric = me;
            initRandomSeeds(in, a, 8, seed);
            initRandomSeeds(in, b, 8, seed);
            b.smooth.lambda = 0.1;
            b.smooth.kappa = 1.0;
            runLloyd(in, a);
            const LloydResult r = runLloyd(in, b);
            CHECK(r.iterations <= 100);
            CHECK(allLabeled(b) && regionsConnected(fx.mesh, b));

            const BoundaryStats sa = computeBoundaryStats(fx.mesh, a.label);
            const BoundaryStats sb = computeBoundaryStats(fx.mesh, b.label);
            teethPlain += sa.teeth;
            teethSmooth += sb.teeth;
            perimPlain += sa.perimeter;
            perimSmooth += sb.perimeter;
        }
    std::printf("teeth %d -> %d, perimeter %.2f -> %.2f\n", teethPlain, teethSmooth, perimPlain, perimSmooth);
    CHECK(teethSmooth < teethPlain);
    CHECK(perimSmooth < perimPlain);
}

static void testEdgeWeights()
{
    const Fixture fx(makeCreased(12, 0.6));
    const VSAInput in = fx.input();
    const std::vector<double> plain = computeEdgeWeights(in, 0.0);
    const std::vector<double> crease = computeEdgeWeights(in, 2.0);
    bool lighter = false;
    for (auto e : fx.mesh.edges()) {
        CHECK_NEAR(plain[e.idx()], fx.mesh.calc_edge_length(e), 1e-12);
        CHECK(crease[e.idx()] <= plain[e.idx()] + 1e-12);
        if (crease[e.idx()] < plain[e.idx()] - 1e-6) lighter = true;
    }
    CHECK(lighter);   // the edges on the crease
    const std::vector<int> label = halves(fx.mesh, 6.0);
    CHECK_NEAR(weightedPerimeter(fx.mesh, label, plain), computeBoundaryStats(fx.mesh, label).perimeter, 1e-12);
    CHECK(weightedPerimeter(fx.mesh, label, crease) < weightedPerimeter(fx.mesh, label, plain));
}

int main(int argc, char** argv)
{
    testStats();
    testEdgeWeights();
    testRepairsNoise();
    testPerfectPartitionUntouched();
    testGuardsKeepTopology();
    testLloydWithSmoothing();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
