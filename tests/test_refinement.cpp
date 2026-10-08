#include "core/Operations.h"
#include "TestUtil.h"
#include "param/Refinement.h"

#include <set>

static const double kPi = 3.14159265358979323846;

static TriMesh makeGrid(int n, double ox = 0.0)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) v.push_back(m.add_vertex(Vec3(ox + i * 1.0, j * 0.8 + 0.1 * i, 0)));
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// Strip on a cone winding `turns` times; it overlaps itself when unrolled if 2 pi turns sin(gamma) > 2 pi.
static void addConeStrip(TriMesh& m, double gamma, double turns, int segments, int rows, double ox = 0.0)
{
    std::vector<TriMesh::VertexHandle> v;
    for (int i = 0; i <= segments; ++i)
        for (int j = 0; j < rows; ++j) {
            const double phi = 2 * kPi * turns * i / segments, rho = 1.0 + 0.3 * j / (rows - 1);
            v.push_back(m.add_vertex(Vec3(ox + rho * std::sin(gamma) * std::cos(phi), rho * std::sin(gamma) * std::sin(phi),
                                          rho * std::cos(gamma))));
        }
    auto id = [rows](int i, int j) { return i * rows + j; };
    for (int i = 0; i < segments; ++i)
        for (int j = 0; j + 1 < rows; ++j) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
}

static TriMesh makeTorus(int n, int k)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < k; ++j) {
            const double th = 2 * kPi * i / n, ph = 2 * kPi * j / k;
            v.push_back(m.add_vertex(Vec3((2 + std::cos(ph)) * std::cos(th), (2 + std::cos(ph)) * std::sin(th), std::sin(ph))));
        }
    auto id = [&](int i, int j) { return (i % n) * k + (j % k); };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < k; ++j) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// Every face belongs to exactly one patch and the patch maps point back to the faces.
static void checkConsistency(const TriMesh& m, const RefineResult& r)
{
    std::vector<int> seen(m.n_faces(), 0);
    for (size_t pi = 0; pi < r.extraction.patches.size(); ++pi)
        for (int f : r.extraction.patches[pi].faceToGlobal) {
            ++seen[f];
            CHECK(r.labels[f] == int(pi));
        }
    for (int c : seen) CHECK(c == 1);
    CHECK(r.params.size() == r.extraction.patches.size() && r.bad.size() == r.params.size());
}

static void checkAllGood(const RefineResult& r)
{
    CHECK(r.stats.unresolved == 0);
    for (size_t i = 0; i < r.params.size(); ++i) {
        CHECK(!r.bad[i]);
        CHECK(r.extraction.patches[i].isDisk());
        CHECK(r.params[i].injective && !r.params[i].needsSplit);
        CHECK(checkUV(r.params[i].patch.mesh, r.params[i].uv).ok());   // verified again from scratch
    }
}

static void testConeStripIsSplit()
{
    TriMesh m;
    addConeStrip(m, 1.05, 1.5, 90, 4);   // 60 degrees: unrolls to 8.2 rad > 2 pi
    const std::vector<int> one(m.n_faces(), 0);

    const RefineResult r = refinePatches(m, one);
    checkConsistency(m, r);
    checkAllGood(r);
    CHECK(r.extraction.patches.size() >= 2);
    CHECK(r.stats.splits >= 1 && r.stats.initialBad == 1);
    // the split starts from the faces at the crossing, from the two ends of the patch or from the worst fitted faces:
    // whichever start gives the lowest energy wins
    CHECK(r.stats.splitsByCrossingHint + r.stats.splitsByFarthest + r.stats.splitsByNormals >= 1);
    for (const PatchParam& p : r.params) CHECK(p.badAreaFraction <= 0.005);

    // policy: split right away, no repair ladder: every final patch is flattened by plain LinABF
    RefineOptions imm;
    imm.policy = RefinePolicy::SplitImmediately;
    const RefineResult ri = refinePatches(m, one, imm);
    checkConsistency(m, ri);
    checkAllGood(ri);
    for (const PatchParam& p : ri.params) CHECK(p.method == ParamMethod::LinABF);
}

static void testNothingToDo()
{
    const TriMesh g = makeGrid(8);
    const std::vector<int> one(g.n_faces(), 0);
    const RefineResult r = refinePatches(g, one);
    checkConsistency(g, r);
    checkAllGood(r);
    CHECK(r.stats.splits == 0 && r.stats.rounds == 1 && r.stats.initialBad == 0);
    CHECK(r.extraction.patches.size() == 1);
}

static void testNeighboursAreNotTouched()
{
    TriMesh m = makeGrid(6, 0.0);
    const int gridFaces = int(m.n_faces());
    addConeStrip(m, 1.05, 1.5, 90, 4, 50.0);   // a separate component far away
    std::vector<int> label(m.n_faces(), 1);
    for (int f = 0; f < gridFaces; ++f) label[f] = 0;

    const RefineResult r = refinePatches(m, label);
    checkConsistency(m, r);
    checkAllGood(r);
    const int gridPatch = r.labels[0];
    for (int f = 0; f < gridFaces; ++f) CHECK(r.labels[f] == gridPatch);
    CHECK(int(r.extraction.patches[gridPatch].faceToGlobal.size()) == gridFaces);   // exactly the old faces
    CHECK(r.extraction.patches.size() >= 3);                                          // the strip was split
}

static void testLimits()
{
    TriMesh m;
    addConeStrip(m, 1.05, 1.5, 90, 4);
    const std::vector<int> one(m.n_faces(), 0);

    RefineOptions noDepth;
    noDepth.maxDepth = 0;
    RefineResult r = refinePatches(m, one, noDepth);
    CHECK(r.stats.splits == 0 && r.stats.unresolved == 1 && r.extraction.patches.size() == 1 && r.bad[0]);

    RefineOptions bigMin;
    bigMin.minFaces = 10000;
    r = refinePatches(m, one, bigMin);
    CHECK(r.stats.splits == 0 && r.stats.unresolved == 1);

    RefineOptions oneRound;
    oneRound.maxRounds = 1;
    r = refinePatches(m, one, oneRound);
    checkConsistency(m, r);
    CHECK(r.stats.rounds <= 2);

    // stepwise use gives the same result as the one-shot call
    Refiner step(m, one, RefineOptions());
    int rounds = 0;
    while (step.step()) ++rounds;
    CHECK(step.finished());
    const RefineResult full = refinePatches(m, one);
    CHECK(step.result().labels == full.labels && step.result().stats.splits == full.stats.splits);
    CHECK(rounds >= 1);
}

static void testTorusIsCutIntoDisks()
{
    const TriMesh t = makeTorus(24, 12);
    const std::vector<int> one(t.n_faces(), 0);
    const RefineResult r = refinePatches(t, one);
    checkConsistency(t, r);
    checkAllGood(r);
    CHECK(r.extraction.patches.size() >= 2);
    CHECK(r.stats.initialBad == 1);
}

static void testBunny(const std::string& path)
{
    TriMesh m;
    CHECK(loadMesh(path, m));
    normalizeToUnitBox(m);
    const Fixture fx(std::move(m));
    VSAState s;
    initFarthestSeeds(fx.input(), s, 3, 1);
    s.smooth.lambda = 0.1;
    runLloyd(fx.input(), s);

    const RefineResult r = refinePatches(fx.mesh, s.label);
    checkConsistency(fx.mesh, r);
    std::printf("bunny k=3: %d bad at start, %zu patches, %d splits, %d unresolved, %d rounds, %.1fs\n",
                r.stats.initialBad, r.extraction.patches.size(), r.stats.splits, r.stats.unresolved, r.stats.rounds,
                r.stats.seconds);
    CHECK(r.stats.initialBad >= 1);
    CHECK(r.stats.splits >= 1);
    CHECK(r.stats.unresolved == 0);
    checkAllGood(r);
}

int main(int argc, char** argv)
{
    testConeStripIsSplit();
    testNothingToDo();
    testNeighboursAreNotTouched();
    testLimits();
    testTorusIsCutIntoDisks();
    if (argc > 1) testBunny(argv[1]);
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
