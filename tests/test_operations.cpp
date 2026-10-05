#include "core/Operations.h"
#include "TestUtil.h"

#include <algorithm>

// Six perfect cube regions (one per side), L2,1.
static VSAState cubeSixSides(const Fixture& fx, Metric me = Metric::L21)
{
    VSAState s;
    s.metric = me;
    setSeeds(fx.input(), s, {0, 2, 4, 6, 8, 10});
    partition(fx.input(), s);
    fitProxies(fx.input(), s);
    return s;
}

static double sumAreas(const VSAState& s)
{
    double a = 0.0;
    for (const Moments& m : s.regions) a += m.area;
    return a;
}

static void testWeights()
{
    const Fixture fx = makeCubeFixture();
    Moments m;
    for (const Moments& f : fx.fm) m += f;
    Moments w3 = m;
    w3 *= 3.0;
    CHECK_NEAR(w3.area, 3.0 * m.area, 1e-12);
    Proxy p;
    p.X = EVec3(0.1, 0.2, 0.3);
    p.N = EVec3(1, 2, 3).normalized();
    for (Metric me : {Metric::L2, Metric::L21})
        CHECK_NEAR(metricError(me, w3, p), 3.0 * metricError(me, m, p), 1e-11);
    // a uniform weight does not move the fitted proxy
    Moments top;
    for (auto f : fx.mesh.faces()) if (fx.fd.normal[f.idx()][2] > 0.99) top += fx.fm[f.idx()];
    Moments top5 = top;
    top5 *= 5.0;
    CHECK_NEAR((fitProxy(Metric::L2, top).N - fitProxy(Metric::L2, top5).N).norm(), 0.0, 1e-12);
}

static void testMergeCost()
{
    const Fixture fx = makeCubeFixture();
    const VSAInput in = fx.input();
    VSAState s = cubeSixSides(fx);
    RegionPair m;
    CHECK(findBestMerge(in, s, m));
    CHECK(m.a < m.b);
    // two perpendicular sides of area 4: |sum of normals| = 4*sqrt(2), cost = 2 (8 - 4 sqrt 2)
    CHECK_NEAR(m.cost, 2.0 * (8.0 - 4.0 * std::sqrt(2.0)), 1e-12);

    // the cost agrees with really merging and refitting
    const double before = totalEnergy(s);
    CHECK(deleteProxy(in, s, m.a, m.b));
    CHECK_NEAR(totalEnergy(s) - before, m.cost, 1e-12);

    VSAState none;
    CHECK(!findBestMerge(in, none, m));   // not partitioned
}

static void testDeleteIndexing()
{
    const Fixture fx = makeCubeFixture();
    const VSAInput in = fx.input();
    // every ordered pair, in particular a == last and b == last
    for (int a = 0; a < 6; ++a)
        for (int b = 0; b < 6; ++b) {
            if (a == b) continue;
            VSAState s = cubeSixSides(fx);
            const double merged = s.regions[a].area + s.regions[b].area;
            // faces of the two regions before deletion
            std::vector<int> fa, fb;
            for (size_t f = 0; f < s.label.size(); ++f) {
                if (s.label[f] == a) fa.push_back(int(f));
                if (s.label[f] == b) fb.push_back(int(f));
            }
            CHECK(deleteProxy(in, s, a, b));
            CHECK(s.k() == 5 && s.regions.size() == 5);
            CHECK(allLabeled(s));
            CHECK_NEAR(sumAreas(s), 24.0, 1e-12);
            // all faces of a and b share one label that holds their union
            const int l = s.label[fa[0]];
            for (int f : fa) CHECK(s.label[f] == l);
            for (int f : fb) CHECK(s.label[f] == l);
            CHECK_NEAR(s.regions[l].area, merged, 1e-12);
            CHECK(s.proxies[l].N.norm() > 0.99);
        }
    VSAState s = cubeSixSides(fx);
    CHECK(!deleteProxy(in, s, 2, 2));
    CHECK(!deleteProxy(in, s, 0, 9));
}

static void testInsertAndGrow()
{
    const Fixture fx = makeCubeFixture();
    const VSAInput in = fx.input();

    // merge two sides, then insert: the new proxy lands in the merged (worst) region
    VSAState s = cubeSixSides(fx);
    RegionPair m;
    CHECK(findBestMerge(in, s, m));
    deleteProxy(in, s, m.a, m.b);
    const double e5 = totalEnergy(s);
    CHECK(e5 > 1.0);
    CHECK(insertProxy(in, s));
    CHECK(s.k() == 6);
    CHECK_NEAR(sumAreas(s), 24.0, 1e-12);
    runLloyd(in, s);
    CHECK_NEAR(totalEnergy(s), 0.0, 1e-12);   // back to six planar sides

    // nothing to insert into an exact partition with single-face regions only
    VSAState tiny;
    setSeeds(in, tiny, {0});
    partition(in, tiny);
    CHECK(insertProxy(in, tiny) == (tiny.regions[0].area > 0));   // whole cube in one region: splittable

    // growing from one proxy: energy falls monotonically with k and ends exact at k = 6
    VSAState g;
    initFarthestSeeds(in, g, 1, 1);
    std::vector<double> e;
    runLloyd(in, g);
    e.push_back(totalEnergy(g));
    while (g.k() < 6 && insertProxy(in, g)) {
        runLloyd(in, g);
        e.push_back(totalEnergy(g));
    }
    CHECK(g.k() == 6);
    for (size_t i = 1; i < e.size(); ++i) CHECK(e[i] <= e[i - 1] + 1e-9);
    CHECK_NEAR(e.back(), 0.0, 1e-9);

    VSAState g2;
    initFarthestSeeds(in, g2, 1, 1);
    const LloydResult r = growToK(in, g2, 6);
    CHECK(g2.k() == 6 && r.iterations > 0);
    CHECK_NEAR(totalEnergy(g2), 0.0, 1e-9);
    // asking for more proxies than faces stops at the number of faces that can be split
    VSAState g3;
    initFarthestSeeds(in, g3, 1, 1);
    growToK(in, g3, 100);
    CHECK(g3.k() <= 12 && allLabeled(g3));
}

// flat half (z = 0) next to a strongly curved half
static TriMesh makeHalfFlatHalfWavy(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const double x = -1 + 2.0 * i / n, y = -1 + 2.0 * j / n;
            const double z = x > 0 ? 0.35 * std::sin(9 * x) * std::cos(5 * y) : 0.0;
            v.push_back(m.add_vertex(Vec3(x, y, z)));
        }
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

static void testTeleport()
{
    const Fixture fx(makeHalfFlatHalfWavy(24));
    const VSAInput in = fx.input();

    // five of six seeds on the flat half: four proxies are wasted there
    std::vector<int> seeds;
    for (auto f : fx.mesh.faces()) {
        const double x = fx.fd.centroid[f.idx()][0];
        if (x < -0.3 && seeds.size() < 5 && f.idx() % 37 == 0) seeds.push_back(f.idx());
    }
    for (auto f : fx.mesh.faces())
        if (fx.fd.centroid[f.idx()][0] > 0.5) { seeds.push_back(f.idx()); break; }
    CHECK(seeds.size() == 6);

    for (Metric me : {Metric::L21, Metric::L2}) {
        VSAState plain;
        plain.metric = me;
        setSeeds(in, plain, seeds);
        runLloyd(in, plain);

        VSAState tele;
        tele.metric = me;
        setSeeds(in, tele, seeds);
        const LloydResult r = runLloydTeleport(in, tele);
        CHECK(tele.k() == plain.k());
        CHECK(allLabeled(tele) && regionsConnected(fx.mesh, tele));
        CHECK(totalEnergy(tele) <= totalEnergy(plain) + 1e-12);   // accepted moves only improve
        CHECK(r.teleports >= 1);
        CHECK(totalEnergy(tele) < 0.9 * totalEnergy(plain));
    }

    // the acceptance rule: never worse than plain Lloyd, from any start
    const Fixture bump(makeBump(14));
    for (Metric me : {Metric::L2, Metric::L21})
        for (uint64_t seed = 1; seed <= 6; ++seed) {
            VSAState a, b;
            a.metric = b.metric = me;
            initRandomSeeds(bump.input(), a, 7, seed);
            initRandomSeeds(bump.input(), b, 7, seed);
            runLloyd(bump.input(), a);
            runLloydTeleport(bump.input(), b);
            CHECK(totalEnergy(b) <= totalEnergy(a) + 1e-12);
            CHECK(b.k() == 7);
        }

    // too few proxies to teleport
    VSAState two;
    setSeeds(in, two, {seeds[0], seeds[5]});
    runLloyd(in, two);
    CHECK(!teleportProxy(in, two));
}

static void testFarthest()
{
    // long thin strip: the second seed must be far from the first
    TriMesh strip;
    {
        const int n = 40;
        std::vector<TriMesh::VertexHandle> v;
        for (int j = 0; j <= 1; ++j)
            for (int i = 0; i <= n; ++i) v.push_back(strip.add_vertex(Vec3(i * 0.1, j * 0.1, 0)));
        for (int i = 0; i < n; ++i) {
            strip.add_face(v[i], v[i + 1], v[n + 1 + i + 1]);
            strip.add_face(v[i], v[n + 1 + i + 1], v[n + 1 + i]);
        }
    }
    const Fixture fs(std::move(strip));
    for (uint64_t seed = 1; seed <= 5; ++seed) {
        VSAState s;
        initFarthestSeeds(fs.input(), s, 2, seed);
        CHECK(s.k() == 2 && s.seedFace[0] != s.seedFace[1]);
        const double d = (fs.fd.centroid[s.seedFace[0]] - fs.fd.centroid[s.seedFace[1]]).norm();
        CHECK(d >= 2.0);   // strip length 4: farthest point is at least half the length away
    }

    // components: two cubes with k = 2 get one seed each
    TriMesh two;
    addCube(two, 0.0);
    addCube(two, 10.0);
    const Fixture f2(std::move(two));
    for (uint64_t seed = 1; seed <= 5; ++seed) {
        VSAState s;
        initFarthestSeeds(f2.input(), s, 2, seed);
        CHECK(s.k() == 2);
        CHECK((s.seedFace[0] < 12) != (s.seedFace[1] < 12));
        CHECK(partition(f2.input(), s) == 0);   // nothing left unseeded
    }

    // determinism, clamping, distinct seeds
    const Fixture cube = makeCubeFixture();
    VSAState a, b;
    initFarthestSeeds(cube.input(), a, 6, 4);
    initFarthestSeeds(cube.input(), b, 6, 4);
    CHECK(a.seedFace == b.seedFace);
    CHECK(std::set<int>(a.seedFace.begin(), a.seedFace.end()).size() == 6);
    initFarthestSeeds(cube.input(), a, 100, 4);
    CHECK(a.k() == 12);
    initFarthestSeeds(cube.input(), a, 0, 4);
    CHECK(a.k() == 1);
}

int main()
{
    testWeights();
    testMergeCost();
    testDeleteIndexing();
    testInsertAndGrow();
    testTeleport();
    testFarthest();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
