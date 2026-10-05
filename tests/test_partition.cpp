#include "core/MeshPrep.h"
#include "core/Partition.h"

#include <cmath>
#include <cstdio>
#include <queue>
#include <set>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)
#define CHECK_NEAR(a, b, tol) do { double va_ = (a), vb_ = (b); \
    if (!(std::abs(va_ - vb_) <= (tol))) { std::printf("FAIL %s:%d  %s = %.15g, %s = %.15g\n", \
    __FILE__, __LINE__, #a, va_, #b, vb_); ++failures; } } while (0)

// ---- mesh builders
static void addCube(TriMesh& m, double ox)
{
    static const int p[8][3] = {{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
    static const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                                 {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
    TriMesh::VertexHandle v[8];
    for (int i = 0; i < 8; ++i) v[i] = m.add_vertex(Vec3(p[i][0] + ox, p[i][1], p[i][2]));
    for (auto& t : f) m.add_face(v[t[0]], v[t[1]], v[t[2]]);
}

static TriMesh makeBump(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const double x = -1 + 2.0 * i / n, y = -1 + 2.0 * j / n;
            v.push_back(m.add_vertex(Vec3(x, y, 0.4 * std::sin(3 * x) * std::cos(2 * y))));
        }
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// Everything the algorithm needs, bundled for a test.
struct Fixture {
    TriMesh mesh;
    FaceData fd;
    std::vector<Moments> fm;
    explicit Fixture(TriMesh m) : mesh(std::move(m))
    {
        fd = computeFaceData(mesh);
        fm = computeFaceMoments(mesh, fd);
    }
    VSAInput input() const { return {mesh, fd, fm}; }
};

// Every region must be non-empty and connected through shared edges.
static bool regionsConnected(const TriMesh& m, const VSAState& s)
{
    for (int r = 0; r < s.k(); ++r) {
        int start = -1, count = 0;
        for (size_t f = 0; f < s.label.size(); ++f)
            if (s.label[f] == r) { ++count; if (start < 0) start = int(f); }
        if (count == 0) return false;

        std::vector<char> seen(s.label.size(), 0);
        std::queue<int> q;
        q.push(start);
        seen[start] = 1;
        int reached = 0;
        while (!q.empty()) {
            const int f = q.front(); q.pop();
            ++reached;
            for (auto nb : m.ff_range(TriMesh::FaceHandle(f)))
                if (!seen[nb.idx()] && s.label[nb.idx()] == r) { seen[nb.idx()] = 1; q.push(nb.idx()); }
        }
        if (reached != count) return false;
    }
    return true;
}

static bool allLabeled(const VSAState& s)
{
    for (int l : s.label) if (l < 0 || l >= s.k()) return false;
    return true;
}

static void testCubeSixSeeds()
{
    Fixture fx(TriMesh{});
    addCube(fx.mesh, 0.0);
    fx = Fixture(std::move(fx.mesh));

    VSAState s;
    s.metric = Metric::L21;
    // one seed per cube face: faces 0,2,4,6,8,10 are on six different sides
    s.seedFace = {0, 2, 4, 6, 8, 10};
    for (int f : s.seedFace) s.proxies.push_back(fitProxy(s.metric, fx.fm[f]));

    const VSAInput in = fx.input();
    CHECK(partition(in, s) == 0);
    CHECK(allLabeled(s));
    CHECK(regionsConnected(fx.mesh, s));
    for (int r = 0; r < 6; ++r) CHECK_NEAR(s.regions[r].area, 4.0, 1e-12);   // one cube side: 2 triangles, 2x2
    CHECK_NEAR(totalEnergy(s), 0.0, 1e-12);
    fitProxies(in, s);
    CHECK_NEAR(totalEnergy(s), 0.0, 1e-12);
    // the seed of the next partition is inside the same region: the result must be stable
    const std::vector<int> before = s.label;
    partition(in, s);
    CHECK(before == s.label);
}

static void testDisconnectedComponents()
{
    Fixture fx(TriMesh{});
    addCube(fx.mesh, 0.0);
    addCube(fx.mesh, 10.0);
    fx = Fixture(std::move(fx.mesh));

    VSAState s;
    s.metric = Metric::L2;
    initRandomSeeds(fx.input(), s, 1, 5);
    CHECK(s.k() == 1);
    const int added = partition(fx.input(), s);
    CHECK(added == 1);
    CHECK(s.k() == 2);
    CHECK(allLabeled(s));
    CHECK(regionsConnected(fx.mesh, s));
    // each cube belongs to exactly one proxy
    std::set<int> a, b;
    for (int f = 0; f < 12; ++f) a.insert(s.label[f]);
    for (int f = 12; f < 24; ++f) b.insert(s.label[f]);
    CHECK(a.size() == 1 && b.size() == 1 && *a.begin() != *b.begin());
}

static void testSeedCount()
{
    Fixture fx(TriMesh{});
    addCube(fx.mesh, 0.0);
    fx = Fixture(std::move(fx.mesh));
    VSAState s;
    initRandomSeeds(fx.input(), s, 100, 1);
    CHECK(s.k() == 12);
    initRandomSeeds(fx.input(), s, 0, 1);
    CHECK(s.k() == 1);
    initRandomSeeds(fx.input(), s, 7, 1);
    CHECK(s.k() == 7);
    CHECK(std::set<int>(s.seedFace.begin(), s.seedFace.end()).size() == 7);   // distinct

    // k = number of faces: every face is its own region
    initRandomSeeds(fx.input(), s, 12, 1);
    partition(fx.input(), s);
    CHECK(allLabeled(s));
    CHECK(std::set<int>(s.label.begin(), s.label.end()).size() == 12);
}

static void testRandomBump()
{
    Fixture fx(makeBump(14));
    const VSAInput in = fx.input();

    for (Metric me : {Metric::L2, Metric::L21}) {
        for (uint64_t seed = 1; seed <= 5; ++seed) {
            VSAState s;
            s.metric = me;
            initRandomSeeds(in, s, 6, seed);
            CHECK(partition(in, s) == 0);   // connected mesh: no extra proxies
            CHECK(allLabeled(s));
            CHECK(regionsConnected(fx.mesh, s));

            // refitting each region cannot increase the energy
            const double e0 = totalEnergy(s);
            fitProxies(in, s);
            CHECK(totalEnergy(s) <= e0 + 1e-12);

            // later partitions keep every invariant
            for (int it = 0; it < 5; ++it) {
                partition(in, s);
                CHECK(allLabeled(s));
                CHECK(regionsConnected(fx.mesh, s));
                fitProxies(in, s);
            }

            // determinism
            VSAState a, b;
            a.metric = b.metric = me;
            initRandomSeeds(in, a, 6, seed);
            initRandomSeeds(in, b, 6, seed);
            partition(in, a);
            partition(in, b);
            CHECK(a.label == b.label);
        }
    }

    // region moments are consistent with the labels
    VSAState s;
    initRandomSeeds(in, s, 4, 9);
    partition(in, s);
    Moments total;
    for (const Moments& r : s.regions) total += r;
    Moments all;
    for (const Moments& m : fx.fm) all += m;
    CHECK_NEAR(total.area, all.area, 1e-12);
    CHECK_NEAR((total.areaNormal - all.areaNormal).norm(), 0.0, 1e-12);
}

static void testHemisphereFile(const std::string& path)
{
    TriMesh m;
    CHECK(loadMesh(path, m));
    normalizeToUnitBox(m);
    Fixture fx(std::move(m));
    const MeshDiagnostics d = diagnose(fx.mesh, fx.fd);
    CHECK(d.eulerCharacteristic == 1 && d.boundaryLoops == 1 && d.components == 1);   // a disk
    CHECK(d.degenerateFaces == 0 && d.nonManifoldVertices == 0);

    for (Metric me : {Metric::L2, Metric::L21}) {
        VSAState s;
        s.metric = me;
        initRandomSeeds(fx.input(), s, 8, 3);
        CHECK(partition(fx.input(), s) == 0);
        CHECK(allLabeled(s));
        CHECK(regionsConnected(fx.mesh, s));
        for (int it = 0; it < 10; ++it) {
            fitProxies(fx.input(), s);
            partition(fx.input(), s);
            CHECK(allLabeled(s) && regionsConnected(fx.mesh, s));
        }
    }
}

int main(int argc, char** argv)
{
    testCubeSixSeeds();
    testDisconnectedComponents();
    testSeedCount();
    testRandomBump();
    if (argc > 1) testHemisphereFile(argv[1]);
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
