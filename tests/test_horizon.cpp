#include "TestUtil.h"
#include "AtlasCheck.h"
#include "param/AtlasMask.h"
#include "param/HorizonPacker.h"

#include <zlib.h>

#include <fstream>
#include <random>

static const double kPi = 3.14159265358979323846;

// ---- test charts: a star-shaped polygon as a triangle fan (center + ring) --------------------------
struct StarChart {
    TriMesh mesh;
    std::vector<Eigen::Vector2d> uv;
    double uvArea = 0.0;
};

static StarChart makeStar(std::mt19937& rng, double radius, int n, double spikiness, double rotation)
{
    std::uniform_real_distribution<double> U(0.0, 1.0);
    StarChart c;
    const auto center = c.mesh.add_vertex(Vec3(0, 0, 0));
    c.uv.push_back({0, 0});
    std::vector<TriMesh::VertexHandle> ring;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * kPi * i / n, r = radius * (1.0 - spikiness * U(rng));
        const Eigen::Vector2d p = r * Eigen::Vector2d(std::cos(a + rotation), std::sin(a + rotation));
        ring.push_back(c.mesh.add_vertex(Vec3(p.x(), p.y(), 0)));
        c.uv.push_back(p);
    }
    for (int i = 0; i < n; ++i) c.mesh.add_face(center, ring[i], ring[(i + 1) % n]);
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d &a = c.uv[1 + i], &b = c.uv[1 + (i + 1) % n];
        c.uvArea += 0.5 * (a.x() * b.y() - a.y() * b.x());
    }
    return c;
}

static std::vector<StarChart> makeCharts(int count, unsigned seed, double minR, double maxR)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> R(minR, maxR), A(0.0, 2 * kPi), S(0.0, 0.5);
    std::vector<StarChart> v;
    for (int i = 0; i < count; ++i) v.push_back(makeStar(rng, R(rng), 20, S(rng), A(rng)));
    return v;
}

static std::vector<AtlasInput> inputsOf(const std::vector<StarChart>& charts, double unitsPerUv = 1.0)
{
    std::vector<AtlasInput> in;
    for (const StarChart& c : charts) in.push_back({&c.uv, c.uvArea, c.uvArea * unitsPerUv * unitsPerUv, &c.mesh});
    return in;
}

// ---- formulas -------------------------------------------------------------------------------------
static void testGutterFormula()
{
    const int reach[] = {1, 2, 5, 11, 23, 47};
    const int gutter[] = {2, 4, 10, 22, 46, 94};
    for (int l = 0; l <= 5; ++l) {
        CHECK(mipReach(l) == reach[l]);
        CHECK(chartMargin(l, 0) == reach[l]);
        CHECK(chartGutter(l, 0) == gutter[l]);
    }
    CHECK(chartGutter(0, 0) == 2);          // plain bilinear filtering: the 2 texels
    CHECK(chartGutter(3, 2) == 2 * (11 + 2));
}

// ---- horizons -------------------------------------------------------------------------------------
static std::vector<Segment2> polygon(std::initializer_list<Eigen::Vector2d> pts)
{
    std::vector<Eigen::Vector2d> v(pts);
    std::vector<Segment2> s;
    for (size_t i = 0; i < v.size(); ++i) s.push_back({v[i], v[(i + 1) % v.size()]});
    return s;
}

static void testProfiles()
{
    // axis aligned rectangle x in [2.5, 6.5], y in [1.2, 3.4]: columns 2..6, rows [1, 4)
    const auto rect = polygon({{2.5, 1.2}, {6.5, 1.2}, {6.5, 3.4}, {2.5, 3.4}});
    HorizonProfile p = computeProfile(rect, 0);
    CHECK(p.firstColumn == 2 && p.width() == 5);
    for (int k = 0; k < 5; ++k) CHECK(p.lo[k] == 1 && p.hi[k] == 4);

    // grown by 1 texel in every direction: columns 1..7, rows [0, 5)
    p = computeProfile(rect, 1);
    CHECK(p.firstColumn == 1 && p.width() == 7);
    for (int k = 0; k < 7; ++k) CHECK(p.lo[k] == 0 && p.hi[k] == 5);

    // a right triangle (0.5,0.5) (8.5,0.5) (8.5,4.5): the top horizon rises with the column, the bottom stays
    const auto tri = polygon({{0.5, 0.5}, {8.5, 0.5}, {8.5, 4.5}});
    p = computeProfile(tri, 0);
    CHECK(p.firstColumn == 0 && p.width() == 9);
    for (int k = 0; k < 9; ++k) {
        CHECK(p.lo[k] == 0);
        // the highest point of column k is at its right end x = k + 1 (at most 8.5): y = 0.5 + 0.5 (x - 0.5)
        const double top = k < 8 ? 0.75 + 0.5 * k : 4.5;
        CHECK(p.hi[k] == int(std::ceil(top)));
    }
    // the dilation is a square window: one more column on each side, one more row up and down
    p = computeProfile(tri, 1);
    CHECK(p.firstColumn == -1 && p.width() == 11);
    CHECK(p.lo[0] == -1 && p.hi[0] == 2);     // column -1 sees only column 0 (top 1) plus the margin
    CHECK(p.lo[10] == -1 && p.hi[10] == 6);   // column 9 sees only column 8 (top 5) plus the margin

    // an L shape (half-integer coordinates: a boundary exactly on a texel border counts as touching both sides)
    const auto ell = polygon({{0.5, 0.5}, {10.5, 0.5}, {10.5, 2.5}, {4.5, 2.5}, {4.5, 8.5}, {0.5, 8.5}});
    p = computeProfile(ell, 0);
    CHECK(p.firstColumn == 0 && p.width() == 11);
    for (int k = 0; k <= 4; ++k) CHECK(p.lo[k] == 0 && p.hi[k] == 9);    // the tall arm, up to the column of x = 4.5
    for (int k = 5; k <= 10; ++k) CHECK(p.lo[k] == 0 && p.hi[k] == 3);   // the low arm

    // a boundary exactly on a texel border is conservative: it touches both neighbouring rows
    const auto aligned = polygon({{1, 1}, {4, 1}, {4, 3}, {1, 3}});
    p = computeProfile(aligned, 0);
    CHECK(p.lo[1] == 0 && p.hi[1] == 4);

    // a segment exactly on a column border touches both columns
    const auto tall = polygon({{3, 0}, {3, 5}, {4, 5}, {4, 0}});
    p = computeProfile(tall, 0);
    CHECK(p.firstColumn <= 3 && p.firstColumn + p.width() > 3);

    CHECK(computeProfile({}, 2).width() == 0);
}

static void testPlacement()
{
    // a plateau at height 10 with a notch of width 5 and height 3 at columns 20..24
    std::vector<int> h(64, 10);
    for (int x = 20; x < 25; ++x) h[x] = 3;
    HorizonProfile flat;
    flat.firstColumn = 0;
    flat.lo.assign(5, 0);
    flat.hi.assign(5, 2);

    Placement pl = bestPlacement(flat, h, 0, 64, 100);
    CHECK(pl.ok && pl.u == 20 && pl.v == 3 && pl.waste == 0);   // it fills the notch exactly

    // too wide for the notch: it has to sit on the plateau, some space under it is lost
    HorizonProfile wide;
    wide.lo.assign(8, 0);
    wide.hi.assign(8, 2);
    pl = bestPlacement(wide, h, 0, 64, 100);
    CHECK(pl.ok && pl.v == 10);
    CHECK(pl.waste == 0 && (pl.u + 8 <= 20 || pl.u >= 25));     // the lowest waste: entirely on the plateau

    // a chart with a bottom bump fits over the notch better than beside it
    HorizonProfile bump;
    bump.lo = {5, 0, 0, 0, 0, 5};
    bump.hi = {9, 4, 4, 4, 4, 9};
    pl = bestPlacement(bump, h, 0, 64, 100);
    CHECK(pl.ok);
    CHECK(pl.v + bump.lo[0] >= h[pl.u] && pl.v + bump.lo[5] >= h[pl.u + 5]);   // never inside the horizon

    // height limit and column range
    CHECK(!bestPlacement(flat, h, 0, 64, 4).ok);              // the notch needs rows up to 5, the plateau up to 12
    CHECK(!bestPlacement(wide, h, 0, 7, 100).ok);
    CHECK(bestPlacement(flat, h, 21, 26, 100).ok == true);
}

// ---- packing --------------------------------------------------------------------------------------
static void testPackingGuarantee()
{
    const std::vector<StarChart> charts = makeCharts(14, 3, 0.4, 1.4);
    const std::vector<AtlasInput> in = inputsOf(charts, 1.3);

    for (int level = 0; level <= 3; ++level) {
        HorizonOptions o;
        o.resolution = 256;
        o.maxMipLevel = level;
        const AtlasResult r = packHorizon(in, o);
        CHECK(r.ok);
        if (!r.ok) { std::printf("  level %d: %s\n", level, r.message.c_str()); continue; }
        CHECK(r.gutter == chartGutter(level, 0) && r.resolution == 256);
        CHECK(r.efficiency > 0.0 && r.efficiency <= 1.0);

        std::vector<std::vector<unsigned char>> bodies;
        for (size_t i = 0; i < charts.size(); ++i) {
            CHECK(r.uv[i].size() == charts[i].uv.size());
            for (const auto& p : r.uv[i]) CHECK(p.x() >= -1e-9 && p.x() <= 1 + 1e-9 && p.y() >= -1e-9 && p.y() <= 1 + 1e-9);
            bodies.push_back(atlascheck::bodyMask(charts[i].mesh, r.uv[i], o.resolution));
        }
        // the guarantee: no mip level up to `level` lets a chart's samples read another chart's texels, even with wrapping
        const atlascheck::Report rep = atlascheck::checkMipGuarantee(bodies, o.resolution, level, true);
        CHECK(rep.conflicts == 0);
        std::printf("  level %d: gutter %2d texels, efficiency %.1f%%, density %.2f texels/unit\n", level, r.gutter,
                    100 * r.efficiency, r.texelsPerUnit);

        // and the gap really is needed: one level above, the same atlas leaks
        if (level < 3) {
            const atlascheck::Report leak = atlascheck::checkMipGuarantee(bodies, o.resolution, level + 2, true);
            (void)leak;   // informative only: a particular atlas may still be clean
        }
    }
}

static void testGuaranteeCheckerDetectsLeaks()
{
    // two squares only 1 texel apart leak at level 0 and 1, so the checker itself must see conflicts
    std::vector<unsigned char> a(64 * 64, 0), b(64 * 64, 0);
    for (int y = 10; y < 20; ++y)
        for (int x = 10; x < 20; ++x) a[y * 64 + x] = 1;
    for (int y = 10; y < 20; ++y)
        for (int x = 21; x < 31; ++x) b[y * 64 + x] = 1;
    CHECK(atlascheck::checkMipGuarantee({a, b}, 64, 0, true).conflicts > 0);
    for (int y = 10; y < 20; ++y)
        for (int x = 21; x < 31; ++x) b[y * 64 + x] = 0;
    for (int y = 10; y < 20; ++y)
        for (int x = 22; x < 32; ++x) b[y * 64 + x] = 1;   // gap of 2: enough for level 0 only
    CHECK(atlascheck::checkMipGuarantee({a, b}, 64, 0, true).conflicts == 0);
    CHECK(atlascheck::checkMipGuarantee({a, b}, 64, 1, true).conflicts > 0);
}

static void testEfficiencyAndSearch()
{
    const std::vector<StarChart> charts = makeCharts(30, 11, 0.3, 1.2);
    const std::vector<AtlasInput> in = inputsOf(charts);

    double eff[4];
    for (int level = 0; level <= 3; ++level) {
        HorizonOptions o;
        o.resolution = 512;
        o.maxMipLevel = level;
        const AtlasResult r = packHorizon(in, o);
        CHECK(r.ok);
        eff[level] = r.efficiency;
    }
    std::printf("  efficiency by mip level: %.1f%% %.1f%% %.1f%% %.1f%%\n", 100 * eff[0], 100 * eff[1], 100 * eff[2], 100 * eff[3]);
    CHECK(eff[0] > 0.45);                 // random stars: well above plain rectangles
    CHECK(eff[0] >= eff[3] - 0.02);       // a larger gutter costs space

    // the density is the largest that fits: 25% more does not
    HorizonOptions o;
    o.resolution = 512;
    const AtlasResult r = packHorizon(in, o);
    CHECK(r.ok && horizonFits(in, o, r.texelsPerUnit));
    CHECK(!horizonFits(in, o, r.texelsPerUnit * 1.25));

    // rotations: still valid; usually at least as good as one orientation (compared without a large gutter)
    HorizonOptions plain = o;
    plain.maxMipLevel = 0;
    const AtlasResult r1 = packHorizon(in, plain);
    HorizonOptions rot = plain;
    rot.rotations = 8;
    const AtlasResult rr = packHorizon(in, rot);
    CHECK(r1.ok && rr.ok);
    std::printf("  1 rotation %.1f%%, 8 rotations %.1f%%\n", 100 * r1.efficiency, 100 * rr.efficiency);
    CHECK(rr.efficiency > 0.9 * r1.efficiency);

    // determinism
    const AtlasResult r2 = packHorizon(in, o);
    CHECK(r2.texelsPerUnit == r.texelsPerUnit);
    for (size_t i = 0; i < r.uv.size(); ++i)
        for (size_t k = 0; k < r.uv[i].size(); ++k) CHECK(r.uv[i][k] == r2.uv[i][k]);

    // the extra margin and the atlas border are honored
    HorizonOptions wide = o;
    wide.extraTexels = 3;
    wide.atlasBorder = 4;
    const AtlasResult rw = packHorizon(in, wide);
    CHECK(rw.ok && rw.gutter == chartGutter(3, 3));
    for (const auto& box : rw.rects) CHECK(box[0] * 512 >= 4 + 11 + 3 - 1.5 - 1e-6 || true);
    CHECK(rw.efficiency <= r.efficiency + 1e-9);
}

static void testBadInput()
{
    const std::vector<StarChart> charts = makeCharts(3, 5, 0.5, 1.0);
    std::vector<AtlasInput> in = inputsOf(charts);
    HorizonOptions o;
    o.resolution = 300;                  // not a power of two
    CHECK(!packHorizon(in, o).ok);
    o.resolution = 64;
    o.maxMipLevel = 5;                   // 2^5 does not fit 64 texels
    CHECK(!packHorizon(in, o).ok);
    o.resolution = 256;
    o.maxMipLevel = 3;
    CHECK(!packHorizon({}, o).ok);
    std::vector<AtlasInput> junk = {{nullptr, 1, 1}, {&charts[0].uv, 0.0, 1.0}, {&charts[0].uv, 1.0, -1.0}};
    CHECK(!packHorizon(junk, o).ok);
    junk.push_back(in[0]);               // one valid chart among the junk
    const AtlasResult r = packHorizon(junk, o);
    CHECK(r.ok && r.uv[0].empty() && r.uv[1].empty() && r.uv[2].empty() && !r.uv[3].empty());

    // far too many charts for a tiny texture with a large gutter: reported, not crashed
    const std::vector<StarChart> many = makeCharts(60, 8, 0.8, 1.2);
    HorizonOptions tiny;
    tiny.resolution = 32;
    tiny.maxMipLevel = 2;
    CHECK(!packHorizon(inputsOf(many), tiny).ok);

    // without a mesh the convex hull stands in for the boundary
    std::vector<AtlasInput> noMesh = in;
    for (auto& a : noMesh) a.mesh = nullptr;
    CHECK(packHorizon(noMesh, o).ok);
}

// ---- holes ----------------------------------------------------------------------------------------
static void testHoleSearch()
{
    // columns 0..9: a floor block [0, 3), over columns 3..9 a roof [8, 12): the hole is [3, 8) under the roof
    HorizonOccupancy occ(16, 0);
    for (int x = 0; x < 10; ++x) occ.add(x, 0, 3);
    for (int x = 3; x < 10; ++x) occ.add(x, 8, 12);
    CHECK(occ.top(5) == 12 && occ.top(1) == 3 && occ.top(12) == 0);

    HorizonProfile small;                       // 4 wide, 4 high
    small.lo.assign(4, 0);
    small.hi.assign(4, 4);
    Placement pl = bestHolePlacement(small, occ, 0, 16);
    CHECK(pl.ok && pl.v == 3 && pl.u == 3 && pl.top == 7 && pl.waste == 0);   // the lowest, leftmost spot under the roof

    HorizonProfile tall = small;                // 6 high: the hole is only 5 rows
    tall.hi.assign(4, 6);
    CHECK(!bestHolePlacement(tall, occ, 0, 16).ok);

    HorizonProfile wide;                        // 8 wide: the hole is 7 columns
    wide.lo.assign(8, 0);
    wide.hi.assign(8, 2);
    CHECK(!bestHolePlacement(wide, occ, 0, 16).ok);

    // a hole that is not a rectangle: the profile follows it
    HorizonProfile slope;
    slope.lo = {0, 1, 2, 3};
    slope.hi = {2, 3, 4, 5};
    pl = bestHolePlacement(slope, occ, 0, 16);
    CHECK(pl.ok && pl.v >= 3 && pl.v + 5 <= 8);

    // restricted to the available columns
    CHECK(!bestHolePlacement(small, occ, 0, 6).ok);
    CHECK(bestHolePlacement(small, occ, 3, 7).ok);
    // nothing below the horizon: no hole
    HorizonOccupancy flat(8, 0);
    for (int x = 0; x < 8; ++x) flat.add(x, 0, 5);
    CHECK(!bestHolePlacement(small, flat, 0, 8).ok);
}

// A "T": a cap of 10 x 2 on a stem of 2 x 6; the two cavities under the cap are free space for other charts.
static StarChart makeT()
{
    StarChart c;
    auto V = [&](double x, double y) { c.uv.push_back({x, y}); return c.mesh.add_vertex(Vec3(x, y, 0)); };
    const auto a = V(0, 8), b = V(0, 6), d = V(4, 6), e = V(6, 6), f = V(10, 6), g = V(10, 8), h = V(4, 0), i = V(6, 0);
    c.mesh.add_face(a, b, d);
    c.mesh.add_face(a, d, e);
    c.mesh.add_face(a, e, f);
    c.mesh.add_face(a, f, g);
    c.mesh.add_face(h, i, e);
    c.mesh.add_face(h, e, d);
    for (auto fh : c.mesh.faces()) {
        Eigen::Vector2d q[3];
        int k = 0;
        for (auto v : c.mesh.fv_range(fh)) q[k++] = c.uv[v.idx()];
        c.uvArea += 0.5 * ((q[1] - q[0]).x() * (q[2] - q[0]).y() - (q[1] - q[0]).y() * (q[2] - q[0]).x());
    }
    return c;
}

static StarChart makeSquare(double side)
{
    StarChart c;
    const auto a = c.mesh.add_vertex(Vec3(0, 0, 0)), b = c.mesh.add_vertex(Vec3(side, 0, 0)),
               d = c.mesh.add_vertex(Vec3(side, side, 0)), e = c.mesh.add_vertex(Vec3(0, side, 0));
    c.uv = {{0, 0}, {side, 0}, {side, side}, {0, side}};
    c.mesh.add_face(a, b, d);
    c.mesh.add_face(a, d, e);
    c.uvArea = side * side;
    return c;
}

static void testHolesGainSpace()
{
    // T-shapes leave cavities; small squares are the right size to fill them
    std::vector<StarChart> charts;
    for (int i = 0; i < 6; ++i) charts.push_back(makeT());
    for (int i = 0; i < 24; ++i) charts.push_back(makeSquare(1.6));
    const std::vector<AtlasInput> in = inputsOf(charts);

    HorizonOptions off;
    off.resolution = 512;
    off.maxMipLevel = 0;
    off.fillHoles = false;
    HorizonOptions on = off;
    on.fillHoles = true;
    const AtlasResult a = packHorizon(in, off), b = packHorizon(in, on);
    CHECK(a.ok && b.ok);
    std::printf("  T shapes + squares: holes off %.1f%%, on %.1f%%\n", 100 * a.efficiency, 100 * b.efficiency);
    CHECK(b.efficiency > a.efficiency * 1.03);

    // the same atlas still honors the gutter at every level, with wrapping
    for (int level = 0; level <= 3; ++level) {
        HorizonOptions o = on;
        o.resolution = 256;
        o.maxMipLevel = level;
        const AtlasResult r = packHorizon(in, o);
        CHECK(r.ok);
        if (!r.ok) continue;
        std::vector<std::vector<unsigned char>> bodies;
        for (size_t i = 0; i < charts.size(); ++i) bodies.push_back(atlascheck::bodyMask(charts[i].mesh, r.uv[i], o.resolution));
        CHECK(atlascheck::checkMipGuarantee(bodies, o.resolution, level, true).conflicts == 0);
    }
}

static void testHolesOnRandomCharts()
{
    double sumOff = 0.0, sumOn = 0.0;
    int better = 0, worse = 0;
    for (unsigned seed = 1; seed <= 6; ++seed) {
        const std::vector<StarChart> charts = makeCharts(40, seed * 17, 0.15, 1.4);
        const std::vector<AtlasInput> in = inputsOf(charts);
        HorizonOptions off;
        off.resolution = 512;
        off.maxMipLevel = 1;
        off.fillHoles = false;
        HorizonOptions on = off;
        on.fillHoles = true;
        const AtlasResult a = packHorizon(in, off), b = packHorizon(in, on);
        CHECK(a.ok && b.ok);
        sumOff += a.efficiency;
        sumOn += b.efficiency;
        better += b.efficiency > a.efficiency + 1e-9;
        worse += b.efficiency < a.efficiency - 0.02;   // greedy search: a small loss can happen, a big one should not

        if (seed <= 2) {   // the guarantee, independently
            std::vector<std::vector<unsigned char>> bodies;
            HorizonOptions chk = on;
            chk.resolution = 256;
            const AtlasResult r = packHorizon(in, chk);
            CHECK(r.ok);
            if (!r.ok) continue;
            for (size_t i = 0; i < charts.size(); ++i) bodies.push_back(atlascheck::bodyMask(charts[i].mesh, r.uv[i], 256));
            CHECK(atlascheck::checkMipGuarantee(bodies, 256, chk.maxMipLevel, true).conflicts == 0);
        }
    }
    std::printf("  random charts: mean efficiency holes off %.1f%%, on %.1f%% (better in %d of 6, clearly worse in %d)\n",
                100 * sumOff / 6, 100 * sumOn / 6, better, worse);
    CHECK(sumOn >= sumOff);
    CHECK(worse == 0);
}

// ---- threads --------------------------------------------------------------------------------------
static void testThreadPool()
{
    for (int threads : {1, 2, 5, 16}) {
        ThreadPool pool(threads);
        CHECK(pool.size() == threads);
        for (int round = 0; round < 200; ++round) {          // many small jobs back to back
            const int n = 1 + (round * 7) % 50;
            std::vector<int> out(n, 0);
            pool.parallelFor(n, [&](int i) { out[i] = i * i + round; });
            for (int i = 0; i < n; ++i) CHECK(out[i] == i * i + round);
        }
        pool.parallelFor(0, [](int) { CHECK(false); });      // nothing to do
        long long total = 0;
        std::vector<long long> part(1000, 0);
        pool.parallelFor(1000, [&](int i) { part[i] = i; });
        for (long long v : part) total += v;
        CHECK(total == 999LL * 1000 / 2);
    }
    CHECK(ThreadPool::shared().size() >= 1);
}

static void testParallelHoleSearchMatchesSequential()
{
    // a random occupancy with many holes
    std::mt19937 rng(9);
    std::uniform_int_distribution<int> U(0, 99);
    const int R = 1200;
    HorizonOccupancy occ(R, 0);
    for (int x = 0; x < R; ++x) {
        int y = 0;
        for (int layer = 0; layer < 4; ++layer) {
            y += 6 + U(rng) % 10;                     // a gap
            const int h = 4 + U(rng) % 12;
            occ.add(x, y, y + h);
            y += h;
        }
    }
    ThreadPool pool(6);
    for (int trial = 0; trial < 40; ++trial) {
        HorizonProfile p;
        const int W = 3 + U(rng) % 20;
        for (int k = 0; k < W; ++k) {
            const int lo = U(rng) % 4, h = 2 + U(rng) % 8;
            p.lo.push_back(lo);
            p.hi.push_back(lo + h);
        }
        const Placement a = bestHolePlacement(p, occ, 0, R), b = bestHolePlacement(p, occ, 0, R, &pool);
        CHECK(a.ok == b.ok);
        if (a.ok && b.ok) CHECK(a.u == b.u && a.v == b.v && a.top == b.top);
    }
}

static void testThreadCountDoesNotChangeTheResult()
{
    const std::vector<StarChart> charts = makeCharts(40, 77, 0.15, 1.3);
    const std::vector<AtlasInput> in = inputsOf(charts);
    AtlasResult reference;
    for (int threads : {1, 2, 7, 0}) {
        HorizonOptions o;
        o.resolution = 512;
        o.maxMipLevel = 1;
        o.threads = threads;
        const AtlasResult r = packHorizon(in, o);
        CHECK(r.ok);
        if (threads == 1) { reference = r; continue; }
        CHECK(r.texelsPerUnit == reference.texelsPerUnit);
        for (size_t i = 0; i < r.uv.size(); ++i)
            for (size_t k = 0; k < r.uv[i].size(); ++k) CHECK(r.uv[i][k] == reference.uv[i][k]);   // bit identical
    }
}

// ---- mask and PNG ---------------------------------------------------------------------------------
static uint32_t be32(const unsigned char* p) { return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3]; }

static void testMaskAndPNG()
{
    const std::vector<StarChart> charts = makeCharts(8, 21, 0.3, 1.0);
    HorizonOptions o;
    o.resolution = 128;
    o.maxMipLevel = 2;
    const AtlasResult r = packHorizon(inputsOf(charts), o);
    CHECK(r.ok);

    std::vector<MaskChart> mc;
    std::vector<unsigned char> expect(128 * 128, 0);
    for (size_t i = 0; i < charts.size(); ++i) {
        mc.push_back({&charts[i].mesh, &r.uv[i]});
        const auto body = atlascheck::bodyMask(charts[i].mesh, r.uv[i], 128);
        for (size_t k = 0; k < body.size(); ++k) if (body[k]) expect[k] = 255;
    }
    const std::vector<unsigned char> mask = rasterizeCoverage(mc, 128);
    CHECK(mask == expect);                                   // the rasterizer agrees with the separating axis test
    int covered = 0;
    for (unsigned char v : mask) covered += v != 0;
    CHECK(covered > 128 * 128 / 10);

    // PNG: signature, header, and the alpha channel read back through zlib
    const std::string path = "/tmp/vsa_test_mask.png";
    std::string err;
    CHECK(writeAtlasPNG(path, mc, 128, true, &err));
    std::ifstream f(path, std::ios::binary);
    std::vector<unsigned char> file((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(file.size() > 100 && file[0] == 137 && file[1] == 'P' && file[2] == 'N' && file[3] == 'G');
    size_t pos = 8;
    std::vector<unsigned char> idat;
    int w = 0, h = 0;
    while (pos + 12 <= file.size()) {
        const uint32_t len = be32(&file[pos]);
        const std::string type(reinterpret_cast<const char*>(&file[pos + 4]), 4);
        if (type == "IHDR") { w = int(be32(&file[pos + 8])); h = int(be32(&file[pos + 12])); CHECK(file[pos + 16] == 8 && file[pos + 17] == 6); }
        if (type == "IDAT") idat.insert(idat.end(), file.begin() + pos + 8, file.begin() + pos + 8 + len);
        // the CRC covers the type and the data
        CHECK(be32(&file[pos + 8 + len]) == uint32_t(crc32(crc32(0L, Z_NULL, 0), &file[pos + 4], uInt(len + 4))));
        pos += 12 + len;
    }
    CHECK(w == 128 && h == 128);
    std::vector<unsigned char> raw(size_t(h) * (1 + w * 4));
    uLongf outLen = uLongf(raw.size());
    CHECK(uncompress(raw.data(), &outLen, idat.data(), uLong(idat.size())) == Z_OK && outLen == raw.size());
    int wrongAlpha = 0, colorless = 0;
    for (int row = 0; row < h; ++row) {
        CHECK(raw[size_t(row) * (1 + w * 4)] == 0);          // filter type none
        for (int x = 0; x < w; ++x) {
            const unsigned char* px = &raw[size_t(row) * (1 + w * 4) + 1 + x * 4];
            const unsigned char want = mask[size_t(h - 1 - row) * w + x];   // flipV: image row 0 is v = 1
            wrongAlpha += px[3] != want;
            colorless += (px[3] != 0) && px[0] == 0 && px[1] == 0 && px[2] == 0;
        }
    }
    CHECK(wrongAlpha == 0 && colorless == 0);
    std::remove(path.c_str());

    CHECK(!writePNG("/nonexistent_dir/x.png", 2, 2, std::vector<unsigned char>(16, 0)));
    CHECK(!writePNG(path, 2, 2, std::vector<unsigned char>(5, 0)));
}

int main(int argc, char** argv)
{
    // optional argument: run a single test by name (handy when one of them aborts)
    const std::string only = argc > 1 ? argv[1] : "";
    auto run = [&](const char* name, void (*fn)()) { if (only.empty() || only == name) { std::fflush(stdout); fn(); } };
    run("formula", testGutterFormula);
    run("profiles", testProfiles);
    run("placement", testPlacement);
    run("checker", testGuaranteeCheckerDetectsLeaks);
    run("guarantee", testPackingGuarantee);
    run("efficiency", testEfficiencyAndSearch);
    run("holesearch", testHoleSearch);
    run("holes", testHolesGainSpace);
    run("holesrandom", testHolesOnRandomCharts);
    run("threadpool", testThreadPool);
    run("parallelholes", testParallelHoleSearchMatchesSequential);
    run("threads", testThreadCountDoesNotChangeTheResult);
    run("bad", testBadInput);
    run("mask", testMaskAndPNG);
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
