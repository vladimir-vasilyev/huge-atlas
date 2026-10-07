#include "TestUtil.h"
#include "param/PatchExtractor.h"

#include <algorithm>

// n x n quads in the plane z = 0, each split in two triangles; faces of quad (i, j) are 2*(j*n+i), +1.
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

static TriMesh makeTorus(int n, int k)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < k; ++j) {
            const double th = 2 * M_PI * i / n, ph = 2 * M_PI * j / k;
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

// Every patch face must be exactly its source face; faces are covered once.
static void checkGeometry(const TriMesh& src, const ExtractionResult& r)
{
    std::vector<int> covered(src.n_faces(), 0);
    for (size_t pi = 0; pi < r.patches.size(); ++pi) {
        const Patch& p = r.patches[pi];
        CHECK(p.faceToGlobal.size() == p.mesh.n_faces());
        CHECK(p.vertexToGlobal.size() == p.mesh.n_vertices());
        for (auto lf : p.mesh.faces()) {
            const int gf = p.faceToGlobal[lf.idx()];
            ++covered[gf];
            CHECK(r.faceToPatch[gf] == int(pi));
            auto a = p.mesh.cfv_begin(lf);
            auto b = src.cfv_begin(TriMesh::FaceHandle(gf));
            for (int k = 0; k < 3; ++k, ++a, ++b) {
                CHECK((p.mesh.point(*a) - src.point(*b)).norm() == 0.0);
                CHECK(p.vertexToGlobal[a->idx()] == b->idx());
            }
        }
    }
    for (int c : covered) CHECK(c == 1);
}

static void testCubeSides()
{
    TriMesh m;
    addCube(m, 0.0);
    std::vector<int> label(12);
    for (int f = 0; f < 12; ++f) label[f] = f / 2;

    PatchOptions keep;
    keep.minFaces = 1;   // a cube side is only two triangles
    const ExtractionResult r = extractPatches(m, label, keep);
    CHECK(r.patches.size() == 6 && r.disks == 6 && r.mergedRegions == 0);
    int vertices = 0;
    for (const Patch& p : r.patches) {
        CHECK(p.isDisk() && p.cuts == 0);
        CHECK(p.mesh.n_faces() == 2 && p.mesh.n_vertices() == 4 && p.topology.euler == 1);
        vertices += int(p.mesh.n_vertices());
    }
    CHECK(vertices == 24);   // seams are duplicated: 8 shared corners become 24
    checkGeometry(m, r);
}

static void testAnnulusIsCut()
{
    TriMesh m = makeGrid(6);
    std::vector<int> label(m.n_faces(), 0);
    for (int j = 2; j <= 3; ++j)
        for (int i = 2; i <= 3; ++i) label[2 * (j * 6 + i)] = label[2 * (j * 6 + i) + 1] = 1;

    PatchOptions keep;
    keep.cutMultiBoundary = false;
    const ExtractionResult raw = extractPatches(m, label, keep);
    CHECK(raw.patches.size() == 2);
    CHECK(raw.patches[0].topology.kind == PatchKind::MultiBoundary);
    CHECK(raw.patches[0].topology.boundaryLoops == 2 && raw.patches[0].topology.euler == 0);
    CHECK(raw.patches[1].isDisk());

    const ExtractionResult r = extractPatches(m, label);
    CHECK(r.disks == 2);
    const Patch& ring = r.patches[0];
    CHECK(ring.isDisk() && ring.cuts == 1 && ring.kindBeforeCuts == PatchKind::MultiBoundary);
    CHECK(ring.topology.euler == 1 && ring.topology.boundaryLoops == 1);
    CHECK(ring.mesh.n_faces() == raw.patches[0].mesh.n_faces());
    CHECK(ring.mesh.n_vertices() > raw.patches[0].mesh.n_vertices());   // vertices along the cut are duplicated
    checkGeometry(m, r);
}

static void testButterfly()
{
    // 4x4 quads. Quads (1,2) and (2,1) touch the center vertex (2,2) with one corner each; the region
    // around them touches itself at that vertex through quads (1,1) and (2,2) (a butterfly vertex).
    TriMesh m = makeGrid(4);
    std::vector<int> label(m.n_faces(), 0);
    auto setQuad = [&](int i, int j, int l) { label[2 * (j * 4 + i)] = label[2 * (j * 4 + i) + 1] = l; };
    setQuad(1, 2, 1);
    setQuad(2, 1, 2);

    PatchOptions opt;
    opt.minFaces = 1;
    opt.cutMultiBoundary = false;
    const ExtractionResult raw = extractPatches(m, label, opt);
    CHECK(raw.patches.size() == 3);
    const Patch& ring = raw.patches[0];
    CHECK(ring.topology.vertexManifold);                       // the pinched vertex was split
    CHECK(ring.mesh.n_vertices() == 26);                       // 25 grid vertices, the center is split in two
    // both islands stay glued through the two copies of the center vertex: one inner loop, so an annulus
    CHECK(ring.topology.boundaryLoops == 2 && ring.topology.euler == 0 && ring.topology.kind == PatchKind::MultiBoundary);

    opt.cutMultiBoundary = true;
    const ExtractionResult r = extractPatches(m, label, opt);
    CHECK(r.disks == 3);
    CHECK(r.patches[0].cuts == 1 && r.patches[0].isDisk());
    checkGeometry(m, r);
}

static void testSmallRegions()
{
    TriMesh m = makeGrid(8);
    std::vector<int> label(m.n_faces());
    for (auto f : m.faces()) {
        Vec3 c(0, 0, 0);
        for (auto v : m.fv_range(f)) c += m.point(v);
        label[f.idx()] = (c[0] / 3.0 < 4.0) ? 0 : 1;
    }
    const int lone = 2 * (4 * 8 + 1);   // a single triangle deep inside region 0
    label[lone] = 2;

    const ExtractionResult merged = extractPatches(m, label);   // minFaces = 3
    CHECK(merged.mergedRegions == 1 && merged.patches.size() == 2);
    CHECK(merged.faceToPatch[lone] == merged.faceToPatch[lone + 1]);
    checkGeometry(m, merged);
    for (const Patch& p : merged.patches) CHECK(p.isDisk());

    PatchOptions keep;
    keep.minFaces = 1;
    const ExtractionResult kept = extractPatches(m, label, keep);
    CHECK(kept.mergedRegions == 0 && kept.patches.size() == 3);

    // the labels come out compact even when the merged region had the smallest index
    std::vector<int> l2 = label;
    for (int& l : l2) l = (l == 2) ? 0 : l + 1;
    const ExtractionResult c2 = extractPatches(m, l2);
    CHECK(c2.patches.size() == 2 && c2.mergedRegions == 1);
    for (int pf : c2.faceToPatch) CHECK(pf == 0 || pf == 1);

    // an isolated tiny component cannot be merged and stays
    TriMesh two;
    addCube(two, 0.0);
    const auto a = two.add_vertex(Vec3(10, 0, 0)), b = two.add_vertex(Vec3(11, 0, 0)), c = two.add_vertex(Vec3(10, 1, 0));
    two.add_face(a, b, c);
    std::vector<int> l3(13, 0);
    l3[12] = 1;
    const ExtractionResult iso = extractPatches(two, l3);
    CHECK(iso.patches.size() == 2 && iso.mergedRegions == 0);
    CHECK(iso.patches[0].topology.kind == PatchKind::Closed && iso.patches[0].topology.genus == 0);
    CHECK(iso.patches[1].isDisk());
    CHECK(iso.disks == 1);
    checkGeometry(two, iso);
}

static void testTorus()
{
    TriMesh m = makeTorus(12, 8);
    std::vector<int> label(m.n_faces(), 0);
    ExtractionResult r = extractPatches(m, label);
    CHECK(r.patches.size() == 1);
    CHECK(r.patches[0].topology.kind == PatchKind::Closed && r.patches[0].topology.genus == 1);
    CHECK(!r.patches[0].isDisk() && !r.patches[0].note.empty());

    // punch a hole: boundary plus genus 1 -> handles, not cut
    label[0] = label[1] = label[2] = label[3] = 1;
    PatchOptions keep;
    keep.minFaces = 1;
    keep.cutHandles = false;
    r = extractPatches(m, label, keep);
    CHECK(r.patches.size() == 2);
    CHECK(r.patches[0].topology.kind == PatchKind::Handles && r.patches[0].topology.genus == 1);
    CHECK(r.patches[0].cuts == 0 && !r.patches[0].isDisk());
    CHECK(r.patches[1].isDisk());
    checkGeometry(m, r);

    // by default the handle is opened along a cut graph: a disk with the same faces
    keep.cutHandles = true;
    r = extractPatches(m, label, keep);
    CHECK(r.patches.size() == 2);
    CHECK(r.patches[0].isDisk());
    CHECK(r.patches[0].cuts > 0);
    CHECK(r.patches[0].kindBeforeCuts == PatchKind::Handles);
    CHECK(r.patches[0].faceToGlobal.size() == m.n_faces() - 4);
    checkGeometry(m, r);
}

static void testVsaPartitions(const std::string& path)
{
    TriMesh m;
    CHECK(loadMesh(path, m));
    normalizeToUnitBox(m);
    const Fixture fx(std::move(m));
    for (Metric me : {Metric::L21, Metric::L2})
        for (int k : {2, 5, 12}) {
            VSAState s;
            s.metric = me;
            initRandomSeeds(fx.input(), s, k, 7);
            runLloyd(fx.input(), s);
            const ExtractionResult r = extractPatches(fx.mesh, s.label);
            checkGeometry(fx.mesh, r);
            CHECK(!r.patches.empty());
            for (const Patch& p : r.patches) {
                CHECK(p.topology.vertexManifold);
                CHECK(p.topology.components == 1);
                // a multi-boundary patch is always cut into a disk; what remains is genuinely not a disk
                CHECK(p.topology.kind != PatchKind::MultiBoundary);
            }
        }
}

int main(int argc, char** argv)
{
    testCubeSides();
    testAnnulusIsCut();
    testButterfly();
    testSmallRegions();
    testTorus();
    if (argc > 1) testVsaPartitions(argv[1]);
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
