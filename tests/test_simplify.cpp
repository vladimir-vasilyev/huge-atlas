// Mesh simplification: validity, topology, determinism of the block mode.
#include "core/MeshPrep.h"
#include "param/MeshSimplify.h"
#include "TestUtil.h"

#include <cstdio>

static TriMesh toTriMesh(const FlatMesh& f)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (size_t i = 0; i < f.vertexCount(); ++i)
        v.push_back(m.add_vertex(Vec3(f.pos[3 * i], f.pos[3 * i + 1], f.pos[3 * i + 2])));
    for (size_t i = 0; i < f.faceCount(); ++i) m.add_face(v[f.idx[3 * i]], v[f.idx[3 * i + 1]], v[f.idx[3 * i + 2]]);
    return m;
}

static void checkResult(const FlatMesh& in, const SimplifyResult& r, size_t target, const MeshDiagnostics& d0, const char* name)
{
    const FlatMesh& m = r.mesh;
    std::printf("%s: %zu -> %zu faces (target %zu), %zu vertices, error %.5f, blocks %zu\n", name, in.faceCount(),
                m.faceCount(), target, m.vertexCount(), r.error, r.blocks);
    CHECK(m.faceCount() <= target * 12 / 10 + 10);
    CHECK(m.faceCount() > target / 2);
    CHECK(r.vertexOrigin.size() == m.vertexCount());
    bool valid = true, positions = true, degenerate = false;
    for (size_t i = 0; i < m.idx.size(); ++i) valid = valid && m.idx[i] < m.vertexCount();
    for (size_t v = 0; v < m.vertexCount(); ++v)
        for (int c = 0; c < 3; ++c) positions = positions && m.pos[3 * v + c] == in.pos[3 * size_t(r.vertexOrigin[v]) + c];
    for (size_t f = 0; f < m.faceCount(); ++f)
        degenerate = degenerate || m.idx[3 * f] == m.idx[3 * f + 1] || m.idx[3 * f + 1] == m.idx[3 * f + 2] ||
                     m.idx[3 * f] == m.idx[3 * f + 2];
    CHECK(valid);
    CHECK(positions);
    CHECK(!degenerate);
    if (!valid) return;

    // the topology of the surface survives: same number of pieces and border loops, same genus
    TriMesh t = toTriMesh(m);
    const FaceData fd = computeFaceData(t);
    const MeshDiagnostics d = diagnose(t, fd);
    std::printf("  topology: euler %d (was %d), loops %d (was %d), components %d (was %d), non-manifold vertices %d\n",
                d.eulerCharacteristic, d0.eulerCharacteristic, d.boundaryLoops, d0.boundaryLoops, d.components,
                d0.components, d.nonManifoldVertices);
    CHECK(d.components == d0.components);
    CHECK(d.boundaryLoops == d0.boundaryLoops);
    CHECK(d.eulerCharacteristic == d0.eulerCharacteristic);
    CHECK(d.nonManifoldVertices == 0);
}

int main(int argc, char** argv)
{
    TriMesh m;
    CHECK(loadMesh(argc > 1 ? argv[1] : "data/bunny.ply", m));
    normalizeToUnitBox(m);
    const FaceData fd0 = computeFaceData(m);
    MeshDiagnostics d0 = diagnose(m, fd0);
    for (auto v : m.vertices()) d0.eulerCharacteristic -= m.is_isolated(v);   // unused vertices do not count
    const FlatMesh flat = flatFromTriMesh(m);
    CHECK(flat.faceCount() == m.n_faces());

    SimplifyOptions o;
    o.targetFaces = 3000;
    o.threads = 1;
    const SimplifyResult single = simplifyMesh(flat, o);
    CHECK(single.blocks == 0);
    checkResult(flat, single, o.targetFaces, d0, "single pass");

    o.blockFaces = 5000;   // forces the block mode on the 69k-face bunny
    o.intermediateFactor = 4;
    const SimplifyResult b1 = simplifyMesh(flat, o);
    o.threads = 8;
    const SimplifyResult b8 = simplifyMesh(flat, o);
    CHECK(b1.blocks > 1);
    checkResult(flat, b1, o.targetFaces, d0, "block mode");
    CHECK(b1.mesh.idx == b8.mesh.idx);
    CHECK(b1.vertexOrigin == b8.vertexOrigin);

    // locked vertices stay (and keep their identity)
    std::vector<uint8_t> lock(flat.vertexCount(), 0);
    for (size_t v = 0; v < lock.size(); v += 50) lock[v] = 1;
    SimplifyOptions ol;
    ol.targetFaces = 3000;
    ol.lockVertex = &lock;
    const SimplifyResult locked = simplifyMesh(flat, ol);
    size_t lockedKept = 0, lockedUsed = 0;
    std::vector<uint8_t> kept(flat.vertexCount(), 0);
    for (uint32_t v : locked.vertexOrigin) kept[v] = 1;
    for (size_t v = 0; v < lock.size(); ++v) if (lock[v]) { ++lockedUsed; lockedKept += kept[v]; }
    std::printf("locked vertices still in the mesh: %zu of %zu\n", lockedKept, lockedUsed);
    CHECK(lockedKept * 10 >= lockedUsed * 9);   // vertices that were isolated from any face do not count

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("simplify: OK\n");
    return 0;
}
