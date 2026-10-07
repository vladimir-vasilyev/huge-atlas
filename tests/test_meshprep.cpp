#include "core/MeshPrep.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

// A cube without one face (two triangles: a hole of four edges) and without one triangle (a hole of three edges).
static void testFillSmallHoles()
{
    static const double p[8][3] = {{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
    static const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
    for (int skip : {1, 2}) {   // skip = triangles left out: 1 gives a hole of 3 edges, 2 (a whole side) a hole of 4
        TriMesh m;
        TriMesh::VertexHandle v[8];
        for (int i = 0; i < 8; ++i) v[i] = m.add_vertex(Vec3(p[i][0], p[i][1], p[i][2]));
        for (int i = 0; i < 12 - skip; ++i) m.add_face(v[f[i][0]], v[f[i][1]], v[f[i][2]]);
        // leave out the last `skip` triangles (they share a side)
        const FaceData before = computeFaceData(m);
        CHECK(diagnose(m, before).boundaryLoops == 1);
        CHECK(fillSmallHoles(m, 2) == 0);                    // too long for the limit: untouched
        CHECK(fillSmallHoles(m, 8) == 1);
        const FaceData after = computeFaceData(m);
        const MeshDiagnostics d = diagnose(m, after);
        CHECK(d.boundaryLoops == 0 && d.eulerCharacteristic == 2);
        CHECK(d.nonManifoldVertices == 0);
    }
}

int main(int argc, char** argv)
{
    testFillSmallHoles();
    const std::string path = argc > 1 ? argv[1] : "data/cube.obj";
    TriMesh m;
    std::string err;
    CHECK(loadMesh(path, m, &err));
    normalizeToUnitBox(m);
    const FaceData fd = computeFaceData(m);
    const MeshDiagnostics d = diagnose(m, fd);

    CHECK(d.vertices == 8 && d.edges == 18 && d.faces == 12);
    CHECK(d.eulerCharacteristic == 2);
    CHECK(d.components == 1);
    CHECK(d.boundaryLoops == 0);
    CHECK(d.degenerateFaces == 0 && d.nonManifoldVertices == 0 && d.isolatedVertices == 0);
    CHECK(std::abs(d.totalArea - 6.0) < 1e-12);   // unit cube
    for (auto f : m.faces()) CHECK(std::abs(fd.normal[f.idx()].norm() - 1.0) < 1e-12);

    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
