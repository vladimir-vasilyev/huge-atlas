#include "core/MeshPrep.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

int main(int argc, char** argv)
{
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
    CHECK(std::abs(d.totalArea - 6.0) < 1e-12);   // единичный куб
    for (auto f : m.faces()) CHECK(std::abs(fd.normal[f.idx()].norm() - 1.0) < 1e-12);

    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
