// ABF++ repair of a lifted map: vertices pushed out of their star flip triangles; the kernel placement clears them.
#include "param/Injectivity.h"
#include "param/LiftParam.h"
#include "TestUtil.h"

#include <cstdio>

static TriMesh makeGrid(int n, std::vector<Eigen::Vector2d>& uv)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            v.push_back(m.add_vertex(Vec3(i, j, 0.1 * std::sin(0.7 * i) * std::cos(0.5 * j))));
            uv.emplace_back(i, j);
        }
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

int main()
{
    const int n = 12;
    std::vector<Eigen::Vector2d> uv;
    const TriMesh m = makeGrid(n, uv);
    CHECK(checkUV(m, uv).ok());
    auto id = [n](int i, int j) { return j * (n + 1) + i; };

    // push three interior vertices far outside of their stars (one next to another)
    uv[id(4, 4)] += Eigen::Vector2d(2.6, -1.9);
    uv[id(8, 3)] += Eigen::Vector2d(-1.7, 2.2);
    uv[id(8, 4)] += Eigen::Vector2d(0.4, 3.0);
    UVReport rep = checkUV(m, uv);
    CHECK(!rep.flippedFaces.empty());

    const int moved = repairFlipsByKernel(m, uv, rep.flippedFaces);
    rep = checkUV(m, uv);
    std::printf("moved %d, flipped %zu, crossings %zu\n", moved, rep.flippedFaces.size(), rep.crossings.size());
    CHECK(moved >= 3);
    CHECK(rep.flippedFaces.empty());
    CHECK(rep.crossings.empty());

    // nothing to repair: nothing moves
    CHECK(repairFlipsByKernel(m, uv, {}) == 0);

    // a flipped face whose vertices all lie on the border cannot be repaired (kernel is only used for interior vertices)
    std::vector<Eigen::Vector2d> uv2;
    const TriMesh m2 = makeGrid(2, uv2);
    std::swap(uv2[id(0, 0) % 9], uv2[1]);
    const UVReport r2 = checkUV(m2, uv2);
    if (!r2.flippedFaces.empty()) CHECK(repairFlipsByKernel(m2, uv2, r2.flippedFaces) >= 0);   // must not crash

    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
