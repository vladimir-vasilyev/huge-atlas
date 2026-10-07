// Fine -> coarse correspondence: exactness of the closest-triangle grid, survivors, label transfer, determinism.
#include "core/MeshPrep.h"
#include "param/HierarchyMap.h"
#include "param/ThreadPool.h"
#include "TestUtil.h"

#include <cstdio>
#include <random>

// Brute-force closest point distance (squared) of p to the triangles of m.
static double bruteForce(const FlatMesh& m, const float p[3])
{
    TriangleGrid single(m);   // the reference is the same triangle test, but over every face without the grid
    (void)single;
    double best = 1e300;
    for (size_t f = 0; f < m.faceCount(); ++f) {
        // sample-free exact test: reuse the grid class on one triangle
        FlatMesh one;
        one.pos.resize(9);
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c) one.pos[3 * k + c] = m.pos[3 * size_t(m.idx[3 * f + k]) + c];
        one.idx = {0, 1, 2};
        const TriangleGrid g(one);
        best = std::min(best, g.closest(p).dist2);
    }
    return best;
}

int main(int argc, char** argv)
{
    TriMesh m;
    CHECK(loadMesh(argc > 1 ? argv[1] : "data/bunny.ply", m));
    normalizeToUnitBox(m);
    const FlatMesh fine = flatFromTriMesh(m);
    SimplifyOptions so;
    so.targetFaces = 4000;
    const SimplifyResult sr = simplifyMesh(fine, so);
    const FlatMesh& coarse = sr.mesh;

    // 1. grid answers equal brute force, for points on, near and far from the surface
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> uni(-0.7f, 0.7f);
    const TriangleGrid grid(coarse);
    int mismatches = 0;
    for (int i = 0; i < 150; ++i) {
        float p[3] = {uni(rng), uni(rng), uni(rng)};
        if (i % 3 == 0) for (int c = 0; c < 3; ++c) p[c] = fine.pos[3 * size_t(rng() % fine.vertexCount()) + c];
        const double want = bruteForce(coarse, p), got = grid.closest(p).dist2;
        if (std::abs(want - got) > 1e-12 + 1e-9 * want) ++mismatches;
    }
    CHECK(mismatches == 0);

    // 2. the map: survivors sit on their coarse triangle, everything else close to the surface
    ThreadPool one(1), many(8);
    const FineToCoarse a = mapVertices(fine, coarse, &one);
    const FineToCoarse b = mapVertices(fine, coarse, &many);
    CHECK(a.face == b.face);
    CHECK(a.dist == b.dist);
    float worst = 0;
    for (size_t v = 0; v < fine.vertexCount(); ++v) {
        CHECK(a.face[v] != UINT32_MAX);
        worst = std::max(worst, a.dist[v]);
    }
    for (size_t cv = 0; cv < coarse.vertexCount(); ++cv)
        CHECK(a.dist[sr.vertexOrigin[cv]] < 1e-6f);
    std::printf("coarse %zu faces: worst vertex distance %.5f (simplifier error %.5f)\n", coarse.faceCount(), worst, sr.error);
    CHECK(worst < 0.02f);

    // 3. labels: three slabs of the coarse mesh, transferred to the fine mesh
    std::vector<int> coarseLabel(coarse.faceCount());
    auto slab = [](float x) { return x < -0.15f ? 0 : (x < 0.15f ? 1 : 2); };
    for (size_t f = 0; f < coarse.faceCount(); ++f) {
        float x = 0;
        for (int k = 0; k < 3; ++k) x += coarse.pos[3 * size_t(coarse.idx[3 * f + k])];
        coarseLabel[f] = slab(x / 3.f);
    }
    const std::vector<int> la = transferLabels(fine, coarseLabel, a, &one);
    const std::vector<int> lb = transferLabels(fine, coarseLabel, a, &many);
    CHECK(la == lb);
    size_t agree = 0;
    for (size_t f = 0; f < fine.faceCount(); ++f) {
        float x = 0;
        for (int k = 0; k < 3; ++k) x += fine.pos[3 * size_t(fine.idx[3 * f + k])];
        agree += la[f] == slab(x / 3.f);
    }
    std::printf("label transfer: %.2f%% of the fine faces agree with their own slab\n", 100.0 * agree / fine.faceCount());
    CHECK(agree > fine.faceCount() * 98 / 100);

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("hierarchy: OK\n");
    return 0;
}
