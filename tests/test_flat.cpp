// The flat pipeline (no global TriMesh of the fine mesh) on the bunny: every face gets a clean UV, the atlas is
// valid.
#include "core/MeshPrep.h"
#include "param/FlatAdjacency.h"
#include "param/FlatPipeline.h"
#include "param/UVMetrics.h"
#include "TestUtil.h"

#include <chrono>
#include <cstdio>

int main(int argc, char** argv)
{
    TriMesh m;
    CHECK(loadMesh(argc > 1 ? argv[1] : "data/bunny.ply", m));
    normalizeToUnitBox(m);
    const FlatMesh fine = flatFromTriMesh(m);

    // neighbours: symmetric
    const std::vector<int32_t> nbr = buildFaceNeighbors(fine);
    size_t asym = 0, withNeighbor = 0;
    for (size_t f = 0; f < fine.faceCount(); ++f)
        for (int k = 0; k < 3; ++k) {
            const int32_t g = nbr[3 * f + k];
            if (g < 0) continue;
            ++withNeighbor;
            bool back = false;
            for (int j = 0; j < 3; ++j) back = back || nbr[3 * size_t(g) + j] == int32_t(f);
            asym += !back;
        }
    CHECK(asym == 0);
    CHECK(withNeighbor > fine.faceCount() * 2);

    CoarseOptions o;
    o.simplify.targetFaces = 6000;
    o.k = 20;
    auto t = std::chrono::steady_clock::now();
    FlatResult r = runFlatPipeline(fine, o);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
    const FlatStats& s = r.stats;
    std::printf("%zu pieces (%zu lifted, %zu fallback), %zu patches, %zu faces without UV; %.2f s "
                "(adjacency %.2f, clean %.2f, lift %.2f, fallback %.2f)\n", s.pieces, s.liftedPieces, s.fallbackPieces,
                r.patches.size(), s.facesWithoutUV, secs, s.adjacencySeconds, s.cleanSeconds, s.liftSeconds, s.fallbackSeconds);
    CHECK(s.facesWithoutUV == 0);
    CHECK(s.liftedPieces > 0);
    for (const FlatPatch& p : r.patches) CHECK(p.injective);

    // before packing: the corner UV of every patch is a valid map
    const UVLayoutStats before = analyzeUVLayout(m, r.cornerUV, 1024);
    std::printf("unpacked: flipped %d, L2 mean %.3f\n", before.flipped, before.l2Mean);
    CHECK(before.flipped == 0);

    HorizonOptions ho;
    ho.resolution = 1024;
    const AtlasResult ar = packFlatAtlas(r, ho);
    CHECK(ar.ok);
    const UVLayoutStats after = analyzeUVLayout(m, r.cornerUV, ar.resolution);
    std::printf("packed %d x %d: flipped %d, L2 mean %.3f, overlap %.4f%%, covered %.1f%%\n", ar.resolution, ar.resolution,
                after.flipped, after.l2Mean, 100 * after.overlap, 100 * after.covered);
    CHECK(after.flipped == 0);
    CHECK(after.overlap < 1e-4);
    CHECK(std::abs(after.l2Mean - before.l2Mean) < 1e-3);   // the packer only rescales

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("flat pipeline: OK\n");
    return 0;
}
