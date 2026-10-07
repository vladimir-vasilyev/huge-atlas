// The flat pipeline (no global TriMesh of the fine mesh) on the bunny: every face gets a clean UV, the atlas is
// valid.
#include "core/MeshPrep.h"
#include "param/FlatAdjacency.h"
#include "param/FlatBake.h"
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

    // retexturing: an old planar UV and a gradient texture are baked into the atlas; a point of the surface must show the same
    // colour through the old uv and the old texture as through the new uv and the new texture
    {
        std::vector<float> oldUV(fine.faceCount() * 6);
        for (size_t fc = 0; fc < fine.faceCount(); ++fc)
            for (int k = 0; k < 3; ++k) {
                const float* p = &fine.pos[3 * size_t(fine.idx[3 * fc + k])];
                oldUV[6 * fc + 2 * k] = p[0] + 0.5f;
                oldUV[6 * fc + 2 * k + 1] = p[1] + 0.5f;
            }
        const int W = 256;
        std::vector<unsigned char> tex(size_t(W) * W * 4, 255);
        for (int y = 0; y < W; ++y)
            for (int x = 0; x < W; ++x) {
                unsigned char* px = &tex[(size_t(y) * W + x) * 4];
                px[0] = (unsigned char)x; px[1] = (unsigned char)y; px[2] = 100;
            }
        BakeSource src{W, W, tex.data()};
        BakeOptions bo;
        bo.resolution = ar.resolution;
        bo.margin = ar.margin;
        const std::vector<unsigned char> baked = bakeAtlasFlat(fine, r, oldUV, {}, 0, &src, bo);
        CHECK(baked.size() == size_t(bo.resolution) * bo.resolution * 4);
        double sum = 0;
        size_t n = 0, bad = 0, uncovered = 0;
        for (size_t fc = 0; fc < fine.faceCount(); fc += 7) {
            double u = 0, v = 0, ou = 0, ov = 0;
            for (int k = 0; k < 3; ++k) {
                u += r.cornerUV[6 * fc + 2 * k] / 3.0; v += r.cornerUV[6 * fc + 2 * k + 1] / 3.0;
                ou += oldUV[6 * fc + 2 * k] / 3.0; ov += oldUV[6 * fc + 2 * k + 1] / 3.0;
            }
            const int x = std::min(bo.resolution - 1, int(u * bo.resolution)), y = std::min(bo.resolution - 1, int(v * bo.resolution));
            const unsigned char* px = &baked[(size_t(y) * bo.resolution + x) * 4];
            if (px[3] != 255) { ++uncovered; continue; }
            const double dr = std::abs(px[0] - ou * (W - 1)), dg = std::abs(px[1] - ov * (W - 1));
            sum += 0.5 * (dr + dg);
            bad += (dr > 30 || dg > 30);
            ++n;
        }
        std::printf("baked texture: mean error %.2f of 255 over %zu faces, %zu above 30, %zu centroids not covered\n", n ? sum / n : 0.0, n, bad, uncovered);
        CHECK(uncovered == 0);
        CHECK(n > 0 && sum / n < 6.0);
        CHECK(bad < n / 100 + 1);
    }

    // the PLY with the atlas UV: welded vertices, same triangles, UV in [0, 1]
    {
        const std::string path = "/tmp/vsa_flat_atlas_test.ply";
        std::string err;
        CHECK(writeFlatPlyAtlas(path, fine, r, nullptr, &err));
        FlatMesh back;
        CHECK(loadFlatMesh(path, back, nullptr, &err));
        CHECK(back.faceCount() == fine.faceCount());
        CHECK(back.vertexCount() >= fine.vertexCount() && back.vertexCount() <= 3 * fine.faceCount());
        size_t wrong = 0;
        for (size_t fc = 0; fc < fine.faceCount(); ++fc)
            for (int k = 0; k < 3; ++k)
                for (int c = 0; c < 3; ++c)
                    wrong += back.pos[3 * size_t(back.idx[3 * fc + k]) + c] != fine.pos[3 * size_t(fine.idx[3 * fc + k]) + c];
        CHECK(wrong == 0);
        std::printf("atlas PLY: %zu vertices for %zu original (seams duplicate), triangles preserved\n", back.vertexCount(), fine.vertexCount());
    }

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("flat pipeline: OK\n");
    return 0;
}
