// The coarse-to-fine pipeline on a mesh small enough to compare with the direct one.
#include "core/MeshPrep.h"
#include "param/CoarsePipeline.h"
#include "param/UVMetrics.h"
#include "TestUtil.h"

#include <chrono>
#include <cstdio>

int main(int argc, char** argv)
{
    TriMesh m;
    CHECK(loadMesh(argc > 1 ? argv[1] : "data/bunny.ply", m));
    normalizeToUnitBox(m);

    CoarseOptions o;
    o.simplify.targetFaces = 6000;
    o.k = 20;
    auto t = std::chrono::steady_clock::now();
    const CoarseResult r = runCoarsePipeline(m, o);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
    const CoarseStats& s = r.stats;
    std::printf("coarse %zu faces, %d coarse patches -> %d fine patches: %d lifted, %d lift failed, %d without chart; "
                "%ld vertices solved again\n", s.coarseFaces, s.coarsePatches, s.finePatches, s.lifted, s.liftFailed,
                s.notLifted, s.outsideVertices);
    std::printf("seconds: simplify %.2f, coarse VSA %.2f, coarse refine %.2f, map %.2f, labels %.2f, lift %.2f, "
                "fine refine %.2f (total %.2f)\n", s.simplifySeconds, s.coarseVsaSeconds, s.coarseRefineSeconds,
                s.mapSeconds, s.labelSeconds, s.liftSeconds, s.fineRefineSeconds, secs);

    // every fine face belongs to a patch, every patch is parameterized injectively
    int unlabeled = 0, bad = 0, flipped = 0;
    for (int l : r.fine.labels) unlabeled += l < 0;
    for (size_t i = 0; i < r.fine.params.size(); ++i) {
        bad += r.fine.bad[i];
        flipped += r.fine.params[i].distortion.flipped;
    }
    std::printf("fine: %zu patches, %d bad, %d flipped triangles, %d unlabeled faces\n", r.fine.params.size(), bad, flipped, unlabeled);
    for (size_t i = 0; i < r.fine.params.size(); ++i)
        if (r.fine.bad[i] || r.fine.params[i].method == ParamMethod::None)
        {
            const PatchTopology& tp = r.fine.params[i].patch.topology;
            std::printf("  patch %zu (%zu faces): %s | components %d, loops %d, euler %d, vertex manifold %d\n", i,
                        r.fine.extraction.patches[i].faceToGlobal.size(), r.fine.params[i].log.c_str(), tp.components,
                        tp.boundaryLoops, tp.euler, int(tp.vertexManifold));
        }
    CHECK(unlabeled == 0);
    CHECK(bad == 0);
    CHECK(flipped == 0);
    CHECK(s.lifted > 0);

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("coarse pipeline: OK\n");
    return 0;
}
