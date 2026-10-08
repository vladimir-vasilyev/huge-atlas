#include "param/CoarsePipeline.h"

#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "core/Smoothing.h"
#include "param/HierarchyMap.h"
#include "param/ThreadPool.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace {

using Clock = std::chrono::steady_clock;
double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

TriMesh triMeshFromFlat(const FlatMesh& f)
{
    TriMesh m;
    m.reserve(f.vertexCount(), f.vertexCount() * 3, f.faceCount());
    std::vector<TriMesh::VertexHandle> v(f.vertexCount());
    for (size_t i = 0; i < v.size(); ++i) v[i] = m.add_vertex(Vec3(f.pos[3 * i], f.pos[3 * i + 1], f.pos[3 * i + 2]));
    for (size_t i = 0; i < f.faceCount(); ++i) m.add_face(v[f.idx[3 * i]], v[f.idx[3 * i + 1]], v[f.idx[3 * i + 2]]);
    return m;
}

// A region the transfer left in several pieces becomes one region per piece (each piece lifts from the same chart:
// patches are packed separately, so they may share the coarse chart). Small pieces are merged by the extraction.
void splitComponents(const TriMesh& m, std::vector<int>& label)
{
    std::vector<int> out(label.size(), -1);
    int next = 0;
    std::vector<int> stack;
    for (size_t f0 = 0; f0 < label.size(); ++f0) {
        if (out[f0] >= 0 || label[f0] < 0) continue;
        out[f0] = next;
        stack.assign(1, int(f0));
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            for (auto nb : m.ff_range(TriMesh::FaceHandle(f)))
                if (out[nb.idx()] < 0 && label[nb.idx()] == label[f0]) { out[nb.idx()] = next; stack.push_back(nb.idx()); }
        }
        ++next;
    }
    label = std::move(out);
}

} // namespace

CoarseStage runCoarseStage(const FlatMesh& fineFlat, const CoarseOptions& opt, CoarseStats& st)
{
    CoarseStage cs;
    auto say = [&](const char* m) { if (opt.onStage) opt.onStage(m); };
    say("simplifying the mesh...");
    auto t = Clock::now();
    SimplifyOptions so = opt.simplify;
    so.prune = so.prune || opt.pruneSmallParts;
    const SimplifyResult simp = simplifyMesh(fineFlat, so);
    st.simplifySeconds = since(t);
    st.coarseFaces = simp.mesh.faceCount();
    cs.mesh = triMeshFromFlat(simp.mesh);
    // where the half-edge mesh refused a face there is a hole: a patch around it is a ring that must be cut open, and cut patches
    // almost never unfold without overlapping (they were 37 of 98 patches on the Doom scan, 13 after closing the holes)
    if (opt.fillHolesUpTo > 0) st.holesClosed = fillSmallHoles(cs.mesh, size_t(opt.fillHolesUpTo));
    // the faces OpenMesh refused (complex edges) are dropped: index the flat mesh like the TriMesh from here on
    cs.flat = flatFromTriMesh(cs.mesh);

    say("segmenting the simplified mesh (VSA)...");
    t = Clock::now();
    FaceData cfd = computeFaceData(cs.mesh);
    smoothFaceNormals(cs.mesh, cfd, opt.normals);
    const std::vector<Moments> cfm = computeFaceMoments(cs.mesh, cfd);
    const VSAInput cin{cs.mesh, cfd, cfm};
    VSAState vs;
    vs.metric = opt.metric;
    vs.smooth = opt.smooth;
    initFarthestSeeds(cin, vs, opt.k, 1);
    runLloyd(cin, vs, opt.lloyd);
    st.coarseVsaSeconds = since(t);

    say("parameterizing the patches of the simplified mesh...");
    t = Clock::now();
    cs.result = refinePatches(cs.mesh, vs.label, opt.refine);
    st.coarseRefineSeconds = since(t);
    st.coarsePatches = int(cs.result.params.size());
    st.coarseInitialBad = cs.result.stats.initialBad;
    st.coarseSplits = cs.result.stats.splits;
    st.coarseRounds = cs.result.stats.rounds;
    st.coarseUnresolved = cs.result.stats.unresolved;
    return cs;
}

CoarseResult runCoarsePipeline(const TriMesh& fine, const CoarseOptions& opt)
{
    CoarseResult res;
    CoarseStats& st = res.stats;

    const FlatMesh fineFlat = flatFromTriMesh(fine);
    CoarseStage stage = runCoarseStage(fineFlat, opt, st);
    const FlatMesh& coarseFlat = stage.flat;
    RefineResult& coarseResult = stage.result;
    auto t = Clock::now();

    // ---- fine labels
    t = Clock::now();
    const FineToCoarse map = mapVertices(fineFlat, coarseFlat);
    st.mapSeconds = since(t);
    t = Clock::now();
    std::vector<int> coarseOfFine = transferLabels(fineFlat, coarseResult.labels, map);   // coarse patch id per fine face
    std::vector<int> label = coarseOfFine;
    int nLabels = 0;
    for (int l : label) nLabels = std::max(nLabels, l + 1);
    if (opt.fineSmoothing > 0.0 && nLabels >= 2) {
        const FaceData fd = computeFaceData(fine);   // raw normals: the labels come from the (smoothed) coarse VSA, and the serial smoothing costs seconds per pass here
        const std::vector<Moments> fm = computeFaceMoments(fine, fd);
        const VSAInput fin{fine, fd, fm};
        VSAState fs;
        fs.metric = opt.metric;
        fs.smooth = {opt.fineSmoothing, 0.0, 10};
        fs.proxies.assign(nLabels, Proxy());
        fs.label = label;
        fitProxies(fin, fs);
        smoothLabels(fin, fs);
        label = fs.label;
    }
    splitComponents(fine, label);
    st.labelSeconds = since(t);

    // ---- fine patches and the lift
    t = Clock::now();
    ExtractionResult ex = extractPatches(fine, label, opt.refine.patch);
    st.finePatches = int(ex.patches.size());
    std::vector<PatchParam> params(ex.patches.size());
    std::vector<int> work;
    std::vector<int> coarseOf(ex.patches.size(), -1);
    {
        std::vector<std::vector<int>> votes(ex.patches.size());
        for (size_t i = 0; i < ex.patches.size(); ++i) {
            std::vector<int> count;
            for (int f : ex.patches[i].faceToGlobal) {
                const int c = coarseOfFine[f];
                if (c < 0) continue;
                if (size_t(c) >= count.size()) count.resize(size_t(c) + 1, 0);
                ++count[c];
            }
            if (!count.empty()) coarseOf[i] = int(std::max_element(count.begin(), count.end()) - count.begin());
        }
    }
    for (size_t i = 0; i < ex.patches.size(); ++i) {
        const int q = coarseOf[i];
        const bool usable = q >= 0 && q < int(coarseResult.params.size()) && coarseResult.params[q].injective &&
                            !coarseResult.params[q].uv.empty() && ex.patches[i].isDisk();
        if (usable) work.push_back(int(i));
        else { ++st.notLifted; params[i].patch = ex.patches[i]; params[i].needsSplit = true; params[i].log = "no coarse chart to lift\n"; }
    }
    std::sort(work.begin(), work.end(), [&](int a, int b) {
        return ex.patches[a].faceToGlobal.size() > ex.patches[b].faceToGlobal.size();
    });
    std::vector<LiftStats> lstats(ex.patches.size());
    ThreadPool::shared().parallelFor(int(work.size()), [&](int w) {
        const int i = work[w];
        params[i] = liftPatch(ex.patches[i], coarseResult.params[coarseOf[i]], opt.lift, &lstats[i]);
        if (params[i].patch.mesh.n_faces() == ex.patches[i].mesh.n_faces()) ex.patches[i].mesh = TriMesh();   // one copy only
    });
    for (int i : work) {
        st.outsideVertices += lstats[i].outside;
        if (params[i].needsSplit) ++st.liftFailed; else ++st.lifted;
        if (params[i].needsSplit && std::getenv("VSA_VERBOSE"))
            std::fprintf(stderr, "[lift] patch %d (%zu faces, coarse %d): %s", i, ex.patches[i].faceToGlobal.size(), coarseOf[i],
                         params[i].log.c_str());
    }
    st.liftSeconds = since(t);

    // ---- what did not survive goes through the ordinary refinement on the fine mesh
    t = Clock::now();
    Refiner refiner(fine, ex.faceToPatch, opt.refine);
    refiner.seed(std::move(ex), std::move(params));
    while (refiner.step()) {}
    res.fine = refiner.takeResult();
    st.fineRefineSeconds = since(t);
    return res;
}
