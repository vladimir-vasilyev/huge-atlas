#include "param/Refinement.h"

#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "param/ThreadPool.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <mach/mach.h>
#include <chrono>
#include <limits>
#include <queue>
#include <tuple>

namespace {

uint64_t hashFaces(const std::vector<int>& faces)
{
    uint64_t h = 1469598103934665603ULL;
    for (int f : faces) { h ^= uint64_t(uint32_t(f)); h *= 1099511628211ULL; }
    return h ^ (uint64_t(faces.size()) << 40);
}

ParamOptions ladderFor(const RefineOptions& o)
{
    ParamOptions p = o.param;
    if (o.policy == RefinePolicy::SplitImmediately) {
        p.useActiveSet = false;
        p.allowCut = false;
        p.allowTutte = false;
    }
    return p;
}

// Distances over the dual graph (edge = distance between face centroids) from the given faces.
std::vector<double> dualDistances(const TriMesh& m, const FaceData& fd, const std::vector<int>& sources,
                                  std::vector<int>* nearest = nullptr)
{
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(m.n_faces(), kInf);
    if (nearest) nearest->assign(m.n_faces(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    for (size_t i = 0; i < sources.size(); ++i) {
        dist[sources[i]] = 0.0;
        if (nearest) (*nearest)[sources[i]] = int(i);
        q.push({0.0, sources[i]});
    }
    while (!q.empty()) {
        const auto [d, u] = q.top();
        q.pop();
        if (d > dist[u]) continue;
        for (auto nb : m.ff_range(TriMesh::FaceHandle(u))) {
            const double nd = d + (fd.centroid[nb.idx()] - fd.centroid[u]).norm();
            if (nd < dist[nb.idx()]) {
                dist[nb.idx()] = nd;
                if (nearest) (*nearest)[nb.idx()] = (*nearest)[u];
                q.push({nd, nb.idx()});
            }
        }
    }
    return dist;
}

// Two faces that are far apart: double sweep over the dual graph.
std::pair<int, int> farthestPair(const TriMesh& m, const FaceData& fd)
{
    auto farthest = [&](int from) {
        const std::vector<double> d = dualDistances(m, fd, {from});
        int best = from;
        double bd = -1.0;
        for (size_t f = 0; f < d.size(); ++f)
            if (d[f] != std::numeric_limits<double>::infinity() && d[f] > bd && !fd.degenerate[f]) { bd = d[f]; best = int(f); }
        return best;
    };
    const int a = farthest(0);
    return {a, farthest(a)};
}

} // namespace

Refiner::Refiner(const TriMesh& mesh, std::vector<int> labels, const RefineOptions& options)
    : mesh_(mesh), opt_(options), labels_(std::move(labels))
{
    faceDepth_.assign(mesh.n_faces(), 0);
    for (int l : labels_) nextLabel_ = std::max(nextLabel_, l + 1);
    result_.stats.depthHistogram.assign(size_t(opt_.maxDepth) + 2, 0);
}

void Refiner::seed(ExtractionResult&& ex, std::vector<PatchParam>&& params)
{
    labels_ = ex.faceToPatch;
    nextLabel_ = int(ex.patches.size());
    result_.extraction = std::move(ex);
    result_.params = std::move(params);
}

void Refiner::evaluate()
{
    // What was computed in the previous round is reused for the patches whose faces did not change. The old
    // parameterizations are moved, not copied: a patch carries its mesh, and a copy per round is a copy of the model.
    std::unordered_map<uint64_t, int> previous;                // face set -> index in the previous result
    for (size_t i = 0; i < result_.extraction.patches.size(); ++i)
        previous[hashFaces(result_.extraction.patches[i].faceToGlobal)] = int(i);
    std::vector<PatchParam> oldParams = std::move(result_.params);
    // the previous patches stay (their meshes are empty where a parameterization owns them): a region that did not change is
    // not built again
    const std::vector<Patch> oldPatches = std::move(result_.extraction.patches);
    const PatchReuse reuse = [&](const std::vector<int>& faces) -> const Patch* {
        auto it = previous.find(hashFaces(faces));
        if (it == previous.end() || it->second >= int(oldPatches.size()) || it->second >= int(oldParams.size())) return nullptr;
        const Patch& old = oldPatches[it->second];
        if (old.faceToGlobal != faces || oldParams[it->second].method == ParamMethod::None) return nullptr;   // never parameterized: build it
        return &old;
    };

    const auto tExtract0 = std::chrono::steady_clock::now();
    ExtractionResult ex = extractPatches(mesh_, labels_, opt_.patch, &reuse);
    const double extractSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - tExtract0).count();
    const auto tLadder0 = std::chrono::steady_clock::now();
    labels_ = ex.faceToPatch;           // compact, merged labels (-1 stays -1)
    nextLabel_ = int(ex.patches.size());

    const ParamOptions ladder = ladderFor(opt_);
    std::vector<PatchParam> params(ex.patches.size());
    std::vector<char> bad(ex.patches.size(), 0);

    // the patches that were not seen before and are disks need the parameterization ladder: independent jobs,
    // the biggest first so that the threads finish together
    std::vector<int> work;
    for (size_t i = 0; i < ex.patches.size(); ++i) {
        auto it = previous.find(hashFaces(ex.patches[i].faceToGlobal));
        if (it != previous.end() && it->second < int(oldParams.size()) &&
            oldPatches[it->second].faceToGlobal == ex.patches[i].faceToGlobal) {
            params[i] = std::move(oldParams[it->second]);
        } else if (ex.patches[i].isDisk()) {
            work.push_back(int(i));
        } else {
            params[i].patch = ex.patches[i];
            params[i].needsSplit = true;
            params[i].log = std::string("not a disk: ") + patchKindName(ex.patches[i].topology.kind);
        }
    }
    std::sort(work.begin(), work.end(), [&](int a, int b) {
        return ex.patches[a].faceToGlobal.size() > ex.patches[b].faceToGlobal.size();
    });
    (opt_.pool ? *opt_.pool : ThreadPool::shared()).parallelFor(int(work.size()), [&](int t) {
        const int i = work[t];
        // Tutte (an expensive LU factorization, and always a bad result) is the last resort: a patch that can still
        // be split is simply split in the next round
        ParamOptions mine = ladder;
        const std::vector<int>& faces = ex.patches[i].faceToGlobal;
        const bool canSplit = faceDepth_[faces[0]] < opt_.maxDepth && int(faces.size()) >= 2 * opt_.minFaces;
        if (canSplit) mine.allowTutte = false;
        if (canSplit && opt_.splitAtOnceAboveFaces > 0 && int(faces.size()) > opt_.splitAtOnceAboveFaces) {
            mine.useActiveSet = false;
            mine.allowCut = false;
        }
        params[i] = opt_.parameterize ? opt_.parameterize(mesh_, ex.patches[i], mine) : parameterizePatch(mesh_, ex.patches[i], mine);
        // the mesh of the patch is in params[i].patch now: release the extracted copy at once, not at the end of the round
        if (params[i].patch.mesh.n_faces() == ex.patches[i].mesh.n_faces() && ex.patches[i].mesh.n_faces() > 0)
            ex.patches[i].mesh = TriMesh();
    });
    for (size_t i = 0; i < ex.patches.size(); ++i) bad[i] = params[i].needsSplit;
    if (std::getenv("VSA_VERBOSE"))
        std::fprintf(stderr, "[refine] evaluate: extraction %.1f s, parameterization of %zu new patches %.1f s\n", extractSeconds, work.size(),
                     std::chrono::duration<double>(std::chrono::steady_clock::now() - tLadder0).count());

    // the patch mesh now lives in params[i].patch: release the second copy
    for (size_t i = 0; i < ex.patches.size(); ++i)
        if (params[i].patch.mesh.n_faces() == ex.patches[i].mesh.n_faces() && ex.patches[i].mesh.n_faces() > 0)
            ex.patches[i].mesh = TriMesh();

    RefineStats stats = result_.stats;
    std::fill(stats.depthHistogram.begin(), stats.depthHistogram.end(), 0);
    int nbad = 0;
    for (size_t i = 0; i < ex.patches.size(); ++i) {
        nbad += bad[i];
        const int depth = std::min<int>(faceDepth_[ex.patches[i].faceToGlobal[0]], int(stats.depthHistogram.size()) - 1);
        ++stats.depthHistogram[depth];
    }
    if (stats.rounds == 0) stats.initialBad = nbad;
    stats.unresolved = nbad;

    result_.labels = labels_;
    result_.extraction = std::move(ex);
    result_.params = std::move(params);
    result_.bad = std::move(bad);
    result_.stats = std::move(stats);
}

bool Refiner::splitPatch(int pi)
{
    const Patch& patch = result_.extraction.patches[pi];
    const PatchParam& param = result_.params[pi];
    const std::vector<int>& faces = patch.faceToGlobal;
    const int depth = faceDepth_[faces[0]];
    auto why = [&](const char* reason) {
        if (std::getenv("VSA_VERBOSE"))
            std::fprintf(stderr, "[split] patch %d (%zu faces, %s, genus %d, depth %d) not split: %s\n", pi, faces.size(),
                         patchKindName(patch.topology.kind), patch.topology.genus, depth, reason);
        return false;
    };
    if (depth >= opt_.maxDepth) return why("max depth");
    if (int(faces.size()) < 2 * opt_.minFaces) return why("too few faces");

    // the whole patch without slits: the flooding needs the original adjacency
    Patch sub;
    if (!buildPatch(mesh_, faces, {}, sub)) return why("buildPatch failed");
    const FaceData fd = computeFaceData(sub.mesh);
    const std::vector<Moments> fm = computeFaceMoments(sub.mesh, fd);
    const VSAInput in{sub.mesh, fd, fm};

    // A patch with handles: bisecting it needs many rounds (the handle stays in one half). Several parts at once
    // cut the handles apart: k = 2 + 2 genus parts (at most 8) from farthest-point seeds, one local VSA.
    const int genus = patch.topology.genus;
    if (genus > 0 && (patch.topology.kind == PatchKind::Handles || patch.topology.kind == PatchKind::Closed)) {
        const int k = std::min(8, 2 + 2 * genus);
        if (int(faces.size()) >= k * opt_.minFaces) {
            VSAState s;
            s.pool = opt_.pool;
            s.metric = opt_.metric;
            s.smooth = opt_.smooth;
            initFarthestSeeds(in, s, k, 12345);
            runLloyd(in, s, opt_.lloyd);
            std::vector<int> count(std::max(1, s.k()), 0);
            for (int l : s.label) if (l >= 0) ++count[l];
            bool ok = s.k() >= 2;
            for (int c : count) ok = ok && c >= opt_.minFaces;
            if (ok) {
                std::vector<int> label(s.k());
                label[0] = labels_[sub.faceToGlobal[0]];   // part 0 keeps the label of the patch
                for (int j = 1; j < s.k(); ++j) label[j] = nextLabel_++;
                for (size_t i = 0; i < s.label.size(); ++i) {
                    const int g = sub.faceToGlobal[i];
                    labels_[g] = label[s.label[i]];
                    faceDepth_[g] = depth + 1;
                }
                ++result_.stats.splits;
                ++result_.stats.splitsByFarthest;
                return true;
            }
        }
    }

    std::unordered_map<int, int> local;
    for (size_t i = 0; i < sub.faceToGlobal.size(); ++i) local[sub.faceToGlobal[i]] = int(i);

    int seedA = -1, seedB = -1;
    enum { Hint, Farthest } seedKind = Farthest;
    if (param.hintFaceA >= 0 && param.hintFaceB >= 0 && param.hintFaceA != param.hintFaceB &&
        local.count(param.hintFaceA) && local.count(param.hintFaceB)) {
        seedA = local[param.hintFaceA];
        seedB = local[param.hintFaceB];
        seedKind = Hint;
    } else {
        std::tie(seedA, seedB) = farthestPair(sub.mesh, fd);
    }
    if (seedA == seedB || seedA < 0) return why("no seeds");

    auto sizes = [&](const std::vector<int>& lab) {
        int n[2] = {0, 0};
        for (int l : lab) if (l == 0 || l == 1) ++n[l];
        return std::pair<int, int>(n[0], n[1]);
    };
    // Both parts need minFaces; for a big patch also a real share of it. Peeling a few triangles off a huge patch
    // again and again never gets anywhere (the patch stays as it is), so such a split is rejected and the more
    // balanced fallbacks are tried. `balanced` is false for the last fallback, which only needs minFaces.
    const int total = int(faces.size());
    auto acceptable = [&](const std::vector<int>& lab, int k, bool balanced = true) {
        if (k != 2) return why("local VSA did not give two parts");
        const auto [n0, n1] = sizes(lab);
        const int share = (balanced && total >= 2000) ? std::max(opt_.minFaces, total / 10) : opt_.minFaces;
        return n0 >= share && n1 >= share;
    };

    std::vector<int> split, peel;
    double peelEnergy = 0.0;
    int kind = 0;   // 0 Lloyd (hint/farthest/normals), 1 flood, 2 Voronoi, 3 peel
    enum { ByHint, ByFarthest, ByNormals } seedsOfSplit = seedKind == Hint ? ByHint : ByFarthest;
    {
        // Several starts of the same two-proxy VSA, the best energy wins. Lloyd only finds a local minimum, and where it ends
        // depends on the seeds: faces on both sides of a crossing, or the two ends of the patch, may both lie on the same
        // smooth sheet, and then the two proxies never separate the sheets that a crease joins. The third start is the way
        // VSA itself adds a proxy: the face that the best single plane fits worst, and the face that is worst for the plane
        // of that first face.
        struct Start { int a, b, tag; };
        std::vector<Start> starts{{seedA, seedB, seedKind == Hint ? int(ByHint) : int(ByFarthest)}};
        // The careful way (several starts, parts tried by unfolding) is only for the stubborn patches: those that were split
        // already and came out bad again. A patch seen for the first time gets the cheap split from one start.
        const bool careful = depth >= opt_.carefulSplitFromDepth;
        if (careful && seedKind == Hint) {
            const auto far = farthestPair(sub.mesh, fd);
            if (far.first >= 0 && far.first != far.second) starts.push_back({far.first, far.second, int(ByFarthest)});
        }
        if (careful) {
            Moments whole;
            for (const Moments& m : fm) whole += m;
            const Proxy single = fitProxy(opt_.metric, whole);
            int worst = -1;
            double worstError = -1.0;
            for (size_t f = 0; f < fm.size(); ++f) {
                if (fd.degenerate[f]) continue;
                const double e = metricError(opt_.metric, fm[f], single);
                if (e > worstError) { worstError = e; worst = int(f); }
            }
            int other = -1;
            if (worst >= 0) {
                const Proxy first = fitProxy(opt_.metric, fm[worst]);
                worstError = -1.0;
                for (size_t f = 0; f < fm.size(); ++f) {
                    if (fd.degenerate[f] || int(f) == worst) continue;
                    const double e = metricError(opt_.metric, fm[f], first);
                    if (e > worstError) { worstError = e; other = int(f); }
                }
            }
            if (worst >= 0 && other >= 0) starts.push_back({worst, other, int(ByNormals)});
        }
        double bestEnergy = 0.0;
        int bestBad = 3;
        for (const Start& st : starts) {
            bool duplicate = false;
            for (const Start& earlier : starts)
                if (&earlier != &st && &earlier < &st && ((earlier.a == st.a && earlier.b == st.b) || (earlier.a == st.b && earlier.b == st.a))) duplicate = true;
            if (duplicate) continue;
            VSAState s;
            s.pool = opt_.pool;
            s.metric = opt_.metric;
            s.smooth = opt_.smooth;
            setSeeds(in, s, {st.a, st.b});
            const LloydResult lr = runLloyd(in, s, opt_.lloyd);
            const double energy = s.energyHistory.empty() ? 0.0 : s.energyHistory.back();
            if (std::getenv("VSA_VERBOSE")) {
                int n[2] = {0, 0};
                for (int l : s.label) if (l == 0 || l == 1) ++n[l];
                std::fprintf(stderr, "[split] patch %d: start %s (faces %d, %d): Lloyd %d iterations, parts %d + %d, energy %.5g\n", pi,
                             st.tag == ByHint ? "crossings" : (st.tag == ByFarthest ? "farthest pair" : "worst faces"), st.a, st.b,
                             lr.iterations, n[0], n[1], energy);
            }
            if (!acceptable(s.label, s.k())) {
                // a small part cut off a big patch: only used when nothing better works (below)
                const auto [p0, p1] = sizes(s.label);
                if (s.k() == 2 && p0 >= opt_.minFaces && p1 >= opt_.minFaces && (peel.empty() || energy < peelEnergy)) { peel = s.label; peelEnergy = energy; }
                continue;
            }
            // The energy only says how flat the parts are. What matters is that they unfold: for patches of moderate size the
            // parts are tried with plain LinABF, and a split whose parts both unfold beats one with a part that does not.
            int badParts = 0;
            if (careful && total <= opt_.scoreSplitsUpToFaces) {
                for (int part = 0; part < 2; ++part) {
                    std::vector<int> partFaces;
                    for (size_t i = 0; i < s.label.size(); ++i) if (s.label[i] == part) partFaces.push_back(int(i));
                    Patch pp;
                    bool bad = true;
                    if (buildPatch(sub.mesh, partFaces, {}, pp) && pp.isDisk()) {
                        ParamOptions plain = opt_.param;
                        plain.useActiveSet = false;
                        plain.allowCut = false;
                        plain.allowTutte = false;
                        const PatchParam r = parameterizePatch(sub.mesh, pp, plain);
                        bad = r.needsSplit;
                    }
                    badParts += bad;
                }
            }
            if (std::getenv("VSA_VERBOSE")) std::fprintf(stderr, "[split] patch %d:   parts that do not unfold with plain LinABF: %d\n", pi, badParts);
            if (split.empty() || badParts < bestBad || (badParts == bestBad && energy < bestEnergy)) {
                split = s.label;
                bestEnergy = energy;
                bestBad = badParts;
                seedsOfSplit = st.tag == ByHint ? ByHint : (st.tag == ByFarthest ? ByFarthest : ByNormals);
            }
        }
    }
    if (split.empty()) {
        VSAState s;
        s.pool = opt_.pool;
        s.metric = opt_.metric;
        setSeeds(in, s, {seedA, seedB});
        partition(in, s);
        if (acceptable(s.label, s.k())) { split = s.label; kind = 1; }
    }
    if (split.empty()) {
        std::vector<int> nearest;
        dualDistances(sub.mesh, fd, {seedA, seedB}, &nearest);
        if (acceptable(nearest, 2, false)) { split = nearest; kind = 2; }
    }
    if (split.empty() && !peel.empty()) {   // peeling a few faces off a big patch beats leaving it as it is
        split = peel;
        kind = 3;
        if (std::getenv("VSA_VERBOSE")) std::fprintf(stderr, "[split] patch %d: peeled a small part off\n", pi);
    }
    if (split.empty()) return why("empty split");

    const int newLabel = nextLabel_++;
    for (size_t i = 0; i < split.size(); ++i) {
        const int g = sub.faceToGlobal[i];
        if (split[i] == 1) labels_[g] = newLabel;
        faceDepth_[g] = depth + 1;
    }
    ++result_.stats.splits;
    if (kind == 3) ++result_.stats.splitsByPeel;
    else if (kind == 1) ++result_.stats.splitsByFlood;
    else if (kind == 2) ++result_.stats.splitsByVoronoi;
    else if (seedsOfSplit == ByHint) ++result_.stats.splitsByCrossingHint;
    else if (seedsOfSplit == ByNormals) ++result_.stats.splitsByNormals;
    else ++result_.stats.splitsByFarthest;
    return true;
}

bool Refiner::step()
{
    if (finished_) return false;
    const auto t0 = std::chrono::steady_clock::now();
    evaluate();

    // the largest bad patches first
    std::vector<int> order;
    for (size_t i = 0; i < result_.bad.size(); ++i) if (result_.bad[i]) order.push_back(int(i));
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return result_.extraction.patches[a].faceToGlobal.size() > result_.extraction.patches[b].faceToGlobal.size();
    });

    int done = 0;
    if (result_.stats.rounds < opt_.maxRounds)
        for (int pi : order) {
            if (int(result_.extraction.patches.size()) + done >= opt_.maxPatches) break;
            if (splitPatch(pi)) ++done;
        }
    ++result_.stats.rounds;
    if (std::getenv("VSA_VERBOSE")) {   // per round: how many patches, how many are bad, how much was split, the memory
        task_vm_info_data_t vm;
        mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
        task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&vm), &count);
        size_t biggest = 0;
        for (const Patch& p : result_.extraction.patches) biggest = std::max(biggest, p.faceToGlobal.size());
        std::fprintf(stderr, "[refine] round %d: %zu patches (biggest %zu faces), %zu bad, %d split, footprint %.0f MB, peak %.0f MB\n",
                     result_.stats.rounds, result_.extraction.patches.size(), biggest, order.size(), done,
                     vm.phys_footprint / 1e6, vm.ledger_phys_footprint_peak / 1e6);
    }
    result_.stats.seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (done == 0) { finished_ = true; return false; }
    return true;
}

RefineResult refinePatches(const TriMesh& mesh, std::vector<int> labels, const RefineOptions& options)
{
    Refiner r(mesh, std::move(labels), options);
    while (r.step()) {}
    return r.takeResult();
}
