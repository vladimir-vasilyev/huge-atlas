#include "param/Refinement.h"

#include "core/MeshPrep.h"
#include "core/Operations.h"

#include <algorithm>
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

void Refiner::evaluate()
{
    ExtractionResult ex = extractPatches(mesh_, labels_, opt_.patch);
    labels_ = ex.faceToPatch;           // compact, merged labels (-1 stays -1)
    nextLabel_ = int(ex.patches.size());

    const ParamOptions ladder = ladderFor(opt_);
    std::vector<PatchParam> params(ex.patches.size());
    std::vector<char> bad(ex.patches.size(), 0);
    std::unordered_map<uint64_t, PatchParam> fresh;
    for (size_t i = 0; i < ex.patches.size(); ++i) {
        const uint64_t key = hashFaces(ex.patches[i].faceToGlobal);
        auto it = cache_.find(key);
        if (it != cache_.end() && it->second.patch.faceToGlobal == ex.patches[i].faceToGlobal) {
            params[i] = it->second;
        } else if (ex.patches[i].isDisk()) {
            params[i] = parameterizePatch(mesh_, ex.patches[i], ladder);
        } else {
            params[i].patch = ex.patches[i];
            params[i].needsSplit = true;
            params[i].log = std::string("not a disk: ") + patchKindName(ex.patches[i].topology.kind);
        }
        bad[i] = params[i].needsSplit;
        fresh[key] = params[i];
    }
    cache_ = std::move(fresh);

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
    if (depth >= opt_.maxDepth || int(faces.size()) < 2 * opt_.minFaces) return false;

    // the whole patch without slits: the flooding needs the original adjacency
    Patch sub;
    if (!buildPatch(mesh_, faces, {}, sub)) return false;
    const FaceData fd = computeFaceData(sub.mesh);
    const std::vector<Moments> fm = computeFaceMoments(sub.mesh, fd);
    const VSAInput in{sub.mesh, fd, fm};

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
    if (seedA == seedB || seedA < 0) return false;

    auto sizes = [&](const std::vector<int>& lab) {
        int n[2] = {0, 0};
        for (int l : lab) if (l == 0 || l == 1) ++n[l];
        return std::pair<int, int>(n[0], n[1]);
    };
    auto acceptable = [&](const std::vector<int>& lab, int k) {
        if (k != 2) return false;
        const auto [n0, n1] = sizes(lab);
        return n0 >= opt_.minFaces && n1 >= opt_.minFaces;
    };

    std::vector<int> split;
    int kind = 0;   // 0 Lloyd (hint/farthest), 1 flood, 2 Voronoi
    {
        VSAState s;
        s.metric = opt_.metric;
        s.smooth = opt_.smooth;
        setSeeds(in, s, {seedA, seedB});
        runLloyd(in, s, opt_.lloyd);
        if (acceptable(s.label, s.k())) split = s.label;
    }
    if (split.empty()) {
        VSAState s;
        s.metric = opt_.metric;
        setSeeds(in, s, {seedA, seedB});
        partition(in, s);
        if (acceptable(s.label, s.k())) { split = s.label; kind = 1; }
    }
    if (split.empty()) {
        std::vector<int> nearest;
        dualDistances(sub.mesh, fd, {seedA, seedB}, &nearest);
        if (acceptable(nearest, 2)) { split = nearest; kind = 2; }
    }
    if (split.empty()) return false;

    const int newLabel = nextLabel_++;
    for (size_t i = 0; i < split.size(); ++i) {
        const int g = sub.faceToGlobal[i];
        if (split[i] == 1) labels_[g] = newLabel;
        faceDepth_[g] = depth + 1;
    }
    ++result_.stats.splits;
    if (kind == 1) ++result_.stats.splitsByFlood;
    else if (kind == 2) ++result_.stats.splitsByVoronoi;
    else if (seedKind == Hint) ++result_.stats.splitsByCrossingHint;
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
    result_.stats.seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (done == 0) { finished_ = true; return false; }
    return true;
}

RefineResult refinePatches(const TriMesh& mesh, std::vector<int> labels, const RefineOptions& options)
{
    Refiner r(mesh, std::move(labels), options);
    while (r.step()) {}
    return r.result();
}
