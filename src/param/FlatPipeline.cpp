#include "param/FlatPipeline.h"

#include "param/FlatAdjacency.h"
#include "param/HierarchyMap.h"
#include "param/ThreadPool.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace {

using Clock = std::chrono::steady_clock;
double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

// ---------------------------------------------------------------- labels

struct Pieces {
    std::vector<int32_t> piece;     // per face
    std::vector<int>     coarseOf;  // per piece: the coarse patch it came from
    int count = 0;
};

// `label`: the coarse patch of every fine face. Single-face islands take the label of their neighbours, every
// label region is cut into connected pieces, pieces below minFaces join the neighbour they share most edges with.
Pieces makePieces(const std::vector<int32_t>& nbr, std::vector<int32_t> label, size_t minFaces, ThreadPool& pool)
{
    const size_t nF = label.size();
    // islands: a face without a neighbour of its own label
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<int32_t> next = label;
        const size_t chunk = 1 << 15, chunks = (nF + chunk - 1) / chunk;
        pool.parallelFor(int(chunks), [&](int c) {
            for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) {
                int32_t nl[3];
                int n = 0;
                bool same = false;
                for (int k = 0; k < 3; ++k) {
                    const int32_t g = nbr[3 * f + k];
                    if (g < 0) continue;
                    nl[n++] = label[g];
                    same = same || label[g] == label[f];
                }
                if (same || n == 0) continue;
                int32_t best = nl[0];
                int bestCount = 0;
                for (int i = 0; i < n; ++i) {
                    int cnt = 0;
                    for (int j = 0; j < n; ++j) cnt += nl[j] == nl[i];
                    if (cnt > bestCount || (cnt == bestCount && nl[i] < best)) { best = nl[i]; bestCount = cnt; }
                }
                next[f] = best;
            }
        });
        label.swap(next);
    }

    // connected pieces
    Pieces p;
    p.piece.assign(nF, -1);
    std::vector<int> pieceLabel;
    std::vector<size_t> size;
    std::vector<int32_t> stack;
    for (size_t f0 = 0; f0 < nF; ++f0) {
        if (p.piece[f0] >= 0) continue;
        const int id = int(pieceLabel.size());
        pieceLabel.push_back(label[f0]);
        size.push_back(0);
        p.piece[f0] = id;
        stack.assign(1, int32_t(f0));
        while (!stack.empty()) {
            const int32_t f = stack.back();
            stack.pop_back();
            ++size[id];
            for (int k = 0; k < 3; ++k) {
                const int32_t g = nbr[3 * size_t(f) + k];
                if (g >= 0 && p.piece[g] < 0 && label[g] == label[f0]) { p.piece[g] = id; stack.push_back(g); }
            }
        }
    }

    // tiny pieces join their best neighbour
    const int n0 = int(pieceLabel.size());
    std::vector<int> target(n0);
    for (int i = 0; i < n0; ++i) target[i] = i;
    auto resolve = [&](int i) { while (target[i] != i) i = target[i]; return i; };
    std::vector<int> small;
    for (int i = 0; i < n0; ++i) if (size[i] < minFaces) small.push_back(i);
    if (!small.empty()) {
        std::sort(small.begin(), small.end(), [&](int a, int b) { return size[a] != size[b] ? size[a] < size[b] : a < b; });
        std::vector<int> slot(n0, -1);
        for (size_t i = 0; i < small.size(); ++i) slot[small[i]] = int(i);
        std::vector<std::vector<int32_t>> members(small.size());
        for (size_t f = 0; f < nF; ++f)
            if (slot[p.piece[f]] >= 0) members[slot[p.piece[f]]].push_back(int32_t(f));
        for (size_t i = 0; i < small.size(); ++i) {
            const int me = small[i];
            std::vector<std::pair<int, int>> shared;   // neighbour piece -> shared edges
            for (int32_t f : members[i])
                for (int k = 0; k < 3; ++k) {
                    const int32_t g = nbr[3 * size_t(f) + k];
                    if (g < 0) continue;
                    const int other = resolve(p.piece[g]);
                    if (other == me) continue;
                    auto it = std::find_if(shared.begin(), shared.end(), [&](const auto& s) { return s.first == other; });
                    if (it == shared.end()) shared.push_back({other, 1}); else ++it->second;
                }
            if (shared.empty()) continue;   // a piece of its own: keep
            int best = shared[0].first, bestN = shared[0].second;
            for (const auto& s : shared) if (s.second > bestN || (s.second == bestN && s.first < best)) { best = s.first; bestN = s.second; }
            target[me] = best;
            size[best] += size[me];
        }
    }
    std::vector<int> newId(n0, -1);
    for (int i = 0; i < n0; ++i)
        if (resolve(i) == i) { newId[i] = p.count++; p.coarseOf.push_back(pieceLabel[i]); }
    for (size_t f = 0; f < nF; ++f) p.piece[f] = newId[resolve(p.piece[f])];
    return p;
}

// ---------------------------------------------------------------- one piece

struct SubMesh {
    TriMesh               mesh;
    std::vector<uint32_t> keptFaces;     // sub face -> fine face (faces OpenMesh refused are missing)
    std::vector<uint32_t> vertexGlobal;  // sub vertex -> fine vertex
};

SubMesh buildSubMesh(const FlatMesh& fine, const uint32_t* faces, size_t n)
{
    SubMesh s;
    s.vertexGlobal.reserve(n / 2 + 8);
    for (size_t i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) s.vertexGlobal.push_back(fine.idx[3 * size_t(faces[i]) + k]);
    std::sort(s.vertexGlobal.begin(), s.vertexGlobal.end());
    s.vertexGlobal.erase(std::unique(s.vertexGlobal.begin(), s.vertexGlobal.end()), s.vertexGlobal.end());
    s.mesh.reserve(s.vertexGlobal.size(), s.vertexGlobal.size() * 3, n);
    std::vector<TriMesh::VertexHandle> vh(s.vertexGlobal.size());
    for (size_t i = 0; i < vh.size(); ++i) {
        const size_t v = s.vertexGlobal[i];
        vh[i] = s.mesh.add_vertex(Vec3(fine.pos[3 * v], fine.pos[3 * v + 1], fine.pos[3 * v + 2]));
    }
    auto local = [&](uint32_t v) {
        return vh[size_t(std::lower_bound(s.vertexGlobal.begin(), s.vertexGlobal.end(), v) - s.vertexGlobal.begin())];
    };
    s.keptFaces.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const uint32_t* t = &fine.idx[3 * size_t(faces[i])];
        if (s.mesh.add_face(local(t[0]), local(t[1]), local(t[2])).is_valid()) s.keptFaces.push_back(faces[i]);
    }
    return s;
}

// Writes the corner UV of one parameterized patch and describes it.
void emitPatch(const FlatMesh& fine, const SubMesh& sm, const PatchParam& p, std::vector<float>& cornerUV, FlatPatch& out)
{
    const TriMesh& m = p.patch.mesh;
    out.method = p.method;
    out.injective = p.injective;
    out.needsSplit = p.needsSplit;
    out.badAreaFraction = p.badAreaFraction;
    out.distortion = p.distortion;
    out.faces.reserve(m.n_faces());
    for (auto f : m.faces()) {
        const uint32_t g = sm.keptFaces[p.patch.faceToGlobal[f.idx()]];
        out.faces.push_back(g);
        out.area3D += m.calc_face_area(f);
        Eigen::Vector2d q[3];
        int k = 0;
        for (auto v : m.fv_range(f)) {
            const uint32_t fineV = sm.vertexGlobal[p.patch.vertexToGlobal[v.idx()]];
            const Eigen::Vector2d& uv = p.uv[v.idx()];
            q[k++ % 3] = uv;
            for (int c = 0; c < 3; ++c)
                if (fine.idx[3 * size_t(g) + c] == fineV) {
                    cornerUV[6 * size_t(g) + 2 * c] = float(uv.x());
                    cornerUV[6 * size_t(g) + 2 * c + 1] = float(uv.y());
                }
        }
        out.uvArea += 0.5 * std::abs((q[1] - q[0]).x() * (q[2] - q[0]).y() - (q[1] - q[0]).y() * (q[2] - q[0]).x());
    }
    std::vector<int> index(m.n_vertices(), -1);
    for (auto h : m.halfedges()) {
        if (!m.is_boundary(h)) continue;
        int e[2];
        int k = 0;
        for (auto v : {m.from_vertex_handle(h), m.to_vertex_handle(h)}) {
            if (index[v.idx()] < 0) { index[v.idx()] = int(out.boundaryUV.size()); out.boundaryUV.push_back(p.uv[v.idx()]); }
            e[k++] = index[v.idx()];
        }
        out.boundaryEdges.push_back({e[0], e[1]});
    }
}

class Gate {
public:
    explicit Gate(size_t cap) : cap_(cap) {}
    size_t acquire(size_t n)
    {
        n = std::min(n, cap_);
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [&] { return used_ + n <= cap_; });
        used_ += n;
        return n;
    }
    void release(size_t n)
    {
        { std::lock_guard<std::mutex> lock(m_); used_ -= n; }
        cv_.notify_all();
    }
private:
    std::mutex m_;
    std::condition_variable cv_;
    size_t used_ = 0, cap_;
};


// Shortens the borders of the transferred labels: a face moves to a neighbouring region when it hangs on its own region by
// at most one edge and on the new one by at least two (spikes and notches), and the move lowers
//   E = sum err(f, label) + lambda_eff * (number of edges between different labels),
// err = 2 A (1 - n . N_label) (the L2,1 error against the normal of the coarse patch). No region is cut apart by such a
// move (the face is a leaf of its region). Decisions of one round are made from the labels of the round before; of two
// neighbouring faces that both want to move only the one with the lower index does. Deterministic.
void smoothFlatLabels(const FlatMesh& m, const std::vector<int32_t>& nbr, std::vector<int32_t>& label,
                      const std::vector<Eigen::Vector3d>& normalOf, double lambda, ThreadPool& pool, int maxRounds)
{
    const size_t nF = label.size();
    auto geom = [&](size_t f, double& area, Eigen::Vector3d& n) {
        const float* a = &m.pos[3 * size_t(m.idx[3 * f])];
        const float* b = &m.pos[3 * size_t(m.idx[3 * f + 1])];
        const float* c = &m.pos[3 * size_t(m.idx[3 * f + 2])];
        const Eigen::Vector3d e1(double(b[0]) - a[0], double(b[1]) - a[1], double(b[2]) - a[2]);
        const Eigen::Vector3d e2(double(c[0]) - a[0], double(c[1]) - a[1], double(c[2]) - a[2]);
        const Eigen::Vector3d cr = e1.cross(e2);
        const double len = cr.norm();
        area = 0.5 * len;
        n = len > 0 ? Eigen::Vector3d(cr / len) : Eigen::Vector3d(0, 0, 0);
    };
    auto err = [&](size_t f, int32_t l) {
        double area;
        Eigen::Vector3d n;
        geom(f, area, n);
        return 2.0 * area * (1.0 - n.dot(normalOf[l]));
    };
    const size_t chunk = 1 << 15, chunks = (nF + chunk - 1) / chunk;
    std::vector<double> partE(chunks, 0.0);
    std::vector<size_t> partP(chunks, 0);
    pool.parallelFor(int(chunks), [&](int c) {
        double e = 0;
        size_t pcount = 0;
        for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) {
            e += err(f, label[f]);
            for (int k = 0; k < 3; ++k) {
                const int32_t g = nbr[3 * f + k];
                pcount += g > int32_t(f) && label[g] != label[f];
            }
        }
        partE[c] = e;
        partP[c] = pcount;
    });
    double eRef = 0;
    size_t pRef = 0;
    for (size_t c = 0; c < chunks; ++c) { eRef += partE[c]; pRef += partP[c]; }
    if (eRef <= 1e-13 || pRef == 0) return;
    const double lam = lambda * eRef / double(pRef);

    std::vector<int32_t> want(nF, -1);
    for (int round = 0; round < maxRounds; ++round) {
        pool.parallelFor(int(chunks), [&](int c) {
            for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) {
                want[f] = -1;
                int32_t nl[3];
                int n = 0, same = 0;
                for (int k = 0; k < 3; ++k) {
                    const int32_t g = nbr[3 * f + k];
                    if (g < 0) continue;
                    nl[n++] = label[g];
                    same += label[g] == label[f];
                }
                if (same > 1 || same == n) continue;
                const double own = err(f, label[f]);
                double bestD = 0;
                int32_t best = -1;
                for (int i = 0; i < n; ++i) {
                    if (nl[i] == label[f]) continue;
                    int b = 0;
                    for (int j = 0; j < n; ++j) b += nl[j] == nl[i];
                    if (b < 2) continue;
                    const double d = err(f, nl[i]) - own + lam * double(same - b);
                    if (d < bestD || (d == bestD && best >= 0 && nl[i] < best)) { bestD = d; best = nl[i]; }
                }
                want[f] = best;
            }
        });
        std::vector<size_t> flips(chunks, 0);
        std::vector<int32_t> next = label;
        pool.parallelFor(int(chunks), [&](int c) {
            for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) {
                if (want[f] < 0) continue;
                bool yield = false;
                for (int k = 0; k < 3; ++k) {
                    const int32_t g = nbr[3 * f + k];
                    yield = yield || (g >= 0 && g < int32_t(f) && want[g] >= 0);
                }
                if (yield) continue;
                next[f] = want[f];
                ++flips[c];
            }
        });
        label.swap(next);
        size_t total = 0;
        for (size_t x : flips) total += x;
        if (total == 0) break;
    }
}

} // namespace

FlatResult runFlatPipeline(const FlatMesh& fine, const CoarseOptions& opt, size_t faceBudget)
{
    FlatResult res;
    FlatStats& st = res.stats;
    const size_t nF = fine.faceCount();
    ThreadPool& pool = ThreadPool::shared();

    CoarseStage stage = runCoarseStage(fine, opt, st.coarse);
    const RefineResult& coarse = stage.result;

    // ---- fine labels
    auto t = Clock::now();
    std::vector<int32_t> label;
    {
        const FineToCoarse map = mapVertices(fine, stage.flat);
        st.coarse.mapSeconds = since(t);
        const std::vector<int> l = transferLabels(fine, coarse.labels, map);
        label.assign(l.begin(), l.end());
    }
    t = Clock::now();
    const std::vector<int32_t> nbr = buildFaceNeighbors(fine, &pool);
    st.adjacencySeconds = since(t);
    t = Clock::now();
    if (opt.fineSmoothing > 0.0) {
        // the normal of every coarse patch: area weighted mean over its coarse faces
        std::vector<Eigen::Vector3d> normalOf(coarse.params.size() + 1, Eigen::Vector3d::Zero());
        const FlatMesh& cf = stage.flat;
        for (size_t f = 0; f < cf.faceCount(); ++f) {
            const int l = coarse.labels[f];
            if (l < 0 || size_t(l) >= normalOf.size()) continue;
            const float* a = &cf.pos[3 * size_t(cf.idx[3 * f])];
            const float* b = &cf.pos[3 * size_t(cf.idx[3 * f + 1])];
            const float* c = &cf.pos[3 * size_t(cf.idx[3 * f + 2])];
            const Eigen::Vector3d e1(double(b[0]) - a[0], double(b[1]) - a[1], double(b[2]) - a[2]);
            const Eigen::Vector3d e2(double(c[0]) - a[0], double(c[1]) - a[1], double(c[2]) - a[2]);
            normalOf[l] += 0.5 * e1.cross(e2);
        }
        for (auto& n : normalOf) if (n.norm() > 0) n.normalize();
        smoothFlatLabels(fine, nbr, label, normalOf, opt.fineSmoothing, pool, 30);
    }
    const Pieces pieces = makePieces(nbr, std::move(label), size_t(std::max(1, opt.refine.patch.minFaces)), pool);
    st.cleanSeconds = since(t);
    st.pieces = size_t(pieces.count);

    // faces of every piece
    std::vector<size_t> start(size_t(pieces.count) + 1, 0);
    for (size_t f = 0; f < nF; ++f) ++start[size_t(pieces.piece[f]) + 1];
    for (int i = 0; i < pieces.count; ++i) start[i + 1] += start[i];
    std::vector<uint32_t> faceList(nF);
    {
        std::vector<size_t> fill(start.begin(), start.end() - 1);
        for (size_t f = 0; f < nF; ++f) faceList[fill[pieces.piece[f]]++] = uint32_t(f);
    }

    // ---- lift every piece
    t = Clock::now();
    res.cornerUV.assign(nF * 6, std::nanf(""));
    std::vector<std::vector<FlatPatch>> out(pieces.count);
    std::vector<char> failed(pieces.count, 0);
    std::vector<int> order(pieces.count);
    for (int i = 0; i < pieces.count; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        const size_t sa = start[a + 1] - start[a], sb = start[b + 1] - start[b];
        return sa != sb ? sa > sb : a < b;
    });
    Gate gate(faceBudget);
    ThreadPool::shared().parallelFor(pieces.count, [&](int w) {
        const int j = order[w];
        const size_t n = start[j + 1] - start[j];
        const size_t held = gate.acquire(n);
        const int q = pieces.coarseOf[j];
        const bool chart = q >= 0 && q < int(coarse.params.size()) && coarse.params[q].injective && !coarse.params[q].uv.empty();
        bool ok = false;
        if (chart) {
            SubMesh sm = buildSubMesh(fine, &faceList[start[j]], n);
            const std::vector<int> zeros(sm.mesh.n_faces(), 0);
            PatchOptions po = opt.refine.patch;
            po.minFaces = 1;
            ExtractionResult ex = extractPatches(sm.mesh, zeros, po);
            if (ex.patches.size() == 1 && ex.patches[0].isDisk() && sm.keptFaces.size() == n) {
                PatchParam pp = liftPatch(ex.patches[0], coarse.params[q], opt.lift);
                if (!pp.needsSplit) {
                    out[j].resize(1);
                    emitPatch(fine, sm, pp, res.cornerUV, out[j][0]);
                    ok = true;
                } else if (std::getenv("VSA_VERBOSE")) {
                    std::fprintf(stderr, "[flat] piece %d (%zu faces): lift failed: %s", j, n, pp.log.c_str());
                }
            } else if (std::getenv("VSA_VERBOSE")) {
                std::fprintf(stderr, "[flat] piece %d (%zu faces): %zu patches, kind %s, kept %zu of %zu faces\n", j, n,
                             ex.patches.size(), ex.patches.empty() ? "-" : patchKindName(ex.patches[0].topology.kind),
                             sm.keptFaces.size(), n);
            }
        } else if (std::getenv("VSA_VERBOSE")) {
            std::fprintf(stderr, "[flat] piece %d (%zu faces): no coarse chart (coarse patch %d)\n", j, n, q);
        }
        failed[j] = !ok;
        gate.release(held);
    });
    st.liftSeconds = since(t);

    // ---- the pieces the lift could not handle: the ordinary pipeline on their own sub-mesh. Several at a time, each
    // with a pool of its own (a worker of the shared pool must not use it again).
    t = Clock::now();
    std::vector<int> redo;
    for (int j : order) {
        if (failed[j]) redo.push_back(j); else ++st.liftedPieces;
    }
    st.fallbackPieces = redo.size();
    Gate fallbackGate(faceBudget);
    std::mutex statMutex;
    ThreadPool::shared().parallelFor(int(redo.size()), [&](int w) {
        const int j = redo[w];
        const size_t n = start[j + 1] - start[j];
        const size_t held = fallbackGate.acquire(n * 2);
        ThreadPool inlinePool(1);
        RefineOptions ro = opt.refine;
        ro.pool = &inlinePool;
        // Split what does not lift: every part is lifted from the same coarse chart again (cheap), the ordinary ladder
        // is the last resort for tiny parts and pieces without a chart.
        const int q = pieces.coarseOf[j];
        const bool chart = q >= 0 && q < int(coarse.params.size()) && coarse.params[q].injective && !coarse.params[q].uv.empty();
        if (chart) {
            const LiftOptions lo = opt.lift;
            const PatchParam& cp = coarse.params[q];
            ro.parameterize = [&cp, lo](const TriMesh& source, const Patch& patch, const ParamOptions& popt) {
                if (patch.isDisk()) {
                    PatchParam pp = liftPatch(patch, cp, lo, nullptr, &source);
                    if (!pp.needsSplit || patch.faceToGlobal.size() > 64) return pp;
                }
                return parameterizePatch(source, patch, popt);
            };
        }
        SubMesh sm = buildSubMesh(fine, &faceList[start[j]], n);
        RefineResult rr = refinePatches(sm.mesh, std::vector<int>(sm.mesh.n_faces(), 0), ro);
        {
            std::lock_guard<std::mutex> lock(statMutex);
            st.facesRefused += n - sm.keptFaces.size();
        }
        for (size_t i = 0; i < rr.params.size(); ++i) {
            if (rr.params[i].uv.empty()) {   // unresolved: its faces keep no UV
                std::lock_guard<std::mutex> lock(statMutex);
                ++st.patchesUnresolved;
                st.facesUnresolved += rr.extraction.patches[i].faceToGlobal.size();
                if (std::getenv("VSA_VERBOSE"))
                    std::fprintf(stderr, "[flat] piece %d: unresolved patch of %zu faces, kind %s: %s", j, rr.extraction.patches[i].faceToGlobal.size(),
                                 patchKindName(rr.extraction.patches[i].topology.kind), rr.params[i].log.c_str());
                continue;
            }
            out[j].emplace_back();
            emitPatch(fine, sm, rr.params[i], res.cornerUV, out[j].back());
        }
        fallbackGate.release(held);
    });
    st.fallbackSeconds = since(t);

    // ---- assemble in piece order: deterministic
    res.patchOfFace.assign(nF, -1);
    for (int j = 0; j < pieces.count; ++j)
        for (FlatPatch& p : out[j]) {
            for (uint32_t f : p.faces) res.patchOfFace[f] = int32_t(res.patches.size());
            res.patches.push_back(std::move(p));
        }
    for (size_t f = 0; f < nF; ++f) st.facesWithoutUV += res.patchOfFace[f] < 0;
    return res;
}

AtlasResult packFlatAtlas(FlatResult& r, const HorizonOptions& options)
{
    std::vector<AtlasInput> in(r.patches.size());
    std::vector<char> usable(r.patches.size(), 0);
    std::vector<std::vector<Eigen::Vector2d>> centered(r.patches.size());   // border UV relative to the chart centre
    std::vector<Eigen::Vector2d> centre(r.patches.size(), Eigen::Vector2d::Zero());
    for (size_t i = 0; i < in.size(); ++i) {
        const FlatPatch& p = r.patches[i];
        // a degenerate chart (non-finite, or a sliver whose extent is absurd next to its area) cannot be packed
        bool ok = !p.boundaryUV.empty() && std::isfinite(p.uvArea) && p.uvArea > 0 && p.area3D > 0;
        double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
        for (const auto& q : p.boundaryUV) {
            ok = ok && std::isfinite(q.x()) && std::isfinite(q.y());
            if (!ok) break;
            for (int c = 0; c < 2; ++c) { lo[c] = std::min(lo[c], q[c]); hi[c] = std::max(hi[c], q[c]); }
        }
        if (ok) ok = std::max(hi[0] - lo[0], hi[1] - lo[1]) < 1e4 * std::sqrt(p.uvArea);
        usable[i] = ok;
        if (!ok) continue;
        centre[i] = Eigen::Vector2d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2);
        centered[i] = p.boundaryUV;
        for (auto& q : centered[i]) q -= centre[i];
        in[i].uv = &centered[i];
        in[i].boundaryEdges = &p.boundaryEdges;
        in[i].uvArea = p.uvArea;
        in[i].area3D = p.area3D;
    }
    AtlasResult res = packHorizon(in, options);
    if (!res.ok) return res;

    // the packer moves a chart by a similarity (rotation, uniform scale, translation): recover it from the border
    // points and apply it to every corner
    struct Sim { double ar = 1, ai = 0, br = 0, bi = 0; bool valid = false; };
    std::vector<Sim> sim(r.patches.size());
    for (size_t i = 0; i < sim.size(); ++i) {
        if (!usable[i]) continue;
        const auto& p = centered[i];
        const auto& q = res.uv[i];
        if (p.size() < 2 || q.size() != p.size()) continue;
        Eigen::Vector2d pm(0, 0), qm(0, 0);
        for (size_t k = 0; k < p.size(); ++k) { pm += p[k]; qm += q[k]; }
        pm /= double(p.size()); qm /= double(p.size());
        double nr = 0, ni = 0, den = 0;
        for (size_t k = 0; k < p.size(); ++k) {
            const Eigen::Vector2d a = p[k] - pm, b = q[k] - qm;   // b ~ (ar + i ai) * a
            nr += b.x() * a.x() + b.y() * a.y();
            ni += b.y() * a.x() - b.x() * a.y();
            den += a.squaredNorm();
        }
        if (den <= 0) continue;
        Sim s;
        s.ar = nr / den; s.ai = ni / den;
        s.br = qm.x() - (s.ar * pm.x() - s.ai * pm.y());
        s.bi = qm.y() - (s.ai * pm.x() + s.ar * pm.y());
        // fold the centring into the translation: z -> a (z - centre) + b
        s.br -= s.ar * centre[i].x() - s.ai * centre[i].y();
        s.bi -= s.ai * centre[i].x() + s.ar * centre[i].y();
        s.valid = true;
        sim[i] = s;
    }
    const size_t nF = r.patchOfFace.size();
    const size_t chunk = 1 << 15, chunks = (nF + chunk - 1) / chunk;
    ThreadPool::shared().parallelFor(int(chunks), [&](int c) {
        for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) {
            const int32_t pi = r.patchOfFace[f];
            if (pi < 0) continue;
            if (!sim[pi].valid) {   // not packed: no UV rather than a wrong one
                for (int k = 0; k < 6; ++k) r.cornerUV[6 * f + k] = std::nanf("");
                continue;
            }
            const Sim& s = sim[pi];
            for (int k = 0; k < 3; ++k) {
                const double u = r.cornerUV[6 * f + 2 * k], v = r.cornerUV[6 * f + 2 * k + 1];
                r.cornerUV[6 * f + 2 * k] = float(s.ar * u - s.ai * v + s.br);
                r.cornerUV[6 * f + 2 * k + 1] = float(s.ai * u + s.ar * v + s.bi);
            }
        }
    });
    return res;
}
