#include "core/Smoothing.h"

#include "core/ParallelUtil.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kGap = -2;   // marks a missing face (mesh border) in a ring

// Labels of the faces around triangle f in cyclic order (ccw), excluding f: the faces that share a
// vertex with it. Gaps at the mesh border are marked with kGap.
void ringLabels(const TriMesh& m, const std::vector<int>& label, TriMesh::FaceHandle f, std::vector<int>& ring)
{
    ring.clear();
    TriMesh::VertexHandle v[3];
    int k = 0;
    for (auto fv : m.fv_range(f)) v[k++] = fv;

    // fan of vertex u: faces after f ... before f going ccw around u
    auto fan = [&](TriMesh::VertexHandle u, std::vector<int>& out) {
        std::vector<int> c;
        int pos = -1;
        for (auto g : m.vf_ccw_range(u)) {
            if (g == f) pos = int(c.size());
            c.push_back(label[g.idx()]);
        }
        if (m.is_boundary(u)) c.push_back(kGap);
        const int n = int(c.size());
        for (int j = 1; j < n; ++j) out.push_back(c[(pos + j) % n]);
    };

    std::vector<int> a, b, c;
    fan(v[1], a);
    fan(v[2], b);
    fan(v[0], c);
    // consecutive fans share the face across the common edge: keep it once
    ring = a;
    ring.insert(ring.end(), b.begin() + (b.empty() ? 0 : 1), b.end());
    ring.insert(ring.end(), c.begin() + (c.empty() ? 0 : 1), c.end());
    if (!ring.empty()) ring.pop_back();   // the last element of the last fan is the first of the first
}

int cyclicRuns(const std::vector<int>& ring, int value)
{
    const int n = int(ring.size());
    int runs = 0;
    for (int i = 0; i < n; ++i)
        if (ring[i] == value && ring[(i + n - 1) % n] != value) ++runs;
    return runs;
}

constexpr size_t kChunk = 1 << 15;

ThreadPool& poolOrShared(ThreadPool* pool) { return pool ? *pool : ThreadPool::shared(); }

// Fast path: a bitmask of the colors used around every vertex (up to 256 colors, i.e. vertex valence 256).
// Returns null when the colors run out.
std::shared_ptr<const FaceColoring> buildColoringMasks(const TriMesh& m)
{
    struct Mask { uint64_t w[4] = {0, 0, 0, 0}; };
    std::vector<Mask> used(m.n_vertices());
    auto out = std::make_shared<FaceColoring>();
    for (size_t f = 0; f < m.n_faces(); ++f) {
        int v[3], n = 0;
        for (auto fv : m.fv_range(TriMesh::FaceHandle(int(f)))) v[n++] = fv.idx();
        int c = -1;
        for (int w = 0; w < 4 && c < 0; ++w) {
            const uint64_t taken = used[v[0]].w[w] | used[v[1]].w[w] | used[v[2]].w[w];
            if (taken != ~uint64_t(0)) c = w * 64 + __builtin_ctzll(~taken);
        }
        if (c < 0) return nullptr;
        for (int i = 0; i < 3; ++i) used[v[i]].w[c >> 6] |= uint64_t(1) << (c & 63);
        if (size_t(c) >= out->byColor.size()) out->byColor.resize(size_t(c) + 1);
        out->byColor[c].push_back(int(f));
    }
    return out;
}

std::shared_ptr<const FaceColoring> buildColoring(const TriMesh& m)
{
    if (auto fast = buildColoringMasks(m)) return fast;
    const size_t nF = m.n_faces();
    std::vector<uint16_t> color(nF, 0xffff);
    std::vector<size_t> usedBy;   // usedBy[c] == f + 1: color c is taken around face f
    auto out = std::make_shared<FaceColoring>();
    for (size_t f = 0; f < nF; ++f) {
        for (auto v : m.fv_range(TriMesh::FaceHandle(int(f))))
            for (auto g : m.vf_range(v)) {
                const uint16_t c = color[g.idx()];
                if (c == 0xffff) continue;
                if (c >= usedBy.size()) usedBy.resize(size_t(c) + 1, 0);
                usedBy[c] = f + 1;
            }
        size_t c = 0;
        while (c < usedBy.size() && usedBy[c] == f + 1) ++c;
        color[f] = uint16_t(c);
        if (c >= out->byColor.size()) out->byColor.resize(c + 1);
        out->byColor[c].push_back(int(f));
    }
    return out;
}

} // namespace

double weightedPerimeter(const TriMesh& mesh, const std::vector<int>& label, const std::vector<double>& w,
                         ThreadPool* pool)
{
    const size_t nE = mesh.n_edges();
    std::vector<double> part((nE + kChunk - 1) / kChunk + 1, 0.0);
    forChunks(poolOrShared(pool), nE, kChunk, [&](size_t c, size_t b, size_t e) {
        double p = 0.0;
        for (size_t i = b; i < e; ++i) {
            const TriMesh::EdgeHandle eh{int(i)};
            if (mesh.is_boundary(eh)) continue;
            const auto h = mesh.halfedge_handle(eh, 0);
            if (label[mesh.face_handle(h).idx()] != label[mesh.face_handle(mesh.opposite_halfedge_handle(h)).idx()])
                p += w[i];
        }
        part[c] = p;
    });
    double p = 0.0;
    for (double x : part) p += x;   // chunk order: independent of the thread count
    return p;
}

std::vector<double> computeEdgeWeights(const VSAInput& in, double kappa, ThreadPool* pool)
{
    const TriMesh& m = in.mesh;
    std::vector<double> w(m.n_edges(), 0.0);
    forChunks(poolOrShared(pool), w.size(), kChunk, [&](size_t, size_t b, size_t en) {
        for (size_t i = b; i < en; ++i) {
            const TriMesh::EdgeHandle e{int(i)};
            double len = m.calc_edge_length(e);
            if (kappa > 0.0 && !m.is_boundary(e)) {
                const auto h = m.halfedge_handle(e, 0);
                const Vec3& n1 = in.faces.normal[m.face_handle(h).idx()];
                const Vec3& n2 = in.faces.normal[m.face_handle(m.opposite_halfedge_handle(h)).idx()];
                const double ang = std::acos(std::max(-1.0, std::min(1.0, n1 | n2)));   // 0 for a degenerate face
                len *= std::exp(-kappa * ang);
            }
            w[i] = len;
        }
    });
    return w;
}

SmoothStats smoothLabels(const VSAInput& in, VSAState& s)
{
    SmoothStats st;
    const TriMesh& m = in.mesh;
    if (s.smooth.lambda <= 0.0 || s.label.size() != m.n_faces() || s.k() < 2) return st;
    ThreadPool& pool = poolOf(s);

    recomputeRegions(in, s);
    const std::vector<double> w = computeEdgeWeights(in, s.smooth.kappa, &pool);
    const double eRef = totalEnergy(s);
    st.perimeterBefore = st.perimeterAfter = weightedPerimeter(m, s.label, w, &pool);
    if (eRef <= 1e-13 || st.perimeterBefore <= 0.0) return st;   // nothing to trade
    const double lambda = s.smooth.lambda * eRef / st.perimeterBefore;

    size_t colored = 0;
    if (in.coloring)
        for (const auto& c : in.coloring->byColor) colored += c.size();
    if (colored != m.n_faces()) in.coloring = buildColoring(m);

    std::vector<int> size(s.k(), 0);
    for (int l : s.label) if (l >= 0) ++size[l];

    struct Flip { int face, to; };
    // A decision only depends on the labels around the face, so a face is re-examined only after one of
    // them changed ("dirty"); this is exact, a sweep without flips still means a fixed point.
    std::vector<uint8_t> dirty(m.n_faces(), 1);
    for (int sweep = 0; sweep < s.smooth.maxSweeps; ++sweep) {
        int flips = 0;
        // Faces of one color share no vertex, so a decision (which reads the faces around its own face)
        // never sees another decision of the same color: they are made concurrently, then applied in
        // face order. The result depends neither on the thread count nor on scheduling.
        for (const std::vector<int>& faces : in.coloring->byColor) {
            const size_t chunks = (faces.size() + 4095) / 4096;
            std::vector<std::vector<Flip>> found(chunks);
            forChunks(pool, faces.size(), 4096, [&](size_t c, size_t b, size_t e) {
                std::vector<int> ring;
                for (size_t i = b; i < e; ++i) {
                    const TriMesh::FaceHandle f(faces[i]);
                    if (!dirty[f.idx()]) continue;
                    dirty[f.idx()] = 0;
                    const int a = s.label[f.idx()];
                    if (a < 0) continue;

                    int nbLabel[3], nbCount = 0;
                    double nbW[3];
                    bool differs = false;
                    for (auto h : m.fh_range(f)) {
                        const auto o = m.opposite_halfedge_handle(h);
                        if (m.is_boundary(o)) continue;
                        nbLabel[nbCount] = s.label[m.face_handle(o).idx()];
                        nbW[nbCount] = w[m.edge_handle(h).idx()];
                        differs = differs || (nbLabel[nbCount] != a && nbLabel[nbCount] >= 0);
                        ++nbCount;
                    }
                    if (!differs) continue;   // no neighboring region to move to
                    auto cost = [&](int l) {
                        double err = metricError(s.metric, in.moments[f.idx()], s.proxies[l]);
                        for (int j = 0; j < nbCount; ++j)
                            if (nbLabel[j] != l) err += lambda * nbW[j];
                        return err;
                    };

                    int best = a;
                    double bestCost = cost(a);
                    for (int j = 0; j < nbCount; ++j) {
                        const int l = nbLabel[j];
                        if (l == a || l < 0) continue;
                        const double cc = cost(l);
                        if (cc < bestCost - 1e-12 * std::abs(bestCost)) { bestCost = cc; best = l; }
                    }
                    if (best == a) continue;

                    // topology guard: at most one run of `a` and exactly one run of `best` in the ring around
                    // the face. Zero runs of `a` means the face is an isolated island of its region: removing
                    // it only drops a stray component.
                    ringLabels(m, s.label, f, ring);
                    if (cyclicRuns(ring, a) > 1 || cyclicRuns(ring, best) != 1) continue;
                    found[c].push_back({f.idx(), best});
                }
            });
            for (const auto& list : found)
                for (const Flip& fl : list) {
                    const int a = s.label[fl.face];
                    if (size[a] <= 1) continue;   // never empty a region
                    s.label[fl.face] = fl.to;
                    --size[a];
                    ++size[fl.to];
                    ++flips;
                    for (auto v : m.fv_range(TriMesh::FaceHandle(fl.face)))
                        for (auto g : m.vf_range(v)) dirty[g.idx()] = 1;
                }
        }
        st.flips += flips;
        ++st.sweeps;
        if (flips == 0) break;
    }
    recomputeRegions(in, s);
    st.perimeterAfter = weightedPerimeter(m, s.label, w, &pool);
    return st;
}
