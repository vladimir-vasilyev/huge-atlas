#include "core/Smoothing.h"

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

} // namespace

double weightedPerimeter(const TriMesh& mesh, const std::vector<int>& label, const std::vector<double>& w)
{
    double p = 0.0;
    for (auto e : mesh.edges()) {
        if (mesh.is_boundary(e)) continue;
        const auto h = mesh.halfedge_handle(e, 0);
        if (label[mesh.face_handle(h).idx()] != label[mesh.face_handle(mesh.opposite_halfedge_handle(h)).idx()])
            p += w[e.idx()];
    }
    return p;
}

std::vector<double> computeEdgeWeights(const VSAInput& in, double kappa)
{
    const TriMesh& m = in.mesh;
    std::vector<double> w(m.n_edges(), 0.0);
    for (auto e : m.edges()) {
        double len = m.calc_edge_length(e);
        if (kappa > 0.0 && !m.is_boundary(e)) {
            const auto h = m.halfedge_handle(e, 0);
            const Vec3& n1 = in.faces.normal[m.face_handle(h).idx()];
            const Vec3& n2 = in.faces.normal[m.face_handle(m.opposite_halfedge_handle(h)).idx()];
            const double ang = std::acos(std::max(-1.0, std::min(1.0, n1 | n2)));   // 0 for a degenerate face
            len *= std::exp(-kappa * ang);
        }
        w[e.idx()] = len;
    }
    return w;
}

SmoothStats smoothLabels(const VSAInput& in, VSAState& s)
{
    SmoothStats st;
    const TriMesh& m = in.mesh;
    if (s.smooth.lambda <= 0.0 || s.label.size() != m.n_faces() || s.k() < 2) return st;

    recomputeRegions(in, s);
    const std::vector<double> w = computeEdgeWeights(in, s.smooth.kappa);
    const double eRef = totalEnergy(s);
    st.perimeterBefore = st.perimeterAfter = weightedPerimeter(m, s.label, w);
    if (eRef <= 1e-13 || st.perimeterBefore <= 0.0) return st;   // nothing to trade
    const double lambda = s.smooth.lambda * eRef / st.perimeterBefore;

    std::vector<int> size(s.k(), 0);
    for (int l : s.label) if (l >= 0) ++size[l];

    std::vector<int> ring;
    for (int sweep = 0; sweep < s.smooth.maxSweeps; ++sweep) {
        int flips = 0;
        for (auto f : m.faces()) {
            const int a = s.label[f.idx()];
            if (a < 0) continue;

            int nbLabel[3], nbCount = 0;
            double nbW[3];
            for (auto h : m.fh_range(f)) {
                const auto o = m.opposite_halfedge_handle(h);
                if (m.is_boundary(o)) continue;
                nbLabel[nbCount] = s.label[m.face_handle(o).idx()];
                nbW[nbCount] = w[m.edge_handle(h).idx()];
                ++nbCount;
            }
            auto cost = [&](int c) {
                double e = metricError(s.metric, in.moments[f.idx()], s.proxies[c]);
                for (int i = 0; i < nbCount; ++i)
                    if (nbLabel[i] != c) e += lambda * nbW[i];
                return e;
            };

            int best = a;
            double bestCost = cost(a);
            for (int i = 0; i < nbCount; ++i) {
                const int c = nbLabel[i];
                if (c == a || c < 0) continue;
                const double cc = cost(c);
                if (cc < bestCost - 1e-12 * std::abs(bestCost)) { bestCost = cc; best = c; }
            }
            if (best == a || size[a] <= 1) continue;

            // topology guard: at most one run of `a` and exactly one run of `best` in the ring around
            // the face. Zero runs of `a` means the face is an isolated island of its region: removing
            // it only drops a stray component.
            ringLabels(m, s.label, f, ring);
            if (cyclicRuns(ring, a) > 1 || cyclicRuns(ring, best) != 1) continue;

            s.label[f.idx()] = best;
            --size[a];
            ++size[best];
            ++flips;
        }
        st.flips += flips;
        ++st.sweeps;
        if (flips == 0) break;
    }
    recomputeRegions(in, s);
    st.perimeterAfter = weightedPerimeter(m, s.label, w);
    return st;
}
