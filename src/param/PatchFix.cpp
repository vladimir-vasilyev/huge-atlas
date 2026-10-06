#include "param/PatchFix.h"

#include "param/LinABF.h"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

std::vector<Eigen::Vector2d> tutteEmbedding(const TriMesh& m, std::string* error)
{
    auto fail = [&](const char* msg) { if (error) *error = msg; return std::vector<Eigen::Vector2d>(); };
    const PatchTopology topo = analyzeTopology(m);
    if (topo.kind != PatchKind::Disk) return fail("Tutte needs a disk patch");

    const std::vector<int>& loop = topo.loops[0];
    const int nv = int(m.n_vertices());
    std::vector<Eigen::Vector2d> uv(nv, Eigen::Vector2d::Zero());
    std::vector<char> isBoundary(nv, 0);

    // boundary on the circle; the halfedge loop runs clockwise around the face interior, so go backwards
    std::vector<double> cum(loop.size() + 1, 0.0);
    for (size_t i = 0; i < loop.size(); ++i)
        cum[i + 1] = cum[i] + (m.point(TriMesh::VertexHandle(loop[(i + 1) % loop.size()])) -
                               m.point(TriMesh::VertexHandle(loop[i]))).norm();
    for (size_t i = 0; i < loop.size(); ++i) {
        const double t = -2.0 * kPi * cum[i] / cum.back();
        uv[loop[i]] = Eigen::Vector2d(std::cos(t), std::sin(t));
        isBoundary[loop[i]] = 1;
    }

    std::vector<int> index(nv, -1);
    int n = 0;
    for (int v = 0; v < nv; ++v) if (!isBoundary[v]) index[v] = n++;
    if (n > 0) {
        std::vector<Eigen::Triplet<double>> trip;
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n, 2);
        for (auto v : m.vertices()) {
            if (isBoundary[v.idx()]) continue;
            // mean value weights: w_ij = (tan(d_left / 2) + tan(d_right / 2)) / |v_i v_j|
            std::vector<std::pair<int, double>> w;
            for (auto h : m.voh_range(v)) {
                const auto f = m.face_handle(h);
                if (!f.is_valid()) continue;
                // face (v, j, k): the angle at v contributes to both edges v-j and v-k
                const auto hn = m.next_halfedge_handle(h);
                const int j = m.to_vertex_handle(h).idx(), k = m.to_vertex_handle(hn).idx();
                const Vec3 pv = m.point(v), pj = m.point(TriMesh::VertexHandle(j)), pk = m.point(TriMesh::VertexHandle(k));
                const Vec3 a = pj - pv, b = pk - pv;
                const double delta = std::atan2((a % b).norm(), a | b);
                w.push_back({j, std::tan(0.5 * delta) / a.norm()});
                w.push_back({k, std::tan(0.5 * delta) / b.norm()});
            }
            double diag = 0.0;
            const int row = index[v.idx()];
            for (const auto& [j, wij] : w) {
                diag += wij;
                if (isBoundary[j]) rhs.row(row) += wij * uv[j].transpose();
                else trip.emplace_back(row, index[j], -wij);
            }
            trip.emplace_back(row, row, diag);
        }
        Eigen::SparseMatrix<double> L(n, n);
        L.setFromTriplets(trip.begin(), trip.end());
        Eigen::SparseLU<Eigen::SparseMatrix<double>> lu(L);   // not symmetric: weights are per-row
        if (lu.info() != Eigen::Success) return fail("Tutte: factorization failed");
        const Eigen::MatrixXd x = lu.solve(rhs);
        if (lu.info() != Eigen::Success || !x.allFinite()) return fail("Tutte: solve failed");
        for (int v = 0; v < nv; ++v)
            if (index[v] >= 0) uv[v] = Eigen::Vector2d(x(index[v], 0), x(index[v], 1));
    }
    return uv;
}

bool cutAtWorstVertex(const TriMesh& source, const Patch& p, double kappa, Patch& out, std::string* error)
{
    auto fail = [&](const char* msg) { if (error) *error = msg; return false; };
    const TriMesh& m = p.mesh;
    const std::vector<double> ang = originalAngles(m);

    int worst = -1;
    double worstDefect = -1.0;
    for (auto v : m.vertices()) {
        if (m.is_boundary(v) || m.is_isolated(v)) continue;
        double sum = 0.0;
        for (auto h : m.vih_range(v)) sum += ang[cornerIndex(m, h)];
        const double defect = std::abs(2.0 * kPi - sum);
        if (defect > worstDefect) { worstDefect = defect; worst = v.idx(); }
    }
    if (worst < 0) return fail("no interior vertex to cut from");

    // Dijkstra to the nearest boundary vertex; intermediate vertices are interior
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(m.n_vertices(), kInf);
    std::vector<int> prev(m.n_vertices(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    dist[worst] = 0.0;
    q.push({0.0, worst});
    int target = -1;
    while (!q.empty()) {
        const auto [d, u] = q.top();
        q.pop();
        if (d > dist[u]) continue;
        if (m.is_boundary(TriMesh::VertexHandle(u))) { target = u; break; }
        for (auto h : m.voh_range(TriMesh::VertexHandle(u))) {
            const int w = m.to_vertex_handle(h).idx();
            double cost = m.calc_edge_length(h);
            const auto f1 = m.face_handle(h), f2 = m.face_handle(m.opposite_halfedge_handle(h));
            if (f1.is_valid() && f2.is_valid()) {
                const Vec3 n1 = m.calc_face_normal(f1), n2 = m.calc_face_normal(f2);
                cost *= std::exp(-kappa * std::acos(std::max(-1.0, std::min(1.0, n1 | n2))));
            }
            if (d + cost < dist[w]) { dist[w] = d + cost; prev[w] = u; q.push({d + cost, w}); }
        }
    }
    if (target < 0) return fail("no path to the boundary");

    std::vector<int> cuts = p.cutEdges;
    for (int v = target; prev[v] >= 0; v = prev[v]) {
        const auto h = source.find_halfedge(TriMesh::VertexHandle(p.vertexToGlobal[v]),
                                            TriMesh::VertexHandle(p.vertexToGlobal[prev[v]]));
        if (!h.is_valid()) return fail("cut edge not found in the source mesh");
        cuts.push_back(source.edge_handle(h).idx());
    }
    Patch next;
    if (!buildPatch(source, p.faceToGlobal, cuts, next)) return fail("cut patch could not be built");
    next.region = p.region;
    next.kindBeforeCuts = p.kindBeforeCuts;
    next.cuts = p.cuts + 1;
    next.note = p.note;
    out = std::move(next);
    return true;
}
