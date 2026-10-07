#include "param/LiftParam.h"

#include "param/HierarchyMap.h"

#include <Eigen/Sparse>

#include <cmath>
#include <unordered_map>

namespace {

FlatMesh flatOf(const TriMesh& m)
{
    FlatMesh f = flatFromTriMesh(m);
    return f;
}

// Conformal (LSCM) solve for the vertices flagged in `unknown`; every other vertex keeps its value in `uv`.
// Returns false when the system could not be solved.
bool solveBand(const TriMesh& m, const std::vector<char>& unknown, std::vector<Eigen::Vector2d>& uv, double reg, int& bandFaces)
{
    std::vector<int> index(m.n_vertices(), -1);
    int n = 0;
    for (size_t v = 0; v < unknown.size(); ++v)
        if (unknown[v]) index[v] = n++;
    if (n == 0) return true;

    std::vector<Eigen::Triplet<double>> trip;
    std::vector<double> rhs;
    int rows = 0;
    bandFaces = 0;
    for (auto f : m.faces()) {
        int vid[3], k = 0;
        for (auto v : m.fv_range(f)) vid[k++] = v.idx();
        if (index[vid[0]] < 0 && index[vid[1]] < 0 && index[vid[2]] < 0) continue;
        ++bandFaces;
        // the triangle in its own plane: a at the origin, b on the x axis
        const Vec3 pa = m.point(TriMesh::VertexHandle(vid[0])), pb = m.point(TriMesh::VertexHandle(vid[1])),
                   pc = m.point(TriMesh::VertexHandle(vid[2]));
        const Vec3 e1 = pb - pa, e2 = pc - pa;
        const double xb = e1.norm();
        if (xb < 1e-300) continue;
        const Vec3 ex = e1 / xb;
        const double xc = e2 | ex;
        const double yc = (e2 - ex * xc).norm();
        const double d = xb * yc;
        if (d < 1e-300) continue;   // degenerate
        const double s = 1.0 / std::sqrt(d);
        // sum_j W_j z_j = 0 with z = u + i v
        const double wr[3] = {(xc - xb) * s, -xc * s, xb * s};
        const double wi[3] = {yc * s, -yc * s, 0.0};
        double bre = 0.0, bim = 0.0;
        for (int j = 0; j < 3; ++j) {
            if (index[vid[j]] >= 0) {
                const int c = 2 * index[vid[j]];
                trip.emplace_back(rows, c, wr[j]);       // real row: wr u - wi v
                trip.emplace_back(rows, c + 1, -wi[j]);
                trip.emplace_back(rows + 1, c, wi[j]);   // imaginary row: wi u + wr v
                trip.emplace_back(rows + 1, c + 1, wr[j]);
            } else {
                const Eigen::Vector2d& z = uv[vid[j]];
                bre -= wr[j] * z.x() - wi[j] * z.y();
                bim -= wi[j] * z.x() + wr[j] * z.y();
            }
        }
        rhs.push_back(bre);
        rhs.push_back(bim);
        rows += 2;
    }
    // a pull towards the clamped positions keeps the system well posed (a component without a fixed neighbour)
    const double w = std::sqrt(reg);
    Eigen::VectorXd start(2 * n);
    for (size_t v = 0; v < unknown.size(); ++v)
        if (index[v] >= 0) { start[2 * index[v]] = uv[v].x(); start[2 * index[v] + 1] = uv[v].y(); }
    for (int c = 0; c < 2 * n; ++c) {
        trip.emplace_back(rows, c, w);
        rhs.push_back(w * start[c]);
        ++rows;
    }
    Eigen::SparseMatrix<double> A(rows, 2 * n);
    A.setFromTriplets(trip.begin(), trip.end());
    Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd>(rhs.data(), rows);
    Eigen::SparseMatrix<double> AtA = A.transpose() * A;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(AtA);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::VectorXd x = ldlt.solve(A.transpose() * b);
    if (ldlt.info() != Eigen::Success || !x.allFinite()) return false;
    for (size_t v = 0; v < unknown.size(); ++v)
        if (index[v] >= 0) uv[v] = Eigen::Vector2d(x[2 * index[v]], x[2 * index[v] + 1]);
    return true;
}

} // namespace

PatchParam liftPatch(const Patch& finePatch, const PatchParam& coarse, const LiftOptions& opt, LiftStats* stats)
{
    PatchParam out;
    out.method = ParamMethod::Lifted;
    out.patch = finePatch;
    const TriMesh& fm = finePatch.mesh;
    const TriMesh& cm = coarse.patch.mesh;
    const size_t nV = fm.n_vertices();

    const FlatMesh cflat = flatOf(cm);
    const TriangleGrid grid(cflat);
    out.uv.assign(nV, Eigen::Vector2d::Zero());
    std::vector<char> outside(nV, 0);
    int nOutside = 0;
    for (size_t v = 0; v < nV; ++v) {
        const Vec3 p = fm.point(TriMesh::VertexHandle(int(v)));
        const float pf[3] = {float(p[0]), float(p[1]), float(p[2])};
        const TriangleGrid::Hit h = grid.closest(pf);
        if (h.face == UINT32_MAX) { out.log = "lift: empty coarse chart\n"; out.needsSplit = true; return out; }
        const uint32_t* tri = &cflat.idx[3 * size_t(h.face)];
        out.uv[v] = h.u * coarse.uv[tri[0]] + h.v * coarse.uv[tri[1]] + h.w * coarse.uv[tri[2]];
        if (h.inside) continue;
        // clamped: is the closest point on the border of the chart?
        const double wgt[3] = {h.u, h.v, h.w};
        int zero = 0, onlyPositive = -1;
        for (int k = 0; k < 3; ++k)
            if (wgt[k] <= opt.outsideTolerance) ++zero; else onlyPositive = k;
        bool onBorder = false;
        if (zero == 2) {   // at a vertex
            onBorder = cm.is_boundary(TriMesh::VertexHandle(int(tri[onlyPositive])));
        } else if (zero == 1) {   // on the edge opposite the vertex with weight ~0
            int z = 0;
            for (int k = 0; k < 3; ++k) if (wgt[k] <= opt.outsideTolerance) z = k;
            const auto va = TriMesh::VertexHandle(int(tri[(z + 1) % 3])), vb = TriMesh::VertexHandle(int(tri[(z + 2) % 3]));
            const auto he = cm.find_halfedge(va, vb);
            onBorder = he.is_valid() && cm.is_boundary(cm.edge_handle(he));
        }
        if (onBorder) { outside[v] = 1; ++nOutside; }
    }

    LiftStats local;
    local.vertices = int(nV);
    local.outside = nOutside;
    if (nOutside > 0) {
        if (!solveBand(fm, outside, out.uv, opt.regularization, local.bandFaces)) {
            out.log = "lift: band solve failed\n";
            out.needsSplit = true;
            return out;
        }
    }
    out.report.uv = checkUV(fm, out.uv);
    // a few flipped triangles (the fine surface folds against the coarse one): solve the neighbourhood again,
    // growing it until the map is clean
    for (int ring : {1, 2, 4, 8}) {
        if (out.report.uv.ok()) break;
        std::vector<char> region(nV, 0);
        std::vector<int> frontier;
        for (const auto& pr : out.report.uv.crossings)
            for (int h : {pr.first, pr.second}) {
                const auto he = TriMesh::HalfedgeHandle(h);
                for (auto v : {fm.from_vertex_handle(he), fm.to_vertex_handle(he)})
                    if (!region[v.idx()]) { region[v.idx()] = 1; frontier.push_back(v.idx()); }
            }
        for (int f : out.report.uv.flippedFaces)
            for (auto v : fm.fv_range(TriMesh::FaceHandle(f)))
                if (!region[v.idx()]) { region[v.idx()] = 1; frontier.push_back(v.idx()); }
        for (int r = 0; r < ring; ++r) {
            std::vector<int> next;
            for (int v : frontier)
                for (auto w : fm.vv_range(TriMesh::VertexHandle(v)))
                    if (!region[w.idx()]) { region[w.idx()] = 1; next.push_back(w.idx()); }
            frontier.swap(next);
        }
        int faces = 0;
        std::vector<Eigen::Vector2d> trial = out.uv;
        if (!solveBand(fm, region, trial, opt.regularization, faces)) break;
        const UVReport rep = checkUV(fm, trial);
        if (rep.flippedFaces.size() + rep.crossings.size() < out.report.uv.flippedFaces.size() + out.report.uv.crossings.size()) {
            out.uv = std::move(trial);
            out.report.uv = rep;
            ++local.repairs;
        }
    }
    if (stats) *stats = local;

    out.injective = out.report.ok();
    out.distortion = computeDistortion(fm, out.uv, &out.perTriangleL2);
    double total = 0.0, bad = 0.0;
    for (auto f : fm.faces()) {
        const double a = fm.calc_face_area(f);
        total += a;
        const float l2 = out.perTriangleL2[f.idx()];
        if (l2 < 0.0f || (opt.param.maxStretch > 0.0 && l2 > opt.param.maxStretch)) bad += a;
    }
    out.badAreaFraction = total > 0.0 ? bad / total : 0.0;
    out.needsSplit = !out.injective || (opt.param.maxStretch > 0.0 && out.badAreaFraction > opt.param.maxBadAreaFraction);
    char buf[160];
    std::snprintf(buf, sizeof buf, "lift: %d of %d vertices solved again (%d faces), %zu flipped, %zu crossings, L2 mean %.3f\n",
                  nOutside, int(nV), local.bandFaces, out.report.uv.flippedFaces.size(), out.report.uv.crossings.size(),
                  out.distortion.l2Mean);
    out.log = buf;
    return out;
}
