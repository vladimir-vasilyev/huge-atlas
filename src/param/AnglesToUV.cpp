#include "param/AnglesToUV.h"

#include "param/LinABF.h"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>

namespace {

struct Tri {
    int v[3];       // patch vertices of the corners 0, 1, 2 (corner k = to_vertex of the k-th halfedge)
    double a[3];    // target angles
};

std::vector<Tri> trianglesOf(const TriMesh& m, const std::vector<double>& angles)
{
    std::vector<Tri> t(m.n_faces());
    for (auto f : m.faces())
        for (auto h : m.fh_range(f)) {
            const int c = cornerIndex(m, h);
            t[f.idx()].v[c % 3] = m.to_vertex_handle(h).idx();
            t[f.idx()].a[c % 3] = angles[c];
        }
    return t;
}

// Two boundary vertices that are far apart: double sweep over the boundary vertices.
bool farBoundaryPair(const TriMesh& m, int& a, int& b)
{
    std::vector<int> bv;
    for (auto v : m.vertices()) if (m.is_boundary(v)) bv.push_back(v.idx());
    if (bv.size() < 2) return false;
    auto farthest = [&](int from) {
        int best = from;
        double bestD = -1.0;
        for (int v : bv) {
            const double d = (m.point(TriMesh::VertexHandle(v)) - m.point(TriMesh::VertexHandle(from))).norm();
            if (d > bestD) { bestD = d; best = v; }
        }
        return best;
    };
    b = farthest(bv[0]);
    a = farthest(b);
    return a != b;
}

} // namespace

std::array<int, 3> uvCornerOrder(double a0, double a1, double a2)
{
    const double s[3] = {std::sin(a0), std::sin(a1), std::sin(a2)};
    int big = 0;
    for (int k = 1; k < 3; ++k) if (s[k] > s[big]) big = k;
    return {(big + 1) % 3, (big + 2) % 3, big};   // p1, p2, p3 = corners after the largest, in face order
}

double signedArea(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    return 0.5 * ((b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x()));
}

UVResult anglesToUV(const TriMesh& m, const std::vector<double>& angles)
{
    UVResult res;
    const int nv = int(m.n_vertices()), nt = int(m.n_faces());
    if (nt == 0) { res.message = "empty mesh"; return res; }
    if (angles.size() != size_t(3 * nt)) { res.message = "angle count does not match the mesh"; return res; }
    if (!farBoundaryPair(m, res.pinnedA, res.pinnedB)) { res.message = "no boundary: nothing to pin"; return res; }

    const std::vector<Tri> tris = trianglesOf(m, angles);

    // pinned: A at the origin, B at distance |A B| (3D) along u
    std::vector<int> unknown(2 * nv, -1);   // index of the (u, v) variable in the reduced system
    Eigen::Vector2d fixedPos[2] = {
        {0.0, 0.0},
        {(m.point(TriMesh::VertexHandle(res.pinnedB)) - m.point(TriMesh::VertexHandle(res.pinnedA))).norm(), 0.0}};
    int count = 0;
    for (int v = 0; v < nv; ++v)
        if (v != res.pinnedA && v != res.pinnedB) { unknown[2 * v] = count++; unknown[2 * v + 1] = count++; }

    auto fixedValue = [&](int var) -> double {   // var = 2 * vertex + coordinate
        const int vert = var / 2, coord = var % 2;
        return vert == res.pinnedA ? fixedPos[0][coord] : fixedPos[1][coord];
    };

    std::vector<Eigen::Triplet<double>> trip;
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(2 * nt);
    // Row r of  sum coef * var = 0; pinned variables go to the right-hand side
    auto add = [&](int row, int var, double coef) {
        if (unknown[var] >= 0) trip.emplace_back(row, unknown[var], coef);
        else rhs[row] -= coef * fixedValue(var);
    };

    for (int f = 0; f < nt; ++f) {
        const Tri& t = tris[f];
        const std::array<int, 3> o = uvCornerOrder(t.a[0], t.a[1], t.a[2]);
        const int p1 = t.v[o[0]], p2 = t.v[o[1]], p3 = t.v[o[2]];
        const double a1 = t.a[o[0]], a2 = t.a[o[1]], a3 = t.a[o[2]];
        const double ratio = std::sin(a2) / std::sin(a3);
        const double a = ratio * std::cos(a1), b = ratio * std::sin(a1);

        // x:  u3 + (a-1) u1 - a u2 - b v1 + b v2 = 0
        add(2 * f, 2 * p3, 1.0);
        add(2 * f, 2 * p1, a - 1.0);
        add(2 * f, 2 * p2, -a);
        add(2 * f, 2 * p1 + 1, -b);
        add(2 * f, 2 * p2 + 1, b);
        // y:  v3 + (a-1) v1 - a v2 + b u1 - b u2 = 0
        add(2 * f + 1, 2 * p3 + 1, 1.0);
        add(2 * f + 1, 2 * p1 + 1, a - 1.0);
        add(2 * f + 1, 2 * p2 + 1, -a);
        add(2 * f + 1, 2 * p1, b);
        add(2 * f + 1, 2 * p2, -b);
    }

    Eigen::SparseMatrix<double> A(2 * nt, count);
    A.setFromTriplets(trip.begin(), trip.end());
    const Eigen::SparseMatrix<double> AtA = A.transpose() * A;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(AtA);
    if (ldlt.info() != Eigen::Success) { res.message = "factorization failed (disconnected patch?)"; return res; }
    const Eigen::VectorXd d = ldlt.vectorD();
    if (count > 0 && d.minCoeff() <= 1e-14 * d.maxCoeff()) {
        res.message = "rank-deficient system (disconnected patch?)";
        return res;
    }
    const Eigen::VectorXd x = ldlt.solve(A.transpose() * rhs);
    if (ldlt.info() != Eigen::Success || !x.allFinite()) { res.message = "solve failed"; return res; }

    res.uv.assign(nv, Eigen::Vector2d::Zero());
    for (int v = 0; v < nv; ++v) {
        if (v == res.pinnedA) res.uv[v] = fixedPos[0];
        else if (v == res.pinnedB) res.uv[v] = fixedPos[1];
        else res.uv[v] = Eigen::Vector2d(x[unknown[2 * v]], x[unknown[2 * v + 1]]);
    }

    const Eigen::VectorXd r = A * x - rhs;
    res.rmsResidual = std::sqrt(r.squaredNorm() / double(r.size()));
    res.maxResidual = r.cwiseAbs().maxCoeff();
    res.ok = true;
    return res;
}

UVResult anglesToUVGreedy(const TriMesh& m, const std::vector<double>& angles)
{
    UVResult res;
    const int nv = int(m.n_vertices()), nt = int(m.n_faces());
    if (nt == 0) { res.message = "empty mesh"; return res; }
    if (angles.size() != size_t(3 * nt)) { res.message = "angle count does not match the mesh"; return res; }

    const std::vector<Tri> tris = trianglesOf(m, angles);
    res.uv.assign(nv, Eigen::Vector2d::Zero());
    std::vector<char> placed(nv, 0), done(nt, 0);

    // Place the third vertex p3 from p1, p2: p3 = p1 + sin(a2)/sin(a3) R(a1) (p2 - p1)
    auto third = [&](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double a1, double a2, double a3) {
        const double ratio = std::sin(a2) / std::sin(a3);
        const Eigen::Vector2d e = p2 - p1;
        return Eigen::Vector2d(p1 + ratio * Eigen::Vector2d(std::cos(a1) * e.x() - std::sin(a1) * e.y(),
                                                             std::sin(a1) * e.x() + std::cos(a1) * e.y()));
    };

    for (int seed = 0; seed < nt; ++seed) {
        if (done[seed]) continue;
        {   // first triangle of a component: corner 0 at the origin, corner 1 on the u axis
            const Tri& t = tris[seed];
            const std::array<int, 3> o = uvCornerOrder(t.a[0], t.a[1], t.a[2]);
            const int p1 = t.v[o[0]], p2 = t.v[o[1]], p3 = t.v[o[2]];
            const double len = (m.point(TriMesh::VertexHandle(p2)) - m.point(TriMesh::VertexHandle(p1))).norm();
            if (!placed[p1]) { res.uv[p1] = {0.0, 0.0}; placed[p1] = 1; }
            if (!placed[p2]) { res.uv[p2] = {len, 0.0}; placed[p2] = 1; }
            if (!placed[p3]) { res.uv[p3] = third(res.uv[p1], res.uv[p2], t.a[o[0]], t.a[o[1]], t.a[o[2]]); placed[p3] = 1; }
            done[seed] = 1;
        }
        std::vector<int> stack = {seed};
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            for (auto nb : m.ff_range(TriMesh::FaceHandle(f))) {
                const int g = nb.idx();
                if (done[g]) continue;
                done[g] = 1;
                const Tri& t = tris[g];
                // a corner whose two neighbours (in face order) are both placed gives p1, p2 directly
                for (int k = 0; k < 3; ++k) {
                    const int i1 = (k + 1) % 3, i2 = (k + 2) % 3;   // corners after k, in face order
                    if (placed[t.v[i1]] && placed[t.v[i2]] && !placed[t.v[k]]) {
                        res.uv[t.v[k]] = third(res.uv[t.v[i1]], res.uv[t.v[i2]], t.a[i1], t.a[i2], t.a[k]);
                        placed[t.v[k]] = 1;
                        break;
                    }
                }
                stack.push_back(g);
            }
        }
    }
    // consistency: how far the closing triangles are from the placed positions
    for (int g = 0; g < nt; ++g) {
        const Tri& t = tris[g];
        for (int k = 0; k < 3; ++k) {
            const int i1 = (k + 1) % 3, i2 = (k + 2) % 3;
            const Eigen::Vector2d pred = third(res.uv[t.v[i1]], res.uv[t.v[i2]], t.a[i1], t.a[i2], t.a[k]);
            res.maxResidual = std::max(res.maxResidual, (pred - res.uv[t.v[k]]).norm());
        }
    }
    res.ok = true;
    return res;
}

UVStats analyzeUV(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv, const std::vector<double>& angles)
{
    UVStats s;
    s.triangles = int(m.n_faces());
    bool first = true;
    double sq = 0.0;
    for (auto f : m.faces()) {
        Eigen::Vector2d p[3];
        int c[3];
        for (auto h : m.fh_range(f)) {
            const int k = cornerIndex(m, h) % 3;
            p[k] = uv[m.to_vertex_handle(h).idx()];
            c[k] = cornerIndex(m, h);
        }
        const double area = signedArea(p[0], p[1], p[2]);
        if (area <= 0.0) ++s.flipped;
        if (first || area < s.minArea) s.minArea = area;
        first = false;
        s.totalArea += area;
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector2d a = p[(k + 1) % 3] - p[k], b = p[(k + 2) % 3] - p[k];
            const double ang = std::atan2(std::abs(a.x() * b.y() - a.y() * b.x()), a.dot(b));
            const double dev = std::abs(ang - angles[c[k]]);
            s.maxAngleDeviation = std::max(s.maxAngleDeviation, dev);
            sq += dev * dev;
        }
    }
    if (s.triangles > 0) s.rmsAngleDeviation = std::sqrt(sq / (3.0 * s.triangles));
    return s;
}
