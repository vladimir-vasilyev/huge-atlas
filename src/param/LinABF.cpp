#include "param/LinABF.h"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

using Triplets = std::vector<Eigen::Triplet<double>>;

double cornerAngle(const TriMesh& m, TriMesh::HalfedgeHandle h)
{
    const Vec3 p = m.point(m.to_vertex_handle(h));
    const Vec3 a = (m.point(m.from_vertex_handle(h)) - p);
    const Vec3 b = (m.point(m.to_vertex_handle(m.next_halfedge_handle(h))) - p);
    const double la = a.norm(), lb = b.norm();
    if (la == 0.0 || lb == 0.0) return 0.0;
    // atan2 of |cross| and dot is accurate for tiny and for nearly flat angles, unlike acos
    return std::atan2((a % b).norm(), a | b);
}

// Per interior vertex v: the (A, B) corner indices of every face around v and the corners at v.
struct Wheel {
    std::vector<int> atVertex, a, b;
};

Wheel wheelOf(const TriMesh& m, TriMesh::VertexHandle v)
{
    Wheel w;
    for (auto h : m.vih_range(v)) {   // incoming halfedge u -> v of the face (v, w, u)
        if (!m.face_handle(h).is_valid()) continue;
        w.atVertex.push_back(cornerIndex(m, h));
        w.a.push_back(cornerIndex(m, m.next_halfedge_handle(h)));   // angle at w
        w.b.push_back(cornerIndex(m, m.prev_halfedge_handle(h)));   // angle at u
    }
    return w;
}

} // namespace

int cornerIndex(const TriMesh& m, TriMesh::HalfedgeHandle h)
{
    const auto f = m.face_handle(h);
    if (!f.is_valid()) return -1;
    const auto h0 = m.halfedge_handle(f);
    const int k = (h == h0) ? 0 : (h == m.next_halfedge_handle(h0) ? 1 : 2);
    return 3 * f.idx() + k;
}

std::vector<double> originalAngles(const TriMesh& m)
{
    std::vector<double> a(m.n_faces() * 3, 0.0);
    for (auto f : m.faces())
        for (auto h : m.fh_range(f))
            a[cornerIndex(m, h)] = cornerAngle(m, h);
    return a;
}

void angleResiduals(const TriMesh& m, const std::vector<double>& ang, double& tri, double& vert, double& wheel)
{
    tri = vert = wheel = 0.0;
    for (auto f : m.faces())
        tri = std::max(tri, std::abs(ang[3 * f.idx()] + ang[3 * f.idx() + 1] + ang[3 * f.idx() + 2] - kPi));
    for (auto v : m.vertices()) {
        if (m.is_boundary(v) || m.is_isolated(v)) continue;
        const Wheel w = wheelOf(m, v);
        double sum = 0.0, logs = 0.0;
        for (size_t i = 0; i < w.a.size(); ++i) {
            sum += ang[w.atVertex[i]];
            logs += std::log(std::sin(ang[w.a[i]])) - std::log(std::sin(ang[w.b[i]]));
        }
        vert = std::max(vert, std::abs(sum - 2.0 * kPi));
        wheel = std::max(wheel, std::abs(logs));
    }
}

LinABFResult solveLinABF(const TriMesh& m, const LinABFParams& p, const LinABFConstraints* extra)
{
    LinABFResult res;
    const int nt = int(m.n_faces());
    res.triangles = nt;
    if (nt == 0) { res.message = "empty mesh"; return res; }

    std::vector<TriMesh::VertexHandle> interior;
    for (auto v : m.vertices())
        if (!m.is_boundary(v) && !m.is_isolated(v)) interior.push_back(v);
    const int ni = int(interior.size());
    res.interiorVertices = ni;
    const int nFixed = extra ? int(extra->fixedAngles.size()) : 0;
    const int nSums = extra ? int(extra->boundarySums.size()) : 0;
    res.rows = nt + 2 * ni + nFixed + nSums;
    const int cols = 3 * nt;
    if (res.rows > cols) { res.message = "more constraints than angles"; return res; }

    const std::vector<double> original = originalAngles(m);
    std::vector<Wheel> wheels;
    wheels.reserve(ni);
    for (auto v : interior) wheels.push_back(wheelOf(m, v));
    std::vector<Wheel> sumVertices;   // only atVertex is used
    for (int j = 0; j < nSums; ++j) sumVertices.push_back(wheelOf(m, TriMesh::VertexHandle(extra->boundarySums[j].first)));

    // ---- initial estimation (section 4.2): original angles, exponential-map fans where the deficit is large
    std::vector<double> alpha = original;
    for (int i = 0; i < ni; ++i) {
        double sum = 0.0;
        for (int c : wheels[i].atVertex) sum += alpha[c];
        if (std::abs(2.0 * kPi - sum) > p.deficitThreshold && sum > 0.0) {
            for (int c : wheels[i].atVertex) alpha[c] *= 2.0 * kPi / sum;
            ++res.fansRescaled;
        }
    }
    auto clampAngles = [&](std::vector<double>& a) {
        for (double& x : a) x = std::max(p.epsilon, std::min(kPi - p.epsilon, x));
    };
    clampAngles(alpha);

    std::vector<double> current = alpha;
    for (int it = 0; it < std::max(1, p.maxIterations); ++it) {
        res.iterations = it + 1;
        res.initial = alpha;

        // ---- A e = b
        Triplets t;
        Eigen::VectorXd b(res.rows);
        for (int f = 0; f < nt; ++f) {
            for (int k = 0; k < 3; ++k) t.emplace_back(f, 3 * f + k, 1.0);
            b[f] = kPi - (alpha[3 * f] + alpha[3 * f + 1] + alpha[3 * f + 2]);
        }
        for (int i = 0; i < ni; ++i) {
            const Wheel& w = wheels[i];
            const int rv = nt + i, rw = nt + ni + i;
            double sum = 0.0, rhs = 0.0;
            for (size_t j = 0; j < w.a.size(); ++j) {
                t.emplace_back(rv, w.atVertex[j], 1.0);
                sum += alpha[w.atVertex[j]];
                const double ca = alpha[w.a[j]], cb = alpha[w.b[j]];
                t.emplace_back(rw, w.a[j], std::cos(ca) / std::sin(ca));    // cot A
                t.emplace_back(rw, w.b[j], -std::cos(cb) / std::sin(cb));   // -cot B
                rhs += std::log(std::sin(cb)) - std::log(std::sin(ca));
            }
            b[rv] = 2.0 * kPi - sum;
            b[rw] = rhs;
        }
        const int rowFixed = nt + 2 * ni, rowSums = rowFixed + nFixed;
        for (int j = 0; j < nFixed; ++j) {
            const int c = extra->fixedAngles[j].first;
            t.emplace_back(rowFixed + j, c, 1.0);
            b[rowFixed + j] = extra->fixedAngles[j].second - alpha[c];
        }
        for (int j = 0; j < nSums; ++j) {
            double sum = 0.0;
            for (int c : sumVertices[j].atVertex) {
                t.emplace_back(rowSums + j, c, 1.0);
                sum += alpha[c];
            }
            b[rowSums + j] = extra->boundarySums[j].second - sum;
        }
        Eigen::SparseMatrix<double> A(res.rows, cols);
        A.setFromTriplets(t.begin(), t.end());

        // ---- C = A D, normal equation (C C^T) x = b
        Eigen::VectorXd d(cols);
        for (int c = 0; c < cols; ++c) d[c] = alpha[c];
        Eigen::SparseMatrix<double> C = A * d.asDiagonal();
        Eigen::SparseMatrix<double> M = C * C.transpose();

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(M);
        if (ldlt.info() != Eigen::Success) { res.message = "factorization failed: dependent constraints"; return res; }
        const Eigen::VectorXd diag = ldlt.vectorD();
        if (diag.minCoeff() <= 1e-12 * diag.maxCoeff()) {
            res.message = "rank-deficient system: dependent constraints (not a disk?)";
            return res;
        }
        const Eigen::VectorXd x = ldlt.solve(b);
        if (ldlt.info() != Eigen::Success || !x.allFinite()) { res.message = "solve failed"; return res; }
        const Eigen::VectorXd r = C.transpose() * x;
        if ((C * r - b).norm() > 1e-8 * std::max(1.0, b.norm())) {
            res.message = "linear system is numerically singular";
            return res;
        }

        for (int c = 0; c < cols; ++c) current[c] = alpha[c] + d[c] * r[c];

        bool inRange = true;
        for (double a : current) if (!(a > 0.0 && a < kPi)) { inRange = false; break; }
        if (inRange) {
            double tri, vert, wheel;
            angleResiduals(m, current, tri, vert, wheel);
            if (wheel <= p.wheelTolerance) break;
        }
        if (it + 1 >= std::max(1, p.maxIterations)) break;

        // next iteration: linearize around the current angles, pushed back into the valid range
        alpha = current;
        clampAngles(alpha);
    }

    res.angles = current;
    res.minAngle = *std::min_element(current.begin(), current.end());
    res.maxAngle = *std::max_element(current.begin(), current.end());
    res.anglesValid = res.minAngle > 0.0 && res.maxAngle < kPi;

    double err = 0.0;
    for (int c = 0; c < cols; ++c) err += (current[c] - original[c]) * (current[c] - original[c]);
    res.angularError = std::sqrt(err) / (3.0 * nt);

    angleResiduals(m, current, res.triangleResidual, res.vertexResidual, res.wheelResidual);
    if (!res.anglesValid) res.wheelResidual = std::nan("");   // log sin is undefined outside (0, pi)
    res.ok = true;
    if (!res.anglesValid) res.message = "angles left (0, pi): use the active set / split the patch";
    return res;
}

LinABFResult solveLinABFActiveSet(const TriMesh& m, const LinABFParams& p, const ActiveSetParams& a, ActiveSetInfo* info)
{
    ActiveSetInfo local;
    ActiveSetInfo& inf = info ? *info : local;
    inf = ActiveSetInfo();

    LinABFConstraints cons;
    std::vector<char> cornerFixed(3 * m.n_faces(), 0), vertexLimited(m.n_vertices(), 0);
    LinABFResult res = solveLinABF(m, p, &cons);
    for (int round = 0; res.ok; ++round) {
        bool added = false;
        for (size_t c = 0; c < res.angles.size(); ++c) {
            const double ang = res.angles[c];
            if (cornerFixed[c] || (ang >= a.angleMargin && ang <= kPi - a.angleMargin)) continue;
            cons.fixedAngles.push_back({int(c), std::max(a.angleMargin, std::min(kPi - a.angleMargin, ang))});
            cornerFixed[c] = 1;
            added = true;
        }
        for (auto v : m.vertices()) {
            if (!m.is_boundary(v) || vertexLimited[v.idx()]) continue;
            double sum = 0.0;
            for (auto h : m.vih_range(v)) if (cornerIndex(m, h) >= 0) sum += res.angles[cornerIndex(m, h)];
            if (sum <= 2.0 * kPi - a.foldDelta) continue;
            cons.boundarySums.push_back({v.idx(), 2.0 * kPi - a.foldDelta});
            vertexLimited[v.idx()] = 1;
            added = true;
        }
        if (!added) { inf.converged = true; break; }
        if (round >= a.maxRounds) break;

        LinABFResult next = solveLinABF(m, p, &cons);
        if (!next.ok) { res.message = "active set: " + next.message; break; }
        res = std::move(next);
        ++inf.rounds;
    }
    inf.fixedCorners = int(cons.fixedAngles.size());
    inf.constrainedVertices = int(cons.boundarySums.size());
    return res;
}
