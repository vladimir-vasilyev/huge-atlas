#include "core/Metrics.h"

#include <algorithm>

Moments& Moments::operator+=(const Moments& o)
{
    area += o.area; first += o.first; second += o.second; areaNormal += o.areaNormal;
    return *this;
}

Moments& Moments::operator-=(const Moments& o)
{
    area -= o.area; first -= o.first; second -= o.second; areaNormal -= o.areaNormal;
    return *this;
}

Moments& Moments::operator*=(double w)
{
    area *= w; first *= w; second *= w; areaNormal *= w;
    return *this;
}

Moments triangleMoments(const EVec3& a, const EVec3& b, const EVec3& c, const EVec3& n)
{
    Moments m;
    m.area = 0.5 * (b - a).cross(c - a).norm();
    m.first = m.area * (a + b + c) / 3.0;
    const EVec3 s = a + b + c;
    m.second = m.area / 12.0 * (a * a.transpose() + b * b.transpose() + c * c.transpose() + s * s.transpose());
    m.areaNormal = m.area * n;
    return m;
}

std::vector<Moments> computeFaceMoments(const TriMesh& mesh, const FaceData& fd)
{
    std::vector<Moments> out(mesh.n_faces());
    for (auto f : mesh.faces()) {
        auto it = mesh.cfv_iter(f);
        const Vec3 a = mesh.point(*it); ++it;
        const Vec3 b = mesh.point(*it); ++it;
        const Vec3 c = mesh.point(*it);
        const Vec3& n = fd.normal[f.idx()];
        out[f.idx()] = triangleMoments(EVec3(a[0], a[1], a[2]), EVec3(b[0], b[1], b[2]),
                                       EVec3(c[0], c[1], c[2]), EVec3(n[0], n[1], n[2]));
    }
    return out;
}

double metricError(Metric metric, const Moments& m, const Proxy& p)
{
    double e = 0.0;
    if (metric == Metric::L21) {
        e = 2.0 * (m.area - p.N.dot(m.areaNormal));
    } else {
        // N^T (S - f X^T - X f^T + A X X^T) N,  S = second, f = first
        const double nf = p.N.dot(m.first);
        const double nx = p.N.dot(p.X);
        e = p.N.dot(m.second * p.N) - 2.0 * nf * nx + m.area * nx * nx;
    }
    return std::max(e, 0.0);   // cancellation can give tiny negatives
}

const char* metricName(Metric metric)
{
    return metric == Metric::L2 ? "L2" : "L2,1";
}
