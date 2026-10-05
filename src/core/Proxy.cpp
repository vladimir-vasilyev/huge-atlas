#include "core/Proxy.h"

namespace {

constexpr double kTiny = 1e-12;

EVec3 l2Normal(const Moments& r, const EVec3& X)
{
    const EMat3 cov = r.second - r.area * X * X.transpose();
    Eigen::SelfAdjointEigenSolver<EMat3> es(cov);
    return es.eigenvectors().col(0);   // eigenvalues are sorted ascending
}

} // namespace

Proxy fitProxy(Metric metric, const Moments& r)
{
    Proxy p;
    if (r.area <= 0.0) return p;
    p.X = r.centroid();

    const double len = r.areaNormal.norm();
    if (metric == Metric::L21 && len > kTiny * r.area)
        p.N = r.areaNormal / len;
    else
        p.N = l2Normal(r, p.X);

    if (p.N.dot(r.areaNormal) < 0.0) p.N = -p.N;
    return p;
}
