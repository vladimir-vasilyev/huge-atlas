#include "param/Distortion.h"

#include "param/LinABF.h"

#include <algorithm>
#include <cmath>

DistortionStats computeDistortion(const TriMesh& m, const std::vector<Eigen::Vector2d>& uv, std::vector<float>* perTri)
{
    DistortionStats st;
    st.triangles = int(m.n_faces());
    if (perTri) perTri->assign(m.n_faces(), -1.0f);

    // uniform scale: UV area -> 3D area
    double area3 = 0.0, areaUV = 0.0;
    for (auto f : m.faces()) {
        Vec3 p[3];
        Eigen::Vector2d q[3];
        for (auto h : m.fh_range(f)) {
            const int k = cornerIndex(m, h) % 3;
            p[k] = m.point(m.to_vertex_handle(h));
            q[k] = uv[m.to_vertex_handle(h).idx()];
        }
        area3 += 0.5 * ((p[1] - p[0]) % (p[2] - p[0])).norm();
        areaUV += 0.5 * std::abs((q[1] - q[0]).x() * (q[2] - q[0]).y() - (q[1] - q[0]).y() * (q[2] - q[0]).x());
    }
    st.scale = (areaUV > 0.0 && area3 > 0.0) ? std::sqrt(area3 / areaUV) : 1.0;

    double wSum = 0.0, l2Sq = 0.0, confSum = 0.0, confW = 0.0;
    bool first = true;
    for (auto f : m.faces()) {
        Vec3 p[3];
        Eigen::Vector2d q[3];
        for (auto h : m.fh_range(f)) {
            const int k = cornerIndex(m, h) % 3;
            p[k] = m.point(m.to_vertex_handle(h));
            q[k] = uv[m.to_vertex_handle(h).idx()] * st.scale;
        }
        const Vec3 e1 = p[1] - p[0], e2 = p[2] - p[0];
        const double l1 = e1.norm(), cross = (e1 % e2).norm();
        if (l1 <= 0.0 || cross <= 0.0) { ++st.flipped; continue; }   // degenerate 3D triangle
        const double x2 = (e1 | e2) / l1, y2 = cross / l1;
        const Eigen::Vector2d d1 = q[1] - q[0], d2 = q[2] - q[0];
        const Eigen::Vector2d c0 = d1 / l1, c1 = (-x2 * d1 + l1 * d2) / (l1 * y2);   // columns of J
        const double a = c0.x(), c = c0.y(), b = c1.x(), d = c1.y();

        const double E = 0.5 * (a + d), F = 0.5 * (a - d), G = 0.5 * (c + b), H = 0.5 * (c - b);
        const double Q = std::hypot(E, H), R = std::hypot(F, G);
        const double s1 = Q + R, s2 = Q - R;   // s2 < 0 for an orientation-reversing map
        const double area = 0.5 * cross;
        if (s2 <= 0.0) { ++st.flipped; continue; }

        const double l2 = std::sqrt(0.5 * (s1 * s1 + s2 * s2));
        if (perTri) (*perTri)[f.idx()] = float(l2);
        wSum += area;
        l2Sq += area * l2 * l2;
        confSum += area * (s1 / s2);
        confW += area;
        st.l2Max = std::max(st.l2Max, l2);
        st.linfMax = std::max(st.linfMax, s1);
        st.conformalMax = std::max(st.conformalMax, s1 / s2);
        const double ar = s1 * s2;
        st.areaRatioMin = first ? ar : std::min(st.areaRatioMin, ar);
        st.areaRatioMax = first ? ar : std::max(st.areaRatioMax, ar);
        first = false;
    }
    if (wSum > 0.0) st.l2Mean = std::sqrt(l2Sq / wSum);
    if (confW > 0.0) st.conformalMean = confSum / confW;
    return st;
}
