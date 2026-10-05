#pragma once

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <vector>

using EVec3 = Eigen::Vector3d;
using EMat3 = Eigen::Matrix3d;

enum class Metric {
    L2,    // squared distance of surface points to the proxy plane
    L21    // area-weighted squared difference of normals
};

// A proxy: plane through X with unit normal N.
struct Proxy {
    EVec3 X = EVec3::Zero();
    EVec3 N = EVec3::UnitZ();
};

// Additive integrals over a set of triangles. One triangle gives its own Moments;
// a region is the sum of its triangles, so merge / split cost O(1).
//   area          = sum |T|
//   first         = sum  int_T x dA
//   second        = sum  int_T x x^T dA
//   areaNormal    = sum |T| n_T
struct Moments {
    double area = 0.0;
    EVec3  first = EVec3::Zero();
    EMat3  second = EMat3::Zero();
    EVec3  areaNormal = EVec3::Zero();

    Moments& operator+=(const Moments& o);
    Moments& operator-=(const Moments& o);
    friend Moments operator+(Moments a, const Moments& b) { return a += b; }
    friend Moments operator-(Moments a, const Moments& b) { return a -= b; }

    EVec3 centroid() const { return area > 0.0 ? EVec3(first / area) : EVec3(EVec3::Zero()); }
};

// Exact moments of a triangle (a, b, c); n is the unit normal (zero for a degenerate triangle).
//   int_T x x^T dA = |T|/12 * (sum v_k v_k^T + s s^T),  s = v1 + v2 + v3
Moments triangleMoments(const EVec3& a, const EVec3& b, const EVec3& c, const EVec3& n);

// Moments of every face, indexed by face.idx().
std::vector<Moments> computeFaceMoments(const TriMesh& mesh, const FaceData& fd);

// Error of a set of triangles with the given moments against a proxy (exact, O(1)):
//   L21: sum |T| ||n_T - N||^2 = 2 (area - N . areaNormal)
//   L2 : N^T ( int (x-X)(x-X)^T dA ) N
double metricError(Metric metric, const Moments& m, const Proxy& proxy);

const char* metricName(Metric metric);
