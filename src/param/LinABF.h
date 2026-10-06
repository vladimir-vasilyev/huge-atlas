#pragma once

#include "core/MeshTypes.h"

#include <string>
#include <vector>

// Linear Angle Based Parameterization (Zayer, Levy, Seidel, SGP 2007).
//
// Unknowns are the corner angles of all triangles. The angle of triangle f at its k-th corner,
// where corner k is the vertex to_vertex(h_k) of the k-th halfedge of the face loop, has the
// index 3 * f + k (see cornerIndex).
//
// Constraints on the corrected angles alpha* = alpha + e:
//   triangle:  e_a + e_b + e_c = pi - (alpha_a + alpha_b + alpha_c)
//   vertex  :  sum e_i = 2 pi - sum alpha_i                       (interior vertices)
//   wheel   :  sum cot(A_i) e_A_i - cot(B_i) e_B_i
//              = sum log sin B_i - log sin A_i                    (interior vertices, linearized)
// For the face (v, w, u) taken counter-clockwise around v, A is the angle at w and B the angle at u.
// With e = D r, D = diag(alpha), the least-norm solution of C r = b, C = A D, comes from the normal
// equation (C C^T) x = b, r = C^T x.

// Index of the corner of the face of `h` at to_vertex(h); -1 if h has no face.
int cornerIndex(const TriMesh& mesh, TriMesh::HalfedgeHandle h);

// Angle of every corner measured on the 3D triangles.
std::vector<double> originalAngles(const TriMesh& mesh);

struct LinABFParams {
    double epsilon = 1e-3;            // initial angles are clamped to [epsilon, pi - epsilon]
    double deficitThreshold = 1.0;    // |2 pi - sum| of a fan above this: use the exponential-map fan
    int    maxIterations = 8;         // linearization steps: repeat while the wheel residual is above the
                                      // tolerance or the angles left (0, pi) (then clamp: active-set style)
    double wheelTolerance = 1e-8;     // stop when max |sum log sin A - log sin B| is below this
};

struct LinABFResult {
    bool        ok = false;
    std::string message;              // reason when !ok, or a note

    std::vector<double> angles;       // alpha*, by corner index
    std::vector<double> initial;      // initial estimation alpha of the last iteration, by corner index

    int triangles = 0, interiorVertices = 0, rows = 0;
    int iterations = 0;
    int fansRescaled = 0;             // interior fans replaced by the exponential-map estimation

    // residuals of the original nonlinear constraints on alpha*
    double triangleResidual = 0.0;    // max |sum - pi|
    double vertexResidual = 0.0;      // max |sum - 2 pi| over interior vertices
    double wheelResidual = 0.0;       // max |sum log sin A - log sin B| over interior vertices (NaN if !anglesValid)
    double minAngle = 0.0, maxAngle = 0.0;
    bool   anglesValid = false;       // all alpha* in (0, pi)
    double angularError = 0.0;        // ||alpha* - alpha_original||_2 / (3 nt), the measure of the paper's table
};

// The mesh must be a manifold patch; the angles are meaningful for a topological disk.
// A rank-deficient system (e.g. a closed mesh, where the constraints are dependent) is reported
// as !ok with a message.
LinABFResult solveLinABF(const TriMesh& mesh, const LinABFParams& params = {});

// Residuals of the nonlinear constraints for arbitrary angles (indexed like the result).
void angleResiduals(const TriMesh& mesh, const std::vector<double>& angles, double& triangle, double& vertex,
                    double& wheel);
