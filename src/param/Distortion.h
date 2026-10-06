#pragma once

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <vector>

// Distortion of a patch parameterization. For every triangle the Jacobian J of the map from the
// 3D triangle (in its own plane) to its UV triangle has singular values s1 >= s2; s2 < 0 marks a
// flipped triangle. The UV is first scaled uniformly so that its total area equals the 3D area, so
// the ideal (isometric) map has s1 = s2 = 1 everywhere.
//   L2 stretch (Sander et al. 2001)  sqrt((s1^2 + s2^2) / 2)
//   L-inf stretch                    s1
//   conformal (angle) distortion     s1 / s2        (1 = angle preserving)
//   area ratio                       s1 * s2        (1 = area preserving)
struct DistortionStats {
    int    triangles = 0;
    int    flipped = 0;
    double scale = 1.0;           // uniform UV scale that was applied
    double l2Mean = 0.0;          // area-weighted RMS of the L2 stretch (the global L2 stretch)
    double l2Max = 0.0;
    double linfMax = 0.0;
    double conformalMean = 0.0;   // area-weighted mean of s1/s2 over non-flipped triangles
    double conformalMax = 0.0;
    double areaRatioMin = 0.0, areaRatioMax = 0.0;
};

// perTriangleL2 (optional): the L2 stretch of every face, -1 for flipped or degenerate faces.
DistortionStats computeDistortion(const TriMesh& mesh, const std::vector<Eigen::Vector2d>& uv,
                                  std::vector<float>* perTriangleL2 = nullptr);
