#pragma once

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

// UV coordinates from the corner angles of a patch (angles indexed like LinABF: 3 * f + k).
//
// Least squares of ABF++ (Sheffer et al. 2005): for a triangle with corners p1, p2, p3 and angles
// a1, a2, a3 the law of sines gives
//     p3 - p1 = (sin a2 / sin a3) * R(a1) * (p2 - p1)          (two equations per triangle)
// R is the counter-clockwise rotation, so the patch is flattened with its face orientation kept.
// The corners are cyclically renumbered per triangle so that the largest sine is the denominator.
// Two far-apart boundary vertices are pinned to remove the similarity freedom.

// Cyclic order (i1, i2, i3) of the three corners, in the face's own direction, with sin(a[i3]) the largest.
std::array<int, 3> uvCornerOrder(double a0, double a1, double a2);

struct UVResult {
    bool        ok = false;
    std::string message;
    std::vector<Eigen::Vector2d> uv;   // per patch vertex
    int    pinnedA = -1, pinnedB = -1;
    double rmsResidual = 0.0;          // RMS of the 2 nt equations
    double maxResidual = 0.0;
};

UVResult anglesToUV(const TriMesh& mesh, const std::vector<double>& angles);

// Debug reconstruction: depth-first over the triangles, placing the third vertex of each new triangle
// from the two already placed ones. Vertices reached again are not moved; the largest distance
// between the two placements is reported as `maxResidual` (the error accumulation the paper mentions).
UVResult anglesToUVGreedy(const TriMesh& mesh, const std::vector<double>& angles);

struct UVStats {
    int    triangles = 0;
    int    flipped = 0;              // signed area <= 0
    double minArea = 0.0;            // smallest signed area
    double totalArea = 0.0;          // sum of signed areas
    double maxAngleDeviation = 0.0;  // max |angle of the UV triangle - target angle|
    double rmsAngleDeviation = 0.0;
};

UVStats analyzeUV(const TriMesh& mesh, const std::vector<Eigen::Vector2d>& uv, const std::vector<double>& angles);

// Signed area of a UV triangle (positive = counter-clockwise).
double signedArea(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c);
