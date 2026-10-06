#pragma once

#include "core/MeshTypes.h"

#include <Eigen/Dense>

#include <utility>
#include <vector>

// A locally injective map of a disk with a simple boundary curve is globally injective. The checks
// below test exactly that, in three stages that match where a problem can be repaired:
//   1. angles, before any UV exists: every angle in (0, pi); the angles around a boundary vertex sum
//      to at most 2 pi - delta (otherwise the fan winds around the vertex more than once);
//   2. the UV map: the signed area of every triangle is positive;
//   3. the UV boundary: no two boundary edges intersect (exact predicates).

struct AngleReport {
    std::vector<int> badCorners;     // corner indices with an angle outside (0, pi)
    std::vector<int> foldVertices;   // boundary vertices whose fan angle sum exceeds 2 pi - delta
    std::vector<double> foldSums;    // the sums, parallel to foldVertices
    bool ok() const { return badCorners.empty() && foldVertices.empty(); }
};

struct UVReport {
    std::vector<int> flippedFaces;                   // faces with signed area <= 0
    std::vector<std::pair<int, int>> crossings;      // pairs of boundary halfedge indices that intersect
    bool ok() const { return flippedFaces.empty() && crossings.empty(); }
};

struct InjectivityReport {
    AngleReport angles;
    UVReport    uv;
    bool ok() const { return angles.ok() && uv.ok(); }
};

// angles are indexed 3 * f + k (see LinABF). delta in radians; a sum exactly at 2 pi - delta (within 1e-9) is accepted.
AngleReport checkAngles(const TriMesh& mesh, const std::vector<double>& angles, double foldDelta);

// uv per patch vertex.
UVReport checkUV(const TriMesh& mesh, const std::vector<Eigen::Vector2d>& uv);

// Intersections between the boundary edges of the UV map. Adjacent edges are only reported when they
// overlap along a line. Exposed separately for tests.
std::vector<std::pair<int, int>> boundaryCrossings(const TriMesh& mesh, const std::vector<Eigen::Vector2d>& uv);
