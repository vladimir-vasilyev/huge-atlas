#pragma once

#include "core/MeshTypes.h"

#include <vector>

// How ragged the borders between regions are. "Ragged" borders (saw teeth) give boundary vertices
// with extreme angles, which are the first candidates for flipped triangles / overlaps in a flattening.
struct BoundaryStats {
    double perimeter = 0.0;        // total length of edges whose two faces have different labels
    int    boundaryEdges = 0;
    int    teeth = 0;              // faces with at least two edge neighbours of another label
    int    spikeVertices = 0;      // (vertex, region) pairs with an angle sum below minAngle
    int    reflexVertices = 0;     // (vertex, region) pairs with an angle sum above maxAngle
};

struct BoundaryStatsParams {
    double minAngle = 0.4 * 3.14159265358979323846;   // 72 degrees: one equilateral tooth tip is 60
    double maxAngle = 1.6 * 3.14159265358979323846;   // 288 degrees
};

// label: per-face region label (-1 = unassigned, treated as its own region). Vertices on the mesh
// border are skipped for the angle counts.
BoundaryStats computeBoundaryStats(const TriMesh& mesh, const std::vector<int>& label,
                                   const BoundaryStatsParams& p = {});
