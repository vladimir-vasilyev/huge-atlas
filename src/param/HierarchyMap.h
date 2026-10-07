#pragma once

#include "param/MeshSimplify.h"

#include <cstdint>
#include <vector>

class ThreadPool;

// Closest-triangle queries over (a subset of) the triangles of a flat mesh: a uniform grid whose cells list
// the triangles overlapping them. Queries are thread safe.
class TriangleGrid {
public:
    struct Hit {
        uint32_t face = UINT32_MAX;   // UINT32_MAX: the grid is empty
        float    u = 0, v = 0, w = 0; // barycentric coordinates of the closest point (u + v + w = 1, all >= 0)
        double   dist2 = 0;           // squared distance to it
        bool     inside = false;      // the point projects into the triangle (not clamped to an edge or vertex)
    };

    // `faces` null = all faces of `mesh`. The mesh must outlive the grid.
    explicit TriangleGrid(const FlatMesh& mesh, const std::vector<uint32_t>* faces = nullptr);

    Hit closest(const float p[3]) const;

private:
    const FlatMesh&       mesh_;
    float                 lo_[3], cell_;
    int                   dim_[3];
    std::vector<size_t>   start_;      // CSR over cells
    std::vector<uint32_t> items_;      // face ids
};

// Where every fine vertex lies on the coarse surface.
struct FineToCoarse {
    std::vector<uint32_t> face;   // closest coarse triangle
    std::vector<float>    dist;   // distance to it
};

// Closest coarse triangle of every fine vertex (parallel; null pool = the shared pool).
FineToCoarse mapVertices(const FlatMesh& fine, const FlatMesh& coarse, ThreadPool* pool = nullptr);

// Label of every fine face from the labels of the coarse faces: the majority of its three vertices' coarse
// faces; when all three differ, the vertex closest to the coarse surface decides.
std::vector<int> transferLabels(const FlatMesh& fine, const std::vector<int>& coarseLabel, const FineToCoarse& map,
                                ThreadPool* pool = nullptr);
