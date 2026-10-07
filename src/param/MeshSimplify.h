#pragma once

#include "core/MeshTypes.h"

#include <cstddef>
#include <cstdint>
#include <vector>

// Flat triangle mesh: the representation for scans too big for TriMesh (about 25 bytes per face).
struct FlatMesh {
    std::vector<float>    pos;   // x y z per vertex
    std::vector<uint32_t> idx;   // three vertex indices per face

    size_t vertexCount() const { return pos.size() / 3; }
    size_t faceCount() const { return idx.size() / 3; }
};

FlatMesh flatFromTriMesh(const TriMesh& mesh);

struct SimplifyOptions {
    size_t targetFaces = 100000;
    bool   lockBorder = false;       // keep the open border of the mesh where it is
    bool   prune = false;            // drop small disconnected parts (smaller than the simplification error)
    bool   regularize = false;       // prefer well shaped triangles (fewer slivers) at a small cost in accuracy
    // Meshes above `blockFaces` * 2 are cut into spatial blocks of about that many faces, simplified in
    // parallel with the block borders locked, then simplified once more as a whole (that removes the
    // frozen block seams). The result does not depend on the thread count.
    size_t blockFaces = 1000000;
    size_t intermediateFactor = 8;   // after the block stage about targetFaces * factor faces remain
    int    threads = 0;              // 0 = all cores
    const std::vector<uint8_t>* lockVertex = nullptr;   // optional: nonzero = this vertex must stay in place
};

struct SimplifyResult {
    FlatMesh              mesh;            // compact: only the vertices that are still used
    std::vector<uint32_t> vertexOrigin;    // coarse vertex -> vertex of the input mesh (positions are unchanged)
    float                 error = 0.f;     // largest relative error reported by the final pass (0..1 of the extent)
    size_t                blocks = 0;      // number of blocks of the first stage (0 = single pass)
    double                blockSeconds = 0.0, finalSeconds = 0.0;
};

// Edge-collapse simplification by quadric error (meshoptimizer). Survivors keep their original positions.
SimplifyResult simplifyMesh(const FlatMesh& in, const SimplifyOptions& opt = {});
