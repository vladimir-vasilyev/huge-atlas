#pragma once

#include "param/MeshSimplify.h"

#include <cstdint>
#include <vector>

class ThreadPool;

// Face neighbours of a flat mesh: nbr[3 f + k] is the face across the edge (idx[3 f + k], idx[3 f + (k + 1) % 3]),
// -1 on an open border and on edges shared by more than two faces. 12 bytes per face.
std::vector<int32_t> buildFaceNeighbors(const FlatMesh& mesh, ThreadPool* pool = nullptr);
