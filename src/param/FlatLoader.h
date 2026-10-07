#pragma once

#include "param/MeshSimplify.h"

#include <string>

// Readers that fill a FlatMesh directly (no half-edge mesh), for scans of tens of millions of faces.
// PLY (ascii, binary little or big endian; extra vertex properties are skipped; polygons are fanned) and OBJ
// (v and f only, negative indices, polygons fanned). The file is memory mapped. Faces with an index outside the
// vertex list or with fewer than three corners are skipped and counted.
struct FlatLoadInfo {
    std::string format;           // "ply binary_little_endian", "obj", ...
    size_t skippedFaces = 0;
    double seconds = 0.0;
};

bool loadFlatMesh(const std::string& path, FlatMesh& out, FlatLoadInfo* info = nullptr, std::string* error = nullptr);

// The same normalization as normalizeToUnitBox: centre at the origin, longest side 1. Returns the transform
// (p_original = p / scale + centre).
struct FlatTransform {
    double centre[3] = {0, 0, 0};
    double scale = 1.0;
};
FlatTransform normalizeFlatToUnitBox(FlatMesh& mesh);
