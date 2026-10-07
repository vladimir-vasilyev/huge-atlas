#pragma once

#include "param/MeshSimplify.h"

#include "core/ObjLoader.h"

#include <cstdint>
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

// What a textured scan carries besides the geometry (filled when `surface` is given and the file has it).
struct FlatSurface {
    std::vector<float>   cornerUV;       // 6 floats per face (corner k = vertex idx[3 f + k]), NaN = none; empty: no texture coordinates
    std::vector<uint8_t> faceMaterial;   // per face: index into `materials`, 255 = none (OBJ with usemtl; empty otherwise)
    std::vector<ObjMaterial> materials;  // from the material library of an OBJ
    std::string          directory;      // directory of the file: the texture files are found there
    std::string          textureFile;    // PLY: from "comment TextureFile name"
    bool hasUV() const { return !cornerUV.empty(); }
};

bool loadFlatMesh(const std::string& path, FlatMesh& out, FlatLoadInfo* info = nullptr, std::string* error = nullptr,
                  FlatSurface* surface = nullptr);

// The same normalization as normalizeToUnitBox: centre at the origin, longest side 1. Returns the transform
// (p_original = p / scale + centre).
struct FlatTransform {
    double centre[3] = {0, 0, 0};
    double scale = 1.0;
};
FlatTransform normalizeFlatToUnitBox(FlatMesh& mesh);
