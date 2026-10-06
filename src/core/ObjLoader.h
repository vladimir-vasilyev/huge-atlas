#pragma once

#include "core/MeshTypes.h"

#include <string>
#include <vector>

// Fast Wavefront OBJ reader (memory buffer, hand written number parsing; about 5x faster than the
// stream based reader of OpenMesh on large scans). Besides the geometry it keeps what a viewer needs:
// the texture coordinates of every face corner and the material of every face.
// Polygons are triangulated as fans; faces that the half-edge mesh rejects (non-manifold) are skipped
// and counted. Negative (relative) indices are supported; vn, vp, groups and objects are ignored.

struct ObjMaterial {
    std::string name;
    std::string diffuseMap;      // map_Kd file name, relative to the material library; empty if none
};

struct ObjExtras {
    // 6 floats per mesh face: u, v of the corners in the order of mesh.fv_range(face); empty if the file has no vt
    std::vector<float> cornerUV;
    std::vector<int>   faceMaterial;      // per mesh face, index into `materials`, -1 if none
    std::vector<ObjMaterial> materials;   // from the material library, in file order
    std::string        directory;         // directory of the OBJ (the library and the maps are found there)
    bool               hasUV() const { return !cornerUV.empty(); }
    int                skippedFaces = 0;  // rejected by the mesh (non-manifold) or malformed
};

// false + message if the file cannot be read or has no faces.
bool loadObj(const std::string& path, TriMesh& mesh, ObjExtras* extras = nullptr, std::string* error = nullptr);

// Material library parser (newmtl / map_Kd), exposed for tests.
std::vector<ObjMaterial> parseMtl(const std::string& text);
