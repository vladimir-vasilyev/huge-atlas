#pragma once

#include "core/MeshTypes.h"

#include <string>

struct MeshDiagnostics {
    int    vertices = 0, edges = 0, faces = 0;
    int    eulerCharacteristic = 0;
    int    components = 0;
    int    boundaryLoops = 0;
    int    degenerateFaces = 0;
    int    nonManifoldVertices = 0;
    int    isolatedVertices = 0;
    double totalArea = 0.0;
};

// Reads OBJ/OFF/PLY/STL. Polygons are triangulated. Returns false and sets error on failure.
bool loadMesh(const std::string& path, TriMesh& mesh, std::string* error = nullptr);

// Moves the bbox center to the origin and scales so that the longest bbox side is 1.
void normalizeToUnitBox(TriMesh& mesh);

FaceData computeFaceData(const TriMesh& mesh, double degenerateAreaEps = 1e-14);

MeshDiagnostics diagnose(const TriMesh& mesh, const FaceData& fd);
