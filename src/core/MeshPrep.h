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

// Closes the small boundary loops of a mesh (holes of at most `maxEdges` edges): a triangle for a loop of three edges, a fan
// around a new vertex at the centre of the loop for a longer one. The simplified meshes of the huge pipeline have such holes
// where the half-edge mesh refused a face; a hole makes a patch a ring that has to be cut open, and a cut patch rarely
// unfolds without overlapping. Returns the number of loops closed (loops that cannot be closed are left as they are).
int fillSmallHoles(TriMesh& mesh, size_t maxEdges);
