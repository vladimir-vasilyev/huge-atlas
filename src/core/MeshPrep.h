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

// Smoothing of the face normal field (VSA paper, "Smoothing the normal field"): before the segmentation every face normal is
// replaced by the area-weighted mean of the normals of its edge neighbours, where a neighbour counts with
// exp(-(angle / sigma)^2), so a crease sharper than sigma is not blurred. A noisy scan gives a ragged normal field that makes
// ragged, small regions; the L2,1 metric reads these normals. Only fd.normal changes (positions, areas and the L2 moments don't).
struct NormalSmoothing {
    int    iterations = 0;       // 0 = off
    double sigmaDegrees = 25.0;  // angle at which a neighbour's weight falls to 1/e
};
void smoothFaceNormals(const TriMesh& mesh, FaceData& fd, const NormalSmoothing& opt);

MeshDiagnostics diagnose(const TriMesh& mesh, const FaceData& fd);

// Closes the small boundary loops of a mesh (holes of at most `maxEdges` edges): a triangle for a loop of three edges, a fan
// around a new vertex at the centre of the loop for a longer one. The simplified meshes of the huge pipeline have such holes
// where the half-edge mesh refused a face; a hole makes a patch a ring that has to be cut open, and a cut patch rarely
// unfolds without overlapping. Returns the number of loops closed (loops that cannot be closed are left as they are).
int fillSmallHoles(TriMesh& mesh, size_t maxEdges);
