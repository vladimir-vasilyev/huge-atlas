#pragma once

#include "core/MeshTypes.h"

#include <vector>

enum class PatchKind {
    Disk,           // one component, one boundary loop, genus 0: LinABF applies
    MultiBoundary,  // genus 0 with several boundary loops (annulus, disk with holes): needs cuts
    Closed,         // no boundary
    Handles,        // boundary and genus > 0: must be split by VSA (stage 10)
    Other           // several components, empty, or inconsistent counts
};

const char* patchKindName(PatchKind kind);

struct PatchTopology {
    int  vertices = 0, edges = 0, faces = 0;
    int  euler = 0;                 // V - E + F
    int  components = 0;
    int  boundaryLoops = 0;
    int  genus = 0;                 // (2 - loops - euler) / 2 for a single component
    bool vertexManifold = true;     // every vertex has one fan
    PatchKind kind = PatchKind::Other;
    std::vector<std::vector<int>> loops;   // boundary loops as vertex indices, in halfedge order
};

PatchTopology analyzeTopology(const TriMesh& mesh);
