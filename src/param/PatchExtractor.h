#pragma once

#include "param/PatchTopology.h"

#include <string>
#include <vector>

// One region of the partition as an independent mesh. Vertices on region borders are duplicated
// (a seam belongs to both neighbours); a vertex where a region touches itself, or that lies on a
// cut, is split into one copy per fan of faces.
struct Patch {
    TriMesh          mesh;
    std::vector<int> faceToGlobal;     // local face index -> face index in the source mesh
    std::vector<int> vertexToGlobal;   // local vertex index -> vertex index in the source mesh
    int              region = -1;      // label after merging small regions and compaction
    PatchTopology    topology;         // of `mesh` (after the cuts)
    PatchKind        kindBeforeCuts = PatchKind::Other;
    int              cuts = 0;         // paths cut to turn a multi-boundary patch into a disk
    std::string      note;             // why the patch is not a disk (or what was done)

    bool isDisk() const { return topology.kind == PatchKind::Disk; }
};

struct PatchOptions {
    int  minFaces = 3;      // regions with fewer faces are merged into the neighbour sharing the longest border
    bool cutMultiBoundary = true;
};

struct ExtractionResult {
    std::vector<Patch> patches;
    std::vector<int>   faceToPatch;   // per source face
    int                mergedRegions = 0;
    int                disks = 0;     // patches that are disks in the end
};

// Merges regions with fewer than minFaces faces into a neighbouring region (the one sharing the
// longest border) and renumbers the labels 0..n-1. Isolated small regions are kept.
// Returns the number of merged regions.
int mergeSmallRegions(const TriMesh& mesh, std::vector<int>& label, int minFaces);

// Builds the patch of the given faces. Edges listed in `cutEdges` (indices in the source mesh)
// are treated as slits: faces on the two sides are not connected through them.
// Returns false (and sets note) if the mesh cannot be built.
bool buildPatch(const TriMesh& mesh, const std::vector<int>& faces, const std::vector<int>& cutEdges,
                Patch& out);

// Full pipeline: merge small regions, build every patch, cut multi-boundary patches into disks.
// `label` is the per-face region label (-1 faces are ignored).
ExtractionResult extractPatches(const TriMesh& mesh, std::vector<int> label, const PatchOptions& opt = {});
