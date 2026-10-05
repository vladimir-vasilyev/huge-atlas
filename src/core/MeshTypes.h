#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <vector>

// Double-precision mesh: VSA and LinABF geometry is sensitive to accumulated error.
struct DoubleTraits : OpenMesh::DefaultTraits {
    using Point  = OpenMesh::Vec3d;
    using Normal = OpenMesh::Vec3d;
};

using TriMesh = OpenMesh::TriMesh_ArrayKernelT<DoubleTraits>;
using Vec3 = OpenMesh::Vec3d;

// Per-face geometry in arrays indexed by face.idx(). Degenerate faces have normal = 0.
struct FaceData {
    std::vector<double> area;
    std::vector<Vec3>   normal;
    std::vector<Vec3>   centroid;
    std::vector<char>   degenerate;
};
