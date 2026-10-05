#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <vector>

// Меш с double-точностью: геометрия VSA и LinABF чувствительна к накоплению ошибок.
struct DoubleTraits : OpenMesh::DefaultTraits {
    using Point  = OpenMesh::Vec3d;
    using Normal = OpenMesh::Vec3d;
};

using TriMesh = OpenMesh::TriMesh_ArrayKernelT<DoubleTraits>;
using Vec3 = OpenMesh::Vec3d;

// Геометрия граней в массивах по face.idx(). Для вырожденных граней normal = 0.
struct FaceData {
    std::vector<double> area;
    std::vector<Vec3>   normal;
    std::vector<Vec3>   centroid;
    std::vector<char>   degenerate;
};
