#include "core/BoundaryMetrics.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

BoundaryStats computeBoundaryStats(const TriMesh& mesh, const std::vector<int>& label, const BoundaryStatsParams& p)
{
    BoundaryStats st;

    for (auto e : mesh.edges()) {
        if (mesh.is_boundary(e)) continue;
        const auto h = mesh.halfedge_handle(e, 0);
        if (label[mesh.face_handle(h).idx()] != label[mesh.face_handle(mesh.opposite_halfedge_handle(h)).idx()]) {
            ++st.boundaryEdges;
            st.perimeter += mesh.calc_edge_length(e);
        }
    }

    for (auto f : mesh.faces()) {
        int other = 0;
        for (auto g : mesh.ff_range(f))
            if (label[g.idx()] != label[f.idx()]) ++other;
        if (other >= 2) ++st.teeth;
    }

    // angle sum of every region at every vertex
    std::vector<double> total(mesh.n_vertices(), 0.0);
    std::unordered_map<uint64_t, double> sum;   // key: vertex * 2^32 + (label + 1)
    for (auto f : mesh.faces()) {
        auto it = mesh.cfh_iter(f);
        TriMesh::VertexHandle v[3];
        for (int k = 0; k < 3; ++k, ++it) v[k] = mesh.to_vertex_handle(*it);
        for (int k = 0; k < 3; ++k) {
            const Vec3 a = (mesh.point(v[(k + 1) % 3]) - mesh.point(v[k])).normalized();
            const Vec3 b = (mesh.point(v[(k + 2) % 3]) - mesh.point(v[k])).normalized();
            const double ang = std::acos(std::max(-1.0, std::min(1.0, a | b)));
            total[v[k].idx()] += ang;
            sum[(uint64_t(v[k].idx()) << 32) | uint64_t(label[f.idx()] + 1)] += ang;
        }
    }
    for (const auto& [key, ang] : sum) {
        const TriMesh::VertexHandle v(int(key >> 32));
        if (mesh.is_boundary(v)) continue;
        if (ang > total[v.idx()] - 1e-9) continue;   // the region surrounds the vertex: not on its border
        if (ang < p.minAngle) ++st.spikeVertices;
        else if (ang > p.maxAngle) ++st.reflexVertices;
    }
    return st;
}
