#include "param/PatchTopology.h"

#include <queue>

const char* patchKindName(PatchKind kind)
{
    switch (kind) {
        case PatchKind::Disk:          return "disk";
        case PatchKind::MultiBoundary: return "multi-boundary";
        case PatchKind::Closed:        return "closed";
        case PatchKind::Handles:       return "handles";
        default:                       return "other";
    }
}

PatchTopology analyzeTopology(const TriMesh& mesh)
{
    PatchTopology t;
    t.vertices = int(mesh.n_vertices());
    t.edges    = int(mesh.n_edges());
    t.faces    = int(mesh.n_faces());
    t.euler    = t.vertices - t.edges + t.faces;

    for (auto v : mesh.vertices())
        if (!mesh.is_isolated(v) && !mesh.is_manifold(v)) t.vertexManifold = false;

    std::vector<char> seen(mesh.n_faces(), 0);
    for (auto f0 : mesh.faces()) {
        if (seen[f0.idx()]) continue;
        ++t.components;
        std::queue<TriMesh::FaceHandle> q;
        q.push(f0);
        seen[f0.idx()] = 1;
        while (!q.empty()) {
            const auto f = q.front(); q.pop();
            for (auto g : mesh.ff_range(f))
                if (!seen[g.idx()]) { seen[g.idx()] = 1; q.push(g); }
        }
    }

    std::vector<char> visited(mesh.n_halfedges(), 0);
    for (auto h0 : mesh.halfedges()) {
        if (!mesh.is_boundary(h0) || visited[h0.idx()]) continue;
        std::vector<int> loop;
        auto h = h0;
        do {
            visited[h.idx()] = 1;
            loop.push_back(mesh.from_vertex_handle(h).idx());
            h = mesh.next_halfedge_handle(h);
        } while (h != h0 && !visited[h.idx()]);
        t.loops.push_back(std::move(loop));
    }
    t.boundaryLoops = int(t.loops.size());

    if (t.faces == 0 || t.components != 1 || !t.vertexManifold) {
        t.kind = PatchKind::Other;
        return t;
    }
    const int twiceGenus = 2 - t.boundaryLoops - t.euler;
    if (twiceGenus < 0 || twiceGenus % 2 != 0) {
        t.kind = PatchKind::Other;
        return t;
    }
    t.genus = twiceGenus / 2;
    if (t.boundaryLoops == 0)
        t.kind = PatchKind::Closed;
    else if (t.genus > 0)
        t.kind = PatchKind::Handles;
    else
        t.kind = t.boundaryLoops == 1 ? PatchKind::Disk : PatchKind::MultiBoundary;
    return t;
}
