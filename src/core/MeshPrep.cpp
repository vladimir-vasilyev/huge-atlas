#include "core/MeshPrep.h"

#include "core/ObjLoader.h"

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <queue>

bool loadMesh(const std::string& path, TriMesh& mesh, std::string* error)
{
    mesh.clean();
    if (path.size() > 4) {   // OBJ: the fast reader
        std::string ext = path.substr(path.size() - 4);
        for (char& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (ext == ".obj") return loadObj(path, mesh, nullptr, error);
    }
    OpenMesh::IO::Options opt;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        if (error) *error = "failed to read file: " + path;
        mesh.clean();
        return false;
    }
    if (mesh.n_faces() == 0) {
        if (error) *error = "the file contains no faces";
        mesh.clean();
        return false;
    }
    return true;
}

void normalizeToUnitBox(TriMesh& mesh)
{
    if (mesh.n_vertices() == 0) return;
    Vec3 lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
    for (auto v : mesh.vertices()) {
        const Vec3 p = mesh.point(v);
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    const Vec3 c = (lo + hi) * 0.5;
    const Vec3 d = hi - lo;
    const double ext = std::max({d[0], d[1], d[2]});
    const double s = ext > 0.0 ? 1.0 / ext : 1.0;
    for (auto v : mesh.vertices())
        mesh.set_point(v, (mesh.point(v) - c) * s);
}

FaceData computeFaceData(const TriMesh& mesh, double eps)
{
    const size_t n = mesh.n_faces();
    FaceData fd;
    fd.area.assign(n, 0.0);
    fd.normal.assign(n, Vec3(0, 0, 0));
    fd.centroid.assign(n, Vec3(0, 0, 0));
    fd.degenerate.assign(n, 0);

    for (auto f : mesh.faces()) {
        auto it = mesh.cfv_iter(f);
        const Vec3 a = mesh.point(*it); ++it;
        const Vec3 b = mesh.point(*it); ++it;
        const Vec3 c = mesh.point(*it);
        const Vec3 cr = (b - a) % (c - a);
        const double len = cr.norm();
        const size_t i = f.idx();
        fd.area[i] = 0.5 * len;
        fd.centroid[i] = (a + b + c) / 3.0;
        if (0.5 * len > eps && std::isfinite(len))
            fd.normal[i] = cr / len;
        else
            fd.degenerate[i] = 1;
    }
    return fd;
}

MeshDiagnostics diagnose(const TriMesh& mesh, const FaceData& fd)
{
    MeshDiagnostics d;
    d.vertices = int(mesh.n_vertices());
    d.edges    = int(mesh.n_edges());
    d.faces    = int(mesh.n_faces());
    d.eulerCharacteristic = d.vertices - d.edges + d.faces;

    for (size_t i = 0; i < fd.area.size(); ++i) {
        d.totalArea += fd.area[i];
        d.degenerateFaces += fd.degenerate[i] ? 1 : 0;
    }

    for (auto v : mesh.vertices()) {
        if (mesh.is_isolated(v)) ++d.isolatedVertices;
        else if (!mesh.is_manifold(v)) ++d.nonManifoldVertices;
    }

    // Connected components via edge adjacency
    std::vector<char> seen(mesh.n_faces(), 0);
    for (auto f0 : mesh.faces()) {
        if (seen[f0.idx()]) continue;
        ++d.components;
        std::queue<TriMesh::FaceHandle> q;
        q.push(f0);
        seen[f0.idx()] = 1;
        while (!q.empty()) {
            auto f = q.front(); q.pop();
            for (auto ff : mesh.ff_range(f))
                if (!seen[ff.idx()]) { seen[ff.idx()] = 1; q.push(ff); }
        }
    }

    // Boundary loops: walk along boundary halfedges
    std::vector<char> visited(mesh.n_halfedges(), 0);
    for (auto h0 : mesh.halfedges()) {
        if (!mesh.is_boundary(h0) || visited[h0.idx()]) continue;
        ++d.boundaryLoops;
        auto h = h0;
        do {
            visited[h.idx()] = 1;
            h = mesh.next_halfedge_handle(h);
        } while (h != h0 && !visited[h.idx()]);
    }
    return d;
}
