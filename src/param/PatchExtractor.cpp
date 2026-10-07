#include "param/PatchExtractor.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <unordered_map>

namespace {

struct DisjointSets {
    std::vector<int> parent;
    explicit DisjointSets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int x)
    {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void unite(int a, int b) { parent[find(a)] = find(b); }
};

double edgeLength(const TriMesh& m, TriMesh::HalfedgeHandle h)
{
    return (m.point(m.to_vertex_handle(h)) - m.point(m.from_vertex_handle(h))).norm();
}

// Shortest interior path from the longest boundary loop to any other boundary loop.
// Intermediate vertices are interior, so the cut joins exactly two loops.
// Returns the path as patch vertex indices (outer loop first); empty if there is none.
// The edges of a cut that turns a patch with handles (and borders) into a disk: tree-cotree decomposition for a surface with
// border. The border is the root of the tree: a shortest-path forest grows from all border vertices (as if every border
// loop were capped by a cone, whose apex is the root and whose edges are in the tree), so the tree never uses a border
// edge. A spanning tree C of the dual graph of the faces takes edges that are not in the forest. The remaining edges are
// 2 genus + loops - 1; each of them, with the forest paths from its two ends down to the border, is an arc from the border
// to the border. Cutting along these arcs gives a disk.
// Returns edge indices of `m`; empty when there is nothing to cut or no forest exists.
std::vector<int> cutGraphEdges(const TriMesh& m, const PatchTopology&)
{
    const size_t nV = m.n_vertices(), nE = m.n_edges();
    if (nV == 0 || m.n_faces() == 0) return {};
    std::vector<char> inTree(nE, 0), inCotree(nE, 0);
    std::vector<int> parentEdge(nV, -1), parentVertex(nV, -1);
    std::vector<double> dist(nV, std::numeric_limits<double>::infinity());
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    int roots = 0;
    for (auto v : m.vertices())
        if (m.is_boundary(v)) { dist[v.idx()] = 0.0; q.push({0.0, v.idx()}); ++roots; }
    if (roots == 0) return {};   // a closed patch has no border to cut towards
    while (!q.empty()) {
        const auto [d, u] = q.top();
        q.pop();
        if (d > dist[u]) continue;
        for (auto h : m.voh_range(TriMesh::VertexHandle(u))) {
            if (m.is_boundary(m.edge_handle(h))) continue;
            const int w = m.to_vertex_handle(h).idx();
            const double nd = d + edgeLength(m, h);
            if (nd < dist[w]) { dist[w] = nd; parentEdge[w] = m.edge_handle(h).idx(); parentVertex[w] = u; q.push({nd, w}); }
        }
    }
    for (size_t v = 0; v < nV; ++v) {
        if (m.is_boundary(TriMesh::VertexHandle(int(v)))) continue;
        if (parentEdge[v] < 0) return {};   // an interior vertex the forest cannot reach
        inTree[parentEdge[v]] = 1;
    }

    // dual spanning tree of the faces (border edges lead nowhere)
    const int F = int(m.n_faces());
    std::vector<char> seen(F, 0);
    std::vector<int> queue{0};
    seen[0] = 1;
    for (size_t head = 0; head < queue.size(); ++head)
        for (auto h : m.fh_range(TriMesh::FaceHandle(queue[head]))) {
            const auto e = m.edge_handle(h);
            if (inTree[e.idx()] || m.is_boundary(e)) continue;
            const int g = m.face_handle(m.opposite_halfedge_handle(h)).idx();
            if (!seen[g]) { seen[g] = 1; inCotree[e.idx()] = 1; queue.push_back(g); }
        }
    for (int f = 0; f < F; ++f) if (!seen[f]) return {};   // more than one component

    std::vector<char> cut(nE, 0);
    bool any = false;
    for (auto e : m.edges()) {
        if (inTree[e.idx()] || inCotree[e.idx()] || m.is_boundary(e)) continue;
        any = true;
        cut[e.idx()] = 1;
        for (auto v : {m.from_vertex_handle(m.halfedge_handle(e, 0)), m.to_vertex_handle(m.halfedge_handle(e, 0))})
            for (int x = v.idx(); parentEdge[x] >= 0; x = parentVertex[x]) cut[parentEdge[x]] = 1;
    }
    if (!any) return {};
    std::vector<int> out;
    for (size_t e = 0; e < nE; ++e) if (cut[e]) out.push_back(int(e));
    return out;
}

std::vector<int> pathBetweenLoops(const TriMesh& m, const PatchTopology& topo)
{
    std::vector<double> len(topo.loops.size(), 0.0);
    for (size_t i = 0; i < topo.loops.size(); ++i) {
        const auto& loop = topo.loops[i];
        for (size_t j = 0; j < loop.size(); ++j)
            len[i] += (m.point(TriMesh::VertexHandle(loop[(j + 1) % loop.size()])) -
                       m.point(TriMesh::VertexHandle(loop[j]))).norm();
    }
    const size_t outer = size_t(std::max_element(len.begin(), len.end()) - len.begin());

    std::vector<int> loopOf(m.n_vertices(), -1);
    for (size_t i = 0; i < topo.loops.size(); ++i)
        for (int v : topo.loops[i]) loopOf[v] = int(i);

    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(m.n_vertices(), kInf);
    std::vector<int> prev(m.n_vertices(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    for (int v : topo.loops[outer]) { dist[v] = 0.0; q.push({0.0, v}); }

    while (!q.empty()) {
        const auto [d, u] = q.top();
        q.pop();
        if (d > dist[u]) continue;
        if (loopOf[u] >= 0 && loopOf[u] != int(outer)) {   // reached another loop
            std::vector<int> path;
            for (int v = u; v >= 0; v = prev[v]) path.push_back(v);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (auto h : m.voh_range(TriMesh::VertexHandle(u))) {
            const int w = m.to_vertex_handle(h).idx();
            if (loopOf[w] == int(outer)) continue;           // never walk along or back to the outer loop
            const double nd = d + edgeLength(m, h);
            if (nd < dist[w]) { dist[w] = nd; prev[w] = u; q.push({nd, w}); }
        }
    }
    return {};
}

} // namespace

int mergeSmallRegions(const TriMesh& mesh, std::vector<int>& label, int minFaces)
{
    int merged = 0;
    for (;;) {
        std::map<int, int> size;
        for (int l : label) if (l >= 0) ++size[l];

        // shared border length between regions
        std::map<std::pair<int, int>, double> border;
        for (auto e : mesh.edges()) {
            if (mesh.is_boundary(e)) continue;
            const auto h = mesh.halfedge_handle(e, 0);
            const int a = label[mesh.face_handle(h).idx()];
            const int b = label[mesh.face_handle(mesh.opposite_halfedge_handle(h)).idx()];
            if (a < 0 || b < 0 || a == b) continue;
            border[{a, b}] += edgeLength(mesh, h);
            border[{b, a}] += edgeLength(mesh, h);
        }

        int victim = -1, target = -1;
        for (const auto& [r, n] : size) {   // ascending label; take the smallest region first
            if (n >= minFaces || (victim >= 0 && n >= size[victim])) continue;
            double best = 0.0;
            int bestT = -1;
            for (auto it = border.lower_bound({r, std::numeric_limits<int>::min()});
                 it != border.end() && it->first.first == r; ++it)
                if (it->second > best) { best = it->second; bestT = it->first.second; }
            if (bestT >= 0) { victim = r; target = bestT; }
        }
        if (victim < 0) break;
        for (int& l : label) if (l == victim) l = target;
        ++merged;
    }

    // renumber to 0..n-1 keeping the order
    std::map<int, int> remap;
    for (int l : label) if (l >= 0) remap.emplace(l, 0);
    int next = 0;
    for (auto& [old, id] : remap) id = next++;
    for (int& l : label) if (l >= 0) l = remap[l];
    return merged;
}

bool buildPatch(const TriMesh& mesh, const std::vector<int>& faces, const std::vector<int>& cutEdges, Patch& out)
{
    out = Patch();
    out.cutEdges = cutEdges;
    const int n = int(faces.size());
    if (n == 0) { out.note = "empty"; return false; }

    std::unordered_map<int, int> local;   // source face -> position in `faces`
    local.reserve(size_t(n) * 2);
    for (int i = 0; i < n; ++i) local[faces[i]] = i;

    std::vector<char> isCut(mesh.n_edges(), 0);
    for (int e : cutEdges) isCut[e] = 1;

    // corner c of face i is vertex to_vertex(h_c) of the c-th halfedge of the face
    std::vector<std::array<TriMesh::HalfedgeHandle, 3>> hes(n);
    std::vector<std::array<int, 3>> gv(n);
    for (int i = 0; i < n; ++i) {
        int c = 0;
        for (auto h : mesh.fh_range(TriMesh::FaceHandle(faces[i]))) {
            hes[i][c] = h;
            gv[i][c] = mesh.to_vertex_handle(h).idx();
            ++c;
        }
    }

    // glue corners across every uncut edge shared by two faces of the patch
    DisjointSets ds(size_t(n) * 3);
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) {
            const auto h = hes[i][k];
            const auto o = mesh.opposite_halfedge_handle(h);
            if (mesh.is_boundary(o) || isCut[mesh.edge_handle(h).idx()]) continue;
            const auto it = local.find(mesh.face_handle(o).idx());
            if (it == local.end()) continue;
            const int g = it->second;
            int kg = 0;
            while (hes[g][kg] != o) ++kg;
            ds.unite(i * 3 + k, g * 3 + (kg + 2) % 3);          // to(h) == from(o)
            ds.unite(i * 3 + (k + 2) % 3, g * 3 + kg);          // from(h) == to(o)
        }

    // one patch vertex per corner class
    std::unordered_map<int, TriMesh::VertexHandle> vertexOf;
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) {
            const int root = ds.find(i * 3 + k);
            if (vertexOf.count(root)) continue;
            vertexOf[root] = out.mesh.add_vertex(mesh.point(TriMesh::VertexHandle(gv[i][k])));
            out.vertexToGlobal.push_back(gv[i][k]);
        }

    for (int i = 0; i < n; ++i) {
        const auto fh = out.mesh.add_face(vertexOf[ds.find(i * 3)], vertexOf[ds.find(i * 3 + 1)],
                                          vertexOf[ds.find(i * 3 + 2)]);
        if (!fh.is_valid()) {
            out.note = "add_face failed (non-manifold patch)";
            return false;
        }
        out.faceToGlobal.push_back(faces[i]);
    }
    out.topology = analyzeTopology(out.mesh);
    return true;
}

ExtractionResult extractPatches(const TriMesh& mesh, std::vector<int> label, const PatchOptions& opt, const PatchReuse* reuse)
{
    ExtractionResult res;
    res.mergedRegions = mergeSmallRegions(mesh, label, opt.minFaces);

    int regions = 0;
    for (int l : label) regions = std::max(regions, l + 1);
    std::vector<std::vector<int>> faces(regions);
    for (size_t f = 0; f < label.size(); ++f)
        if (label[f] >= 0) faces[label[f]].push_back(int(f));

    res.faceToPatch.assign(label.size(), -1);
    for (int r = 0; r < regions; ++r) {
        if (faces[r].empty()) continue;

        if (reuse) {
            if (const Patch* old = (*reuse)(faces[r])) {
                Patch same = *old;
                same.region = r;
                for (int f : faces[r]) res.faceToPatch[f] = int(res.patches.size());
                if (same.isDisk()) ++res.disks;
                res.patches.push_back(std::move(same));
                continue;
            }
        }
        Patch p;
        std::vector<int> cuts;   // source edge indices
        if (!buildPatch(mesh, faces[r], cuts, p)) {
            p.region = r;
            p.topology.kind = PatchKind::Other;
            p.faceToGlobal = faces[r];
            res.patches.push_back(std::move(p));
            continue;
        }
        const PatchKind before = p.topology.kind;

        // every cut path between two boundary loops removes one loop
        auto cutRings = [&](Patch& patch, std::vector<int>& cutList) {
            int guard = patch.topology.boundaryLoops;
            while (opt.cutMultiBoundary && patch.topology.kind == PatchKind::MultiBoundary && guard-- > 0) {
                const std::vector<int> path = pathBetweenLoops(patch.mesh, patch.topology);
                if (path.size() < 2) { patch.note = "no cut path between boundary loops"; break; }
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    const auto gu = TriMesh::VertexHandle(patch.vertexToGlobal[path[i]]);
                    const auto gw = TriMesh::VertexHandle(patch.vertexToGlobal[path[i + 1]]);
                    const auto h = mesh.find_halfedge(gu, gw);
                    if (h.is_valid()) cutList.push_back(mesh.edge_handle(h).idx());
                }
                const int done = patch.cuts + 1;
                Patch next;
                if (!buildPatch(mesh, faces[r], cutList, next)) { patch.note = next.note; break; }
                next.cuts = done;
                patch = std::move(next);
            }
        };
        cutRings(p, cuts);
        // handles: open them along a cut graph (a patch with a handle cannot be a disk, and splitting it again and again
        // may never separate the handle)
        if (opt.cutHandles && p.topology.kind == PatchKind::Handles && p.topology.vertexManifold) {
            const std::vector<int> local = cutGraphEdges(p.mesh, p.topology);
            std::vector<int> more = cuts;
            for (int le : local) {
                const auto he = p.mesh.halfedge_handle(TriMesh::EdgeHandle(le), 0);
                const auto gu = TriMesh::VertexHandle(p.vertexToGlobal[p.mesh.from_vertex_handle(he).idx()]);
                const auto gw = TriMesh::VertexHandle(p.vertexToGlobal[p.mesh.to_vertex_handle(he).idx()]);
                const auto h = mesh.find_halfedge(gu, gw);
                if (h.is_valid()) more.push_back(mesh.edge_handle(h).idx());
            }
            Patch next;
            bool built = !local.empty() && buildPatch(mesh, faces[r], more, next);
            if (built && next.topology.kind == PatchKind::MultiBoundary) { next.cuts = p.cuts + int(local.size()); cutRings(next, more); }
            if (std::getenv("VSA_VERBOSE") && !(built && next.topology.kind == PatchKind::Disk))
                std::fprintf(stderr, "[handles] patch of %zu faces (genus %d, %d loops): cut graph %zu edges, built %d, kind after %s (euler %d, loops %d, components %d, manifold %d)\n",
                             faces[r].size(), p.topology.genus, p.topology.boundaryLoops, local.size(), int(built),
                             built ? patchKindName(next.topology.kind) : "-", built ? next.topology.euler : 0,
                             built ? next.topology.boundaryLoops : 0, built ? next.topology.components : 0, built ? int(next.topology.vertexManifold) : 0);
            if (std::getenv("VSA_DUMP_HANDLES") && !(built && next.topology.kind == PatchKind::Disk)) {
                static int dumped = 0;
                if (dumped < 6) {
                    const std::string path = std::string(std::getenv("VSA_DUMP_HANDLES")) + "/handles_" + std::to_string(dumped++) + ".obj";
                    if (FILE* f = std::fopen(path.c_str(), "w")) {
                        for (auto v : p.mesh.vertices()) std::fprintf(f, "v %.9g %.9g %.9g\n", p.mesh.point(v)[0], p.mesh.point(v)[1], p.mesh.point(v)[2]);
                        for (auto fh : p.mesh.faces()) {
                            std::fprintf(f, "f");
                            for (auto v : p.mesh.fv_range(fh)) std::fprintf(f, " %d", v.idx() + 1);
                            std::fprintf(f, "\n");
                        }
                        std::fclose(f);
                    }
                }
            }
            if (built && next.topology.kind == PatchKind::Disk) {
                next.cuts = std::max(next.cuts, p.cuts + int(local.size()));
                next.note = "handles opened along " + std::to_string(local.size()) + " cut edges";
                p = std::move(next);
            }
        }
        p.kindBeforeCuts = before;
        p.region = r;
        if (p.note.empty() && p.topology.kind != PatchKind::Disk)
            p.note = std::string(patchKindName(p.topology.kind)) + " (genus " + std::to_string(p.topology.genus) +
                     ", " + std::to_string(p.topology.boundaryLoops) + " loops)";
        for (int f : faces[r]) res.faceToPatch[f] = int(res.patches.size());
        res.patches.push_back(std::move(p));
    }
    for (const Patch& p : res.patches) if (p.isDisk()) ++res.disks;
    return res;
}
