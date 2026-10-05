#pragma once

// Shared helpers of the partition / operations tests.

#include "core/MeshPrep.h"
#include "core/Partition.h"

#include <cmath>
#include <cstdio>
#include <queue>
#include <set>

inline int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)
#define CHECK_NEAR(a, b, tol) do { double va_ = (a), vb_ = (b); \
    if (!(std::abs(va_ - vb_) <= (tol))) { std::printf("FAIL %s:%d  %s = %.15g, %s = %.15g\n", \
    __FILE__, __LINE__, #a, va_, #b, vb_); ++failures; } } while (0)

// ---- mesh builders
inline void addCube(TriMesh& m, double ox)
{
    static const int p[8][3] = {{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
    static const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                                 {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
    TriMesh::VertexHandle v[8];
    for (int i = 0; i < 8; ++i) v[i] = m.add_vertex(Vec3(p[i][0] + ox, p[i][1], p[i][2]));
    for (auto& t : f) m.add_face(v[t[0]], v[t[1]], v[t[2]]);
}

inline TriMesh makeBump(int n)
{
    TriMesh m;
    std::vector<TriMesh::VertexHandle> v;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const double x = -1 + 2.0 * i / n, y = -1 + 2.0 * j / n;
            v.push_back(m.add_vertex(Vec3(x, y, 0.4 * std::sin(3 * x) * std::cos(2 * y))));
        }
    auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            m.add_face(v[id(i, j)], v[id(i + 1, j)], v[id(i + 1, j + 1)]);
            m.add_face(v[id(i, j)], v[id(i + 1, j + 1)], v[id(i, j + 1)]);
        }
    return m;
}

// Everything the algorithm needs, bundled for a test.
struct Fixture {
    TriMesh mesh;
    FaceData fd;
    std::vector<Moments> fm;
    explicit Fixture(TriMesh m) : mesh(std::move(m))
    {
        fd = computeFaceData(mesh);
        fm = computeFaceMoments(mesh, fd);
    }
    VSAInput input() const { return {mesh, fd, fm}; }
};

// Every region must be non-empty and connected through shared edges.
inline bool regionsConnected(const TriMesh& m, const VSAState& s)
{
    for (int r = 0; r < s.k(); ++r) {
        int start = -1, count = 0;
        for (size_t f = 0; f < s.label.size(); ++f)
            if (s.label[f] == r) { ++count; if (start < 0) start = int(f); }
        if (count == 0) return false;

        std::vector<char> seen(s.label.size(), 0);
        std::queue<int> q;
        q.push(start);
        seen[start] = 1;
        int reached = 0;
        while (!q.empty()) {
            const int f = q.front(); q.pop();
            ++reached;
            for (auto nb : m.ff_range(TriMesh::FaceHandle(f)))
                if (!seen[nb.idx()] && s.label[nb.idx()] == r) { seen[nb.idx()] = 1; q.push(nb.idx()); }
        }
        if (reached != count) return false;
    }
    return true;
}

inline bool allLabeled(const VSAState& s)
{
    for (int l : s.label) if (l < 0 || l >= s.k()) return false;
    return true;
}


inline Fixture makeCubeFixture()
{
    TriMesh m;
    addCube(m, 0.0);
    return Fixture(std::move(m));
}

