#include "param/FlatAdjacency.h"

#include "param/ThreadPool.h"

#include <algorithm>

std::vector<int32_t> buildFaceNeighbors(const FlatMesh& mesh, ThreadPool* pool)
{
    const size_t nV = mesh.vertexCount(), nF = mesh.faceCount();
    // vertex -> faces in CSR form
    std::vector<uint32_t> start(nV + 1, 0);
    for (uint32_t v : mesh.idx) ++start[v + 1];
    for (size_t v = 0; v < nV; ++v) start[v + 1] += start[v];
    std::vector<uint32_t> items(mesh.idx.size());
    {
        std::vector<uint32_t> fill(start.begin(), start.end() - 1);
        for (size_t f = 0; f < nF; ++f)
            for (int k = 0; k < 3; ++k) items[fill[mesh.idx[3 * f + k]]++] = uint32_t(f);
    }

    std::vector<int32_t> nbr(mesh.idx.size(), -1);
    ThreadPool& tp = pool ? *pool : ThreadPool::shared();
    const size_t chunk = 1 << 14, chunks = (nF + chunk - 1) / chunk;
    tp.parallelFor(int(chunks), [&](int c) {
        const size_t b = size_t(c) * chunk, e = std::min(nF, b + chunk);
        for (size_t f = b; f < e; ++f)
            for (int k = 0; k < 3; ++k) {
                const uint32_t a = mesh.idx[3 * f + k], bv = mesh.idx[3 * f + (k + 1) % 3];
                int found = 0;
                int32_t other = -1;
                for (uint32_t i = start[a]; i < start[a + 1]; ++i) {
                    const uint32_t g = items[i];
                    if (g == f) continue;
                    const uint32_t* t = &mesh.idx[3 * size_t(g)];
                    if (t[0] == bv || t[1] == bv || t[2] == bv) { ++found; other = int32_t(g); }
                }
                nbr[3 * f + k] = found == 1 ? other : -1;
            }
    });
    return nbr;
}
