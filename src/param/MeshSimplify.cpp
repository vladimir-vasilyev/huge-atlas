#include "param/MeshSimplify.h"

#include "param/ThreadPool.h"

#include <meshoptimizer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace {

using Clock = std::chrono::steady_clock;
double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

// Simplifies `indices` (global vertex ids) to about `targetFaces`; `lock` is indexed by global vertex id
// or null. Works on a compact copy of the touched vertices, so the cost follows the subset, not the mesh.
std::vector<uint32_t> simplifySubset(const FlatMesh& in, const uint32_t* indices, size_t indexCount, size_t targetFaces,
                                     bool lockBorder, const uint8_t* lock, float* error)
{
    std::vector<uint32_t> used(indices, indices + indexCount);
    std::sort(used.begin(), used.end());
    used.erase(std::unique(used.begin(), used.end()), used.end());

    std::vector<float> pos(used.size() * 3);
    std::vector<unsigned char> localLock;
    if (lock) localLock.resize(used.size());
    for (size_t i = 0; i < used.size(); ++i) {
        for (int c = 0; c < 3; ++c) pos[3 * i + c] = in.pos[3 * size_t(used[i]) + c];
        if (lock) localLock[i] = lock[used[i]] ? meshopt_SimplifyVertex_Lock : 0;
    }
    std::vector<uint32_t> local(indexCount);
    for (size_t i = 0; i < indexCount; ++i)
        local[i] = uint32_t(std::lower_bound(used.begin(), used.end(), indices[i]) - used.begin());

    std::vector<uint32_t> out(indexCount);
    float err = 0.f;
    const size_t n = meshopt_simplifyWithAttributes(
        out.data(), local.data(), indexCount, pos.data(), used.size(), 3 * sizeof(float), nullptr, 0, nullptr, 0,
        lock ? localLock.data() : nullptr, targetFaces * 3, 1.0f,
        lockBorder ? meshopt_SimplifyLockBorder : 0u, &err);
    out.resize(n);
    for (uint32_t& v : out) v = used[v];
    if (error) *error = err;
    return out;
}

} // namespace

FlatMesh flatFromTriMesh(const TriMesh& mesh)
{
    FlatMesh f;
    f.pos.reserve(mesh.n_vertices() * 3);
    for (auto v : mesh.vertices()) {
        const auto& p = mesh.point(v);
        for (int c = 0; c < 3; ++c) f.pos.push_back(float(p[c]));
    }
    f.idx.reserve(mesh.n_faces() * 3);
    for (auto fh : mesh.faces())
        for (auto v : mesh.fv_range(fh)) f.idx.push_back(uint32_t(v.idx()));
    return f;
}

SimplifyResult simplifyMesh(const FlatMesh& in, const SimplifyOptions& opt)
{
    SimplifyResult r;
    const size_t nF = in.faceCount();
    std::vector<uint32_t> cur;   // current index buffer (global vertex ids)
    const uint8_t* lock = opt.lockVertex ? opt.lockVertex->data() : nullptr;

    const bool blocked = nF > 2 * opt.blockFaces && nF > opt.targetFaces * opt.intermediateFactor;
    auto t0 = Clock::now();
    if (blocked) {
        // ---- spatial blocks: a coarse grid over the face centroids
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        for (size_t v = 0; v < in.vertexCount(); ++v)
            for (int c = 0; c < 3; ++c) {
                lo[c] = std::min(lo[c], in.pos[3 * v + c]);
                hi[c] = std::max(hi[c], in.pos[3 * v + c]);
            }
        const double wanted = double(nF) / double(opt.blockFaces);
        const int g = std::max(2, int(std::ceil(std::sqrt(wanted) * 1.5)));
        auto cellOf = [&](size_t f) {
            int ix[3] = {0, 0, 0};
            for (int c = 0; c < 3; ++c) {
                float s = 0.f;
                for (int k = 0; k < 3; ++k) s += in.pos[3 * size_t(in.idx[3 * f + k]) + c];
                const float ext = std::max(hi[c] - lo[c], 1e-20f);
                ix[c] = std::min(g - 1, std::max(0, int((s / 3.f - lo[c]) / ext * g)));
            }
            return (ix[2] * g + ix[1]) * g + ix[0];
        };
        const size_t cells = size_t(g) * g * g;
        std::vector<uint32_t> cell(nF);
        ThreadPool pool(opt.threads > 0 ? opt.threads : int(std::max(1u, std::thread::hardware_concurrency())));
        {
            const size_t chunk = 1 << 16, chunks = (nF + chunk - 1) / chunk;
            pool.parallelFor(int(chunks), [&](int c) {
                for (size_t f = size_t(c) * chunk; f < std::min(nF, size_t(c + 1) * chunk); ++f) cell[f] = uint32_t(cellOf(f));
            });
        }
        std::vector<size_t> start(cells + 1, 0);
        for (size_t f = 0; f < nF; ++f) ++start[cell[f] + 1];
        for (size_t c = 0; c < cells; ++c) start[c + 1] += start[c];
        std::vector<size_t> fill(start.begin(), start.end() - 1);
        std::vector<uint32_t> sortedIdx(in.idx.size());   // faces grouped by cell
        for (size_t f = 0; f < nF; ++f) {
            const size_t o = fill[cell[f]]++;
            for (int k = 0; k < 3; ++k) sortedIdx[3 * o + k] = in.idx[3 * f + k];
        }
        std::vector<uint32_t>().swap(cell);

        // a vertex touched by faces of more than one cell is frozen in the first stage
        std::vector<uint8_t> frozen(in.vertexCount(), 0);
        {
            std::vector<int32_t> owner(in.vertexCount(), -1);
            for (size_t c = 0; c < cells; ++c)
                for (size_t i = 3 * start[c]; i < 3 * start[c + 1]; ++i) {
                    const uint32_t v = sortedIdx[i];
                    if (owner[v] < 0) owner[v] = int32_t(c);
                    else if (owner[v] != int32_t(c)) frozen[v] = 1;
                }
            if (lock)
                for (size_t v = 0; v < frozen.size(); ++v) frozen[v] |= lock[v];
        }

        std::vector<size_t> work;
        for (size_t c = 0; c < cells; ++c)
            if (start[c + 1] > start[c]) work.push_back(c);
        r.blocks = work.size();
        const size_t intermediate = std::min(nF, opt.targetFaces * opt.intermediateFactor);
        std::vector<std::vector<uint32_t>> parts(work.size());
        pool.parallelFor(int(work.size()), [&](int w) {
            const size_t c = work[w];
            const size_t faces = start[c + 1] - start[c];
            const size_t target = std::max<size_t>(1, size_t(double(faces) * double(intermediate) / double(nF)));
            parts[w] = simplifySubset(in, &sortedIdx[3 * start[c]], 3 * faces, target, opt.lockBorder, frozen.data(), nullptr);
        });
        for (auto& p : parts) cur.insert(cur.end(), p.begin(), p.end());
    } else {
        cur = in.idx;
    }
    r.blockSeconds = since(t0);

    t0 = Clock::now();
    cur = simplifySubset(in, cur.data(), cur.size(), opt.targetFaces, opt.lockBorder, lock, &r.error);
    r.finalSeconds = since(t0);

    // compact: only the vertices still in use
    std::vector<uint32_t> used(cur);
    std::sort(used.begin(), used.end());
    used.erase(std::unique(used.begin(), used.end()), used.end());
    r.vertexOrigin = used;
    r.mesh.pos.resize(used.size() * 3);
    for (size_t i = 0; i < used.size(); ++i)
        for (int c = 0; c < 3; ++c) r.mesh.pos[3 * i + c] = in.pos[3 * size_t(used[i]) + c];
    r.mesh.idx.resize(cur.size());
    for (size_t i = 0; i < cur.size(); ++i)
        r.mesh.idx[i] = uint32_t(std::lower_bound(used.begin(), used.end(), cur[i]) - used.begin());
    return r;
}
