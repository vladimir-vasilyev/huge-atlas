#pragma once

#include "core/Partition.h"
#include "param/ThreadPool.h"

#include <algorithm>
#include <cstddef>

inline ThreadPool& poolOf(const VSAState& s) { return s.pool ? *s.pool : ThreadPool::shared(); }

// Splits [0, n) into chunks of `chunk` items and calls f(chunkIndex, begin, end) for each, in parallel.
// The chunking depends on n and `chunk` only, so per-chunk partial results combined in chunk order do not
// depend on the thread count.
template <class F>
void forChunks(ThreadPool& pool, size_t n, size_t chunk, F&& f)
{
    const size_t count = (n + chunk - 1) / chunk;
    if (count <= 1) {
        if (n > 0) f(size_t(0), size_t(0), n);
        return;
    }
    const std::function<void(int)> body = [&](int c) {
        const size_t b = size_t(c) * chunk;
        f(size_t(c), b, std::min(n, b + chunk));
    };
    pool.parallelFor(int(count), body);
}
