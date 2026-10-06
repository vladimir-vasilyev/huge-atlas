#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// A small fixed pool for data parallel loops. parallelFor(n, f) calls f(0) .. f(n - 1), spread over the
// workers and the calling thread, and returns when all calls are done. The calls must not throw and must
// only write to their own results. Not re-entrant: do not call parallelFor from inside f.
class ThreadPool {
public:
    // `threads` counts the calling thread: 1 runs everything inline, N starts N - 1 workers.
    explicit ThreadPool(int threads);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int size() const { return int(workers_.size()) + 1; }
    void parallelFor(int n, const std::function<void(int)>& f);

    // A process-wide pool with one thread per hardware core.
    static ThreadPool& shared();

private:
    void workerLoop();
    void runTasks();

    std::mutex m_;
    std::condition_variable wake_, doneCv_;
    std::vector<std::thread> workers_;
    const std::function<void(int)>* fn_ = nullptr;
    int n_ = 0, next_ = 0, done_ = 0;
    unsigned generation_ = 0;
    bool stop_ = false;
};
