#include "param/ThreadPool.h"

#include <algorithm>

ThreadPool::ThreadPool(int threads)
{
    for (int i = 1; i < std::max(1, threads); ++i) workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard<std::mutex> lock(m_);
        stop_ = true;
    }
    wake_.notify_all();
    for (std::thread& t : workers_) t.join();
}

ThreadPool& ThreadPool::shared()
{
    static ThreadPool pool(int(std::max(1u, std::thread::hardware_concurrency())));
    return pool;
}

void ThreadPool::runTasks()
{
    for (;;) {
        int i;
        const std::function<void(int)>* fn;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (next_ >= n_) return;
            i = next_++;
            fn = fn_;
        }
        (*fn)(i);
        {
            std::lock_guard<std::mutex> lock(m_);
            if (++done_ == n_) doneCv_.notify_all();
        }
    }
}

void ThreadPool::workerLoop()
{
    unsigned seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(m_);
            wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
            if (stop_) return;
            seen = generation_;
        }
        runTasks();
    }
}

void ThreadPool::parallelFor(int n, const std::function<void(int)>& f)
{
    if (n <= 0) return;
    if (workers_.empty() || n == 1) {
        for (int i = 0; i < n; ++i) f(i);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_);
        fn_ = &f;
        n_ = n;
        next_ = 0;
        done_ = 0;
        ++generation_;
    }
    wake_.notify_all();
    runTasks();   // the caller works too
    std::unique_lock<std::mutex> lock(m_);
    doneCv_.wait(lock, [&] { return done_ == n_; });
}
