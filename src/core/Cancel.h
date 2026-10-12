#pragma once

#include <atomic>
#include <exception>

// A long run is stopped from another thread by setting a flag. The pipelines look at it between their stages (on the thread that
// runs them, never inside a worker task) and throw `Cancelled`, which the session that started the run catches.
struct Cancelled : std::exception {
    const char* what() const noexcept override { return "cancelled"; }
};

inline bool isCancelled(const std::atomic<bool>* flag) { return flag && flag->load(std::memory_order_relaxed); }
inline void throwIfCancelled(const std::atomic<bool>* flag) { if (isCancelled(flag)) throw Cancelled(); }
