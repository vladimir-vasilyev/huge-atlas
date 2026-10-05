#pragma once

#include "core/Metrics.h"

// Best-fit proxy of a region for the given metric.
//   X = area-weighted centroid of the region.
//   L21: N = normalize(sum |T| n_T).
//   L2 : N = eigenvector of the smallest eigenvalue of the region covariance
//        C = sum int x x^T dA - area * X X^T.
// N is oriented so that N . areaNormal >= 0. For an empty region, or when the L21 normal sum
// vanishes (e.g. a closed surface), L21 falls back to the L2 normal.
Proxy fitProxy(Metric metric, const Moments& region);
