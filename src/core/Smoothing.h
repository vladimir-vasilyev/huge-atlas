#pragma once

#include "core/Partition.h"

struct SmoothStats {
    int    sweeps = 0;
    int    flips = 0;
    double perimeterBefore = 0.0;   // weighted, as used by the energy
    double perimeterAfter = 0.0;
};

// Weighted length of the region borders: sum of w_e over interior edges with different labels.
double weightedPerimeter(const TriMesh& mesh, const std::vector<int>& label, const std::vector<double>& edgeWeight,
                         ThreadPool* pool = nullptr);

// w_e = |e| * exp(-kappa * angle between the normals of the two faces), indexed by edge index.
// kappa = 0 gives plain lengths; kappa > 0 makes cutting along creases cheaper.
std::vector<double> computeEdgeWeights(const VSAInput& in, double kappa, ThreadPool* pool = nullptr);

// Regularized label refinement for fixed proxies (iterated conditional modes):
//   E(label) = sum_f err(f, label_f) + lambda_eff * sum_e w_e [label differs across e]
//   lambda_eff = s.smooth.lambda * E_ref / P_ref
// where E_ref and P_ref are the data energy and the weighted perimeter of the labeling on entry, so
// lambda is dimensionless: 1 means "the whole border costs as much as the whole fit error".
// A face moves to a neighbouring region only if that lowers E and keeps the topology of both
// regions (the face is a simple point: one run of each region in the ring around it; an isolated
// island face may also leave its region) and does not empty a region. A perfect partition (E_ref ~ 0) is left alone.
// Updates s.label and s.regions. No-op if s.smooth.lambda <= 0 or the state is not partitioned.
SmoothStats smoothLabels(const VSAInput& in, VSAState& s);
