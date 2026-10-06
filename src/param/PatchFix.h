#pragma once

#include "param/PatchExtractor.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

// Tutte embedding with the boundary on the unit circle (arc length proportional to the 3D boundary
// length) and mean value weights, which are positive: the result is injective for any disk patch.
// Empty result and a message if the patch is not a disk.
std::vector<Eigen::Vector2d> tutteEmbedding(const TriMesh& mesh, std::string* error = nullptr);

// Relieves a patch by cutting a slit from the interior vertex with the largest angular defect to
// the boundary along the cheapest path (edge cost = length * exp(-kappa * normal angle), so cuts
// follow creases). `source` is the mesh the patch was extracted from. false + message if there is
// no interior vertex or no path.
bool cutAtWorstVertex(const TriMesh& source, const Patch& patch, double kappa, Patch& out, std::string* error = nullptr);
