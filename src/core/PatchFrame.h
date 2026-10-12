#pragma once

#include <vector>

// Where a patch is and which way it faces: the point of the patch nearest to its centre of mass, the mean normal (the direction a camera
// should look at the patch from), and the points of the patch (a sample of its vertices) to fit the camera to.
struct PatchTriangle {
    double p[3][3];   // the three corners, in the order that makes the normal point out
};

struct PatchFrame {
    bool   ok = false;
    double centre[3] = {0, 0, 0};   // a point on the patch
    double normal[3] = {0, 0, 1};   // unit, area weighted mean of the face normals (the normal of the face at `centre` when they cancel out)
    double area = 0.0;
    std::vector<float> points;      // xyz per point, at most about `maxPoints` of them
};

PatchFrame computePatchFrame(const std::vector<PatchTriangle>& triangles, size_t maxPoints = 30000);
