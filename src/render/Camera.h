#pragma once

#include <cmath>

// Arcball camera around the origin (the mesh is normalized to a unit bbox).
// Stores rotation as a quaternion; returns matrices column-major for glUniformMatrix4fv.
class Camera {
public:
    void reset() { *this = Camera(); }

    // Cursor coordinates in window pixels (any consistent system), w/h - size of the area.
    void beginRotate(double x, double y, int w, int h) { last_ = project(x, y, w, h); }
    void rotateTo(double x, double y, int w, int h);
    void zoom(double wheelDelta) { dist_ *= std::pow(0.9, wheelDelta); if (dist_ < 0.2) dist_ = 0.2; if (dist_ > 20) dist_ = 20; }
    void pan(double dx, double dy, int h);
    // zoom that keeps the point under the cursor in place: (ndcX, ndcY) is the cursor in normalized device coordinates, the
    // point is taken on the plane through the orbit centre
    void zoomAt(double wheelDelta, double ndcX, double ndcY, double aspect);

    void viewProj(float aspect, float out[16]) const;
    void view(float out[16]) const;

private:
    struct V3 { double x, y, z; };
    static V3 project(double x, double y, int w, int h);

    double q_[4] = {1, 0, 0, 0};   // w, x, y, z
    double dist_ = 2.5;
    double panX_ = 0, panY_ = 0;
    V3 last_{0, 0, 1};
};
