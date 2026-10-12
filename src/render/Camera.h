#pragma once

#include <cmath>

// Arcball camera around the origin (the mesh is normalized to a unit bbox).
// Stores rotation as a quaternion; returns matrices column-major for glUniformMatrix4fv.
class Camera {
public:
    void reset() { *this = Camera(); }
    // The view of an empty scene: seen from above at an angle, so the floor grid is visible.
    void resetAbove();

    // Cursor coordinates in window pixels (any consistent system), w/h - size of the area.
    void beginRotate(double x, double y, int w, int h) { last_ = project(x, y, w, h); }
    void rotateTo(double x, double y, int w, int h);
    void zoom(double wheelDelta) { dist_ *= std::pow(0.9, wheelDelta); if (dist_ < 0.2) dist_ = 0.2; if (dist_ > 20) dist_ = 20; }
    void pan(double dx, double dy, int h, double aspect = 1.6);   // aspect: of the area the scene is drawn in
    // zoom that keeps the point under the cursor in place: (ndcX, ndcY) is the cursor in normalized device coordinates, the
    // point is taken on the plane through the orbit centre
    void zoomAt(double wheelDelta, double ndcX, double ndcY, double aspect);

    // The whole pose, to move the camera between two views.
    struct State { double q[4]; double dist; double panX, panY; };
    State state() const { return {{q_[0], q_[1], q_[2], q_[3]}, dist_, panX_, panY_}; }
    void setState(const State& s) { for (int i = 0; i < 4; ++i) q_[i] = s.q[i]; dist_ = s.dist; panX_ = s.panX; panY_ = s.panY; }
    static State blend(const State& a, const State& b, double t);   // t in [0, 1]; the rotation takes the short way

    // The pose that looks at `center` from the point center + normal * distance (the camera sits on the ray out of the surface and
    // looks back along it). The view is turned about its own axis as little as possible away from the world up: `up` is the world
    // direction that should point up on the screen; when it is (nearly) along the normal, `fallbackUp` (e.g. the current up) is used.
    static State lookAtState(const double center[3], const double normal[3], double distance, const double up[3], const double fallbackUp[3]);
    void lookAt(const double center[3], const double normal[3], double distance, const double up[3], const double fallbackUp[3])
    { setState(lookAtState(center, normal, distance, up, fallbackUp)); }
    // The axes of the view in world coordinates: x to the right, y up, z towards the viewer.
    void axes(double x[3], double y[3], double z[3]) const;

    // tan of half the vertical field of view of an area of this aspect (what the projection uses): to fit things into the view
    static double halfFieldTan(double aspect);
    // Turns the view about the vertical axis of the world through the origin (the model turns on a table): positive goes to the right.
    void orbitWorldY(double radians);
    void orbitWorld(const double axis[3], double radians);   // about any world axis (unit) through the origin
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
