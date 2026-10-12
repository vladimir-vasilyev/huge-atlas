#include "render/Camera.h"

#include <algorithm>

Camera::V3 Camera::project(double x, double y, int w, int h)
{
    const double r = 0.5 * std::min(w, h);
    double px = (x - 0.5 * w) / r;
    double py = -(y - 0.5 * h) / r;
    const double d2 = px * px + py * py;
    if (d2 <= 0.5) return {px, py, std::sqrt(1.0 - d2)};
    return {px, py, 0.5 / std::sqrt(d2)};   // hyperbolic branch outside the sphere
}

void Camera::rotateTo(double x, double y, int w, int h)
{
    const V3 a = last_, b = project(x, y, w, h);
    auto norm = [](V3 v) { double n = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); return V3{v.x/n, v.y/n, v.z/n}; };
    const V3 u = norm(a), v = norm(b);
    // quaternion rotating u -> v in view space
    V3 c{u.y*v.z - u.z*v.y, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x};
    const double dot = u.x*v.x + u.y*v.y + u.z*v.z;
    double dq[4] = {1 + dot, c.x, c.y, c.z};
    const double n = std::sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    if (n < 1e-12) return;
    for (double& t : dq) t /= n;
    // q = dq * q
    const double r[4] = {
        dq[0]*q_[0] - dq[1]*q_[1] - dq[2]*q_[2] - dq[3]*q_[3],
        dq[0]*q_[1] + dq[1]*q_[0] + dq[2]*q_[3] - dq[3]*q_[2],
        dq[0]*q_[2] - dq[1]*q_[3] + dq[2]*q_[0] + dq[3]*q_[1],
        dq[0]*q_[3] + dq[1]*q_[2] - dq[2]*q_[1] + dq[3]*q_[0]};
    const double rn = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
    for (int i = 0; i < 4; ++i) q_[i] = r[i] / rn;
    last_ = b;
}

void Camera::resetAbove()
{
    *this = Camera();
    // pitch 28 degrees (looking down), then yaw -32 degrees; q = qPitch * qYaw (the yaw acts first in the world frame)
    const double pitch = 28.0 * M_PI / 180.0, yaw = -32.0 * M_PI / 180.0;
    const double a[4] = {std::cos(0.5 * pitch), std::sin(0.5 * pitch), 0, 0};
    const double b[4] = {std::cos(0.5 * yaw), 0, std::sin(0.5 * yaw), 0};
    q_[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    q_[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    q_[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    q_[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    dist_ = 3.2;
}

// An area that is narrower than about 0.7 : 1 (between docked panels) sees a wider vertical field, so the model keeps
// its margins at the sides instead of touching them.
static double fieldFit(double aspect) { return std::max(1.0, 0.7 / std::max(aspect, 1e-3)); }

void Camera::pan(double dx, double dy, int h, double aspect)
{
    const double s = dist_ / std::max(h, 1) * fieldFit(aspect);
    panX_ += dx * s;
    panY_ -= dy * s;
}

void Camera::zoomAt(double wheelDelta, double ndcX, double ndcY, double aspect)
{
    const double before = dist_;
    zoom(wheelDelta);
    const double t = std::tan(0.5 * 0.7854) * fieldFit(aspect), change = dist_ - before;
    // the view space point on the centre plane under the cursor is (ndcX t aspect dist, ndcY t dist, -dist) for any dist
    panX_ += ndcX * t * aspect * change;
    panY_ += ndcY * t * change;
}

void Camera::view(float m[16]) const
{
    const double w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    const double R[3][3] = {
        {1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)},
        {2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)},
        {2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)}};
    // V = T(pan, -dist) * R   (column-major)
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) m[c*4 + r] = float(R[r][c]);
    m[3] = m[7] = m[11] = 0;
    m[12] = float(panX_); m[13] = float(panY_); m[14] = float(-dist_); m[15] = 1;
}

double Camera::halfFieldTan(double aspect) { return std::tan(0.5 * 0.7854) * fieldFit(aspect); }

void Camera::viewProj(float aspect, float out[16]) const
{
    float V[16];
    view(V);
    const float n = 0.05f, f = 50.0f, t = std::tan(0.5f * 0.7854f * 1.0f) * float(fieldFit(aspect));
    float P[16] = {0};
    P[0] = 1.0f / (t * aspect); P[5] = 1.0f / t;
    P[10] = -(f + n) / (f - n); P[11] = -1; P[14] = -2 * f * n / (f - n);
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            float s = 0;
            for (int k = 0; k < 4; ++k) s += P[k*4 + r] * V[c*4 + k];
            out[c*4 + r] = s;
        }
}

void Camera::axes(double x[3], double y[3], double z[3]) const
{
    const double w = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];
    const double R[3][3] = {
        {1 - 2*(qy*qy + qz*qz), 2*(qx*qy - w*qz),     2*(qx*qz + w*qy)},
        {2*(qx*qy + w*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - w*qx)},
        {2*(qx*qz - w*qy),     2*(qy*qz + w*qx),     1 - 2*(qx*qx + qy*qy)}};
    for (int k = 0; k < 3; ++k) { x[k] = R[0][k]; y[k] = R[1][k]; z[k] = R[2][k]; }   // the rows of R are the axes of the view
}

Camera::State Camera::blend(const State& a, const State& b, double t)
{
    State r;
    double dot = 0;
    for (int i = 0; i < 4; ++i) dot += a.q[i] * b.q[i];
    const double sign = dot < 0 ? -1.0 : 1.0;
    double n = 0;
    for (int i = 0; i < 4; ++i) { r.q[i] = (1 - t) * a.q[i] + t * sign * b.q[i]; n += r.q[i] * r.q[i]; }
    n = std::sqrt(n);
    for (double& v : r.q) v /= n;
    r.dist = (1 - t) * a.dist + t * b.dist;
    r.panX = (1 - t) * a.panX + t * b.panX;
    r.panY = (1 - t) * a.panY + t * b.panY;
    return r;
}

Camera::State Camera::lookAtState(const double c[3], const double nIn[3], double distance, const double upIn[3], const double fallbackIn[3])
{
    auto norm = [](double v[3]) { const double l = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); if (l > 1e-12) for (int k = 0; k < 3; ++k) v[k] /= l; return l; };
    auto cross = [](const double a[3], const double b[3], double o[3]) { o[0] = a[1]*b[2] - a[2]*b[1]; o[1] = a[2]*b[0] - a[0]*b[2]; o[2] = a[0]*b[1] - a[1]*b[0]; };
    double z[3] = {nIn[0], nIn[1], nIn[2]};   // towards the viewer: the camera is on the side the normal points to
    norm(z);
    double u[3] = {upIn[0], upIn[1], upIn[2]};
    norm(u);
    // the up direction without its part along the view axis; if nothing is left, the other hint
    auto flatten = [&](const double v[3], double out[3]) {
        const double d = v[0]*z[0] + v[1]*z[1] + v[2]*z[2];
        for (int k = 0; k < 3; ++k) out[k] = v[k] - d * z[k];
        return norm(out);
    };
    double y[3];
    if (flatten(u, y) < 0.3) { double f[3] = {fallbackIn[0], fallbackIn[1], fallbackIn[2]}; norm(f); if (flatten(f, y) < 1e-6) { const double a[3] = {1, 0, 0}; flatten(a, y); } }
    double x[3];
    cross(y, z, x);   // right = up x back
    norm(x);
    cross(z, x, y);   // an exact right-handed frame
    // rotation matrix with rows x, y, z -> quaternion
    const double R[3][3] = {{x[0], x[1], x[2]}, {y[0], y[1], y[2]}, {z[0], z[1], z[2]}};
    State s;
    const double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0) {
        const double S = std::sqrt(tr + 1.0) * 2;
        s.q[0] = 0.25 * S; s.q[1] = (R[2][1] - R[1][2]) / S; s.q[2] = (R[0][2] - R[2][0]) / S; s.q[3] = (R[1][0] - R[0][1]) / S;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const double S = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
        s.q[0] = (R[2][1] - R[1][2]) / S; s.q[1] = 0.25 * S; s.q[2] = (R[0][1] + R[1][0]) / S; s.q[3] = (R[0][2] + R[2][0]) / S;
    } else if (R[1][1] > R[2][2]) {
        const double S = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2;
        s.q[0] = (R[0][2] - R[2][0]) / S; s.q[1] = (R[0][1] + R[1][0]) / S; s.q[2] = 0.25 * S; s.q[3] = (R[1][2] + R[2][1]) / S;
    } else {
        const double S = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2;
        s.q[0] = (R[1][0] - R[0][1]) / S; s.q[1] = (R[0][2] + R[2][0]) / S; s.q[2] = (R[1][2] + R[2][1]) / S; s.q[3] = 0.25 * S;
    }
    // the view space point of c is R c + (pan, -dist): put it at (0, 0, -distance)
    const double rc[3] = {x[0]*c[0] + x[1]*c[1] + x[2]*c[2], y[0]*c[0] + y[1]*c[1] + y[2]*c[2], z[0]*c[0] + z[1]*c[1] + z[2]*c[2]};
    s.panX = -rc[0];
    s.panY = -rc[1];
    s.dist = distance + rc[2];
    return s;
}

void Camera::orbitWorldY(double a) { const double y[3] = {0, 1, 0}; orbitWorld(y, a); }

void Camera::orbitWorld(const double axis[3], double a)
{
    // R maps the world to the view; turning the world first about Y is R * Ry(a): q = q * qy
    const double sn = std::sin(0.5 * a);
    const double b[4] = {std::cos(0.5 * a), axis[0] * sn, axis[1] * sn, axis[2] * sn};
    const double r[4] = {
        q_[0]*b[0] - q_[1]*b[1] - q_[2]*b[2] - q_[3]*b[3],
        q_[0]*b[1] + q_[1]*b[0] + q_[2]*b[3] - q_[3]*b[2],
        q_[0]*b[2] - q_[1]*b[3] + q_[2]*b[0] + q_[3]*b[1],
        q_[0]*b[3] + q_[1]*b[2] - q_[2]*b[1] + q_[3]*b[0]};
    const double n = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
    for (int i = 0; i < 4; ++i) q_[i] = r[i] / n;
}
