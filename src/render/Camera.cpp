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

void Camera::pan(double dx, double dy, int h)
{
    const double s = dist_ / std::max(h, 1);
    panX_ += dx * s;
    panY_ -= dy * s;
}

void Camera::zoomAt(double wheelDelta, double ndcX, double ndcY, double aspect)
{
    const double before = dist_;
    zoom(wheelDelta);
    const double t = std::tan(0.5 * 0.7854), change = dist_ - before;
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

void Camera::viewProj(float aspect, float out[16]) const
{
    float V[16];
    view(V);
    const float n = 0.05f, f = 50.0f, t = std::tan(0.5f * 0.7854f * 1.0f);
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
