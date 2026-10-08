// Zoom to the cursor: the point of the orbit-centre plane that is under the cursor stays under it.
#include "render/Camera.h"
#include "TestUtil.h"

#include <cmath>

// the cursor (ndc) -> the world point on the plane through the orbit centre, perpendicular to the view direction
static void unproject(const Camera& cam, double nx, double ny, double aspect, double dist, double out[3])
{
    float V[16];
    cam.view(V);
    const double t = std::tan(0.5 * 0.7854);
    const double v[3] = {nx * t * aspect * dist, ny * t * dist, -dist};   // view space
    // V = T R: p = R^T (v - translation)
    const double d[3] = {v[0] - V[12], v[1] - V[13], v[2] - V[14]};
    for (int r = 0; r < 3; ++r) out[r] = V[r * 4 + 0] * d[0] + V[r * 4 + 1] * d[1] + V[r * 4 + 2] * d[2];   // column-major R^T
}

static void project(const Camera& cam, const double p[3], double aspect, double& nx, double& ny)
{
    float vp[16];
    cam.viewProj(float(aspect), vp);
    const double x = vp[0] * p[0] + vp[4] * p[1] + vp[8] * p[2] + vp[12];
    const double y = vp[1] * p[0] + vp[5] * p[1] + vp[9] * p[2] + vp[13];
    const double w = vp[3] * p[0] + vp[7] * p[1] + vp[11] * p[2] + vp[15];
    nx = x / w;
    ny = y / w;
}

int main()
{
    const double aspect = 1.6;
    const double cursors[][2] = {{0.4, -0.3}, {-0.8, 0.6}, {0.0, 0.0}, {0.9, 0.9}};
    for (const auto& c : cursors)
        for (double wheel : {3.0, -2.0, 1.0}) {
            Camera cam;
            cam.rotateTo(0, 0, 100, 100);   // no-op use of the arcball API; the rotation stays the identity
            cam.beginRotate(50, 50, 100, 100);
            cam.rotateTo(63, 41, 100, 100);   // a rotated camera
            cam.pan(12, -7, 100);             // and a panned one
            // the distance is private: read it back through the view matrix
            float V0[16];
            cam.view(V0);
            double p[3];
            unproject(cam, c[0], c[1], aspect, -V0[14], p);
            cam.zoomAt(wheel, c[0], c[1], aspect);
            double nx, ny;
            project(cam, p, aspect, nx, ny);
            CHECK_NEAR(nx, c[0], 1e-4);
            CHECK_NEAR(ny, c[1], 1e-4);
        }
    // without zoomAt the point would have moved: zoom() alone does not keep it
    {
        Camera cam;
        cam.beginRotate(50, 50, 100, 100);
        cam.rotateTo(60, 45, 100, 100);
        float V0[16];
        cam.view(V0);
        double p[3];
        unproject(cam, 0.7, 0.5, aspect, -V0[14], p);
        cam.zoom(4.0);
        double nx, ny;
        project(cam, p, aspect, nx, ny);
        CHECK(std::abs(nx - 0.7) > 0.05);
    }
    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("camera: OK\n");
    return 0;
}
