#pragma once

#include <cmath>

// Arcball-камера вокруг начала координат (меш нормализован в единичный bbox).
// Хранит поворот кватернионом; матрицы отдаёт в column-major для glUniformMatrix4fv.
class Camera {
public:
    void reset() { *this = Camera(); }

    // Координаты курсора в пикселях окна (любой согласованной системы), w/h — размер области.
    void beginRotate(double x, double y, int w, int h) { last_ = project(x, y, w, h); }
    void rotateTo(double x, double y, int w, int h);
    void zoom(double wheelDelta) { dist_ *= std::pow(0.9, wheelDelta); if (dist_ < 0.2) dist_ = 0.2; if (dist_ > 20) dist_ = 20; }
    void pan(double dx, double dy, int h);

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
