#pragma once

#include <utility>
#include <optional>

namespace MathUtils
{
    const double EPS = 1e-8;

    bool isZero(double number, double scale);

    // a1 * x + a2 * y + a3 = 0
    // b1 * x + b2 * y + b3 = 0
    std::optional<std::pair<double, double>> solve2x2Equation(double a1, double a2, double a3,
                                                              double b1, double b2, double b3);
    struct Vector3
    {
        double v[3];
    };

    struct Matrix3x3
    {
        double m[3][3];

        Matrix3x3(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
                  : m{{v1.v[0], v2.v[0], v3.v[0]},
                      {v1.v[1], v2.v[1], v3.v[1]},
                      {v1.v[2], v2.v[2], v3.v[2]}} {}
    };
    
    std::optional<Vector3> solve3x3Equation(const Matrix3x3 &coeffs, const Vector3 &constants);
};