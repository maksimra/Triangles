#include <utility>
#include <optional>
#include <cmath>
#include <array>

namespace MathUtils
{
    const double EPS = 1e-7;
    bool isZero(double number)
    {
        return std::abs(number) < EPS;
    }

    // a1 * x + a2 * y + a3 = 0
    // b1 * x + b2 * y + b3 = 0
    std::optional<std::pair<double, double>> solve2x2Equation(double a1, double a2, double a3,
                                                              double b1, double b2, double b3)
    {
        double det = a1 * b2 - b1 * a2;
        if (isZero(det))
            return std::nullopt;

        double det_x = a2 * b3 - a3 * b2;
        double det_y = a3 * b1 - a1 * b3;
        return std::make_pair(det_x / det, det_y / det);
    }

    std::optional<Vector3> solve3x3Equation(const Matrix3x3 &coeffs, const Vector3 &constants)
    {
        double det = coeffs.m[0][0] * (coeffs.m[1][1] * coeffs.m[2][2] - coeffs.m[1][2] * coeffs.m[2][1])
                   - coeffs.m[0][1] * (coeffs.m[1][0] * coeffs.m[2][2] - coeffs.m[1][2] * coeffs.m[2][0])
                   + coeffs.m[0][2] * (coeffs.m[1][0] * coeffs.m[2][1] - coeffs.m[1][1] * coeffs.m[2][0]);

        if (isZero(det))
            return std::nullopt;

        double detA = constants.v[0] * (coeffs.m[1][1] * coeffs.m[2][2] - coeffs.m[1][2] * coeffs.m[2][1])
                    - coeffs.m[0][1] * (constants.v[1] * coeffs.m[2][2] - coeffs.m[1][2] * constants.v[2])
                    + coeffs.m[0][2] * (constants.v[1] * coeffs.m[2][1] - coeffs.m[1][1] * constants.v[2]);

        double detB = coeffs.m[0][0] * (constants.v[1] * coeffs.m[2][2] - coeffs.m[1][2] * constants.v[2])
                    - constants.v[0] * (coeffs.m[1][0] * coeffs.m[2][2] - coeffs.m[1][2] * coeffs.m[2][0])
                    + coeffs.m[0][2] * (coeffs.m[1][0] * constants.v[2] - constants.v[1] * coeffs.m[2][0]);

        double detC = coeffs.m[0][0] * (coeffs.m[1][1] * constants.v[2] - constants.v[1] * coeffs.m[2][1])
                    - coeffs.m[0][1] * (coeffs.m[1][0] * constants.v[2] - constants.v[1] * coeffs.m[2][0])
                    + constants.v[0] * (coeffs.m[1][0] * coeffs.m[2][1] - coeffs.m[1][1] * coeffs.m[2][0]);

        return Vector3{{detA / det, detB / det, detC / det}};
    }
};