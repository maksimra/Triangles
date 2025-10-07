#include <utility>
#include <optional>
#include <cmath>

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
        return std::make_pair<det_x / det, det_y / det>;
    }
};