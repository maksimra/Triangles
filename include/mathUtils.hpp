#pragma once

#include <utility>
#include <optional>

namespace MathUtils
{
    bool isZero(double number);

    // a1 * x + a2 * y + a3 = 0
    // b1 * x + b2 * y + b3 = 0
    std::optional<std::pair<double, double>> solve2x2Equation(double a1, double a2, double a3,
                                                               double b1, double b2, double b3);
};