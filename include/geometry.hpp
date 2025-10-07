#include <cmath>
#include <stdexcept>
#include <optional>
#include <cassert>
#include "math_utils.hpp"

namespace Geometry
{
    struct Point3
    {
        double x = NAN, y = NAN, z = NAN;

        Point3(double x_, double y_, double z_ = 0) : 
               x(x_), y(y_), z(z_) {}
        
        bool operator==(const Point3 &rhs)
        {
            return MathUtils::isZero(x - rhs.x) &&
                   MathUtils::isZero(y - rhs.y) &&
                   MathUtils::isZero(z - rhs.z);
        }
    };

    struct Vector3
    {
        double x = NAN, y = NAN, z = NAN;

        Vector3(double x_, double y_, double z_ = 0) : 
                x(x_), y(y_), z(z_) {}

        Vector3(const Point3 &p) :
                x(p.x), y(p.y), z(p.z) {}

        bool isNull() const
        {
            return MathUtils::isZero(x) && MathUtils::isZero(y) && MathUtils::isZero(z);
        }

        Vector3 cross(const Vector3 &other) const
        {
            return Vector3(y * other.z - z * other.y,
                           z * other.x - x * other.z,
                           x * other.y - y * other.x);
        }

        bool collinear(const Vector3 &other) const
        {
            return cross(other).isNull();
        }
    };

    class Line3
    {
        Point3 point;
        Vector3 direction;
      public:
        Line3(const Point3 &p1, const Point3 &p2) :
              direction(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z),
              point(p1) {}
        Line3(const Point3 &p, const Vector3 &v) :
              point(p), direction(v) {}
    };

    class Plane
    {
        double A = NAN, B = NAN, C = NAN, D = NAN;
        Vector3 normal;
      public:
        Plane(double A_, double B_, double C_, double D_) :
              A(A_), B(B_), C(C_), D(D_), normal(A_, B_, C_) {} 

        Plane(const Point3 &p, const Vector3 &n) :
              A(n.x), B(n.y), C(n.z), D(-A * p.x - B * p.y - C * p.z), normal(n) {}

        std::optional<Line3> intersecLine(const Plane &other) const
        {
            if (normal.collinear(other.normal))
                return std::nullopt;

            Vector3 intersecDirection = normal.cross(other.normal);
            std::optional<std::pair<double, double>> solutionZIs0 = 
                                                     MathUtils::solve2x2Equation(A, B, D,
                                                                                 other.A, other.B, other.D);
            if (solutionZIs0)
            {
                return Line3{Point3{solutionZIs0->first, solutionZIs0->second, /* z = */ 0},
                             intersecDirection};
            }

            std::optional<std::pair<double, double>> solutionYIs0 = 
                                                     MathUtils::solve2x2Equation(A, C, D,
                                                                                 other.A, other.C, other.D);
            if (solutionYIs0)
            {
                return Line3{Point3{solutionYIs0->first, /* y = */ 0, solutionYIs0->second},
                             intersecDirection};
            }

            std::optional<std::pair<double, double>> solutionXIs0 = 
                                                     MathUtils::solve2x2Equation(B, C, D,
                                                                                 other.B, other.C, other.D);
            if (solutionXIs0)
            {
                return Line3{Point3{/* x = */ 0, solutionXIs0->first, solutionXIs0->second},
                             intersecDirection};
            }

            assert(0 && "Intersection searching was broken.\n");
        }
    };
};