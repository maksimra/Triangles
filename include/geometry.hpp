#include <cmath>
#include <stdexcept>
#include <optional>
#include <array>
#include <cassert>
#include "math_utils.hpp"

namespace Geometry
{
    struct Point
    {
        double x, y, z;

        Point(double x_, double y_, double z_ = 0) : 
               x(x_), y(y_), z(z_) {}
        
        bool operator==(const Point &rhs) const
        {
            return MathUtils::isZero(x - rhs.x) &&
                   MathUtils::isZero(y - rhs.y) &&
                   MathUtils::isZero(z - rhs.z);
        }
    };

    struct Segment
    {
        Point p1, p2;

        Segment(const Point &p1_, const Point &p2_) :
                p1(p1_), p1(p2_) {}

        double length() const
        {   
            double x = p1.x - p2.x;
            double y = p1.y - p2.y;
            double z = p1.z - p2.z;
            return std::sqrt(x * x + y * y + z * z);
        }        
    };

    struct Vector
    {
        double x = NAN, y = NAN, z = NAN;

        Vector(double x_, double y_, double z_ = 0) : 
                x(x_), y(y_), z(z_) {}

        Vector(const Point &p) :
                x(p.x), y(p.y), z(p.z) {}

        Vector(const Point &p1, const Point &p2) :
                x(p2.x - p1.x), y(p2.y - p1.y), z(p2.z - p1.z) {}

        bool isNull() const
        {
            return MathUtils::isZero(x) && MathUtils::isZero(y) && MathUtils::isZero(z);
        }

        double length() const
        {
            return std::sqrt(x * x + y * y + z * z);
        }

        Vector cross(const Vector &other) const
        {
            return Vector(y * other.z - z * other.y,
                           z * other.x - x * other.z,
                           x * other.y - y * other.x);
        }

        double operator*(const Vector &other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        double mixedProduct(const Vector &v1, const Vector &v2) const
        {
            return *this * v1.cross(v2);
        }

        bool collinear(const Vector &other) const
        {
            return cross(other).isNull();
        }
    };

    class Line
    {
        Point point;
        Vector direction;
      public:
        Line(const Point &p1, const Point &p2) :
              direction(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z),
              point(p1) {}
        Line(const Point &p, const Vector &v) :
              point(p), direction(v) {}
            
        bool contain(const Point &p) const
        {
            if (p == point)
                return true;
            
            return direction.collinear(Vector{p.x - point.x, p.y - point.y, p.z - point.z});
        }
        
        std::optional<Point> intersecLine(const Line &other) const
        {
            if (point == other.point)
            {
                return point;
            }

            Vector suppVec{other.point, point};
            if (MathUtils::isZero(direction.mixedProduct(other.direction, suppVec))) // []
            {
                return std::nullopt;
            }
            
            double sumProjection = std::abs(suppVec * direction - suppVec * other.direction);
            double scale = suppVec.length() / sumProjection;

            Point candidate{point.x + direction.x * scale,
                             point.y + direction.y * scale,
                             point.z + direction.z * scale};
            if (!other.contain(candidate))
                candidate = {point.x - direction.x * scale,
                             point.y - direction.y * scale,
                             point.z - direction.z * scale};
            return candidate;
        }

        std::optional<Point> intersecSegment(const Segment &seg) const
        {
            auto pointCandidate = intersecLine(Line{seg.p1, seg.p2});
            if (!pointCandidate)
                return std::nullopt;
            
            Vector suppVector{seg.p1, pointCandidate.value()};
            Vector segVector{seg.p1, seg.p2};

            double suppVecLength = suppVector.length();
            double segVecLength = segVector.length();

            if (suppVector * segVector == suppVecLength * segVecLength &&
                suppVecLength <= segVecLength)
            return pointCandidate;

            return std::nullopt;
        }
    };

    class Plane
    {
        double A = NAN, B = NAN, C = NAN, D = NAN;
        Vector normal;
      public:
        Plane(double A_, double B_, double C_, double D_) :
              A(A_), B(B_), C(C_), D(D_), normal(A_, B_, C_) {} 

        Plane(const Point &p, const Vector &n) :
              A(n.x), B(n.y), C(n.z), D(-A * p.x - B * p.y - C * p.z), normal(n) {}

        Plane(const Point &p1, const Point &p2, const Point &p3)
        {
            Vector firstVec{p1, p2};
            Vector secondVec{p1, p3};
            normal = firstVec.cross(secondVec);

            A = normal.x;
            B = normal.y;
            C = normal.z;
            D = -A * p1.x - B * p1.y - C * p1.z;
        }

        std::optional<Line> intersecPlane(const Plane &other) const
        {
            if (normal.collinear(other.normal))
                return std::nullopt;

            Vector intersecDirection = normal.cross(other.normal);

            std::optional<std::pair<double, double>> solutionZIs0 = 
                                                     MathUtils::solve2x2Equation(A, B, D,
                                                                                 other.A, other.B, other.D);
            if (solutionZIs0)
            {
                return Line{Point{solutionZIs0->first, solutionZIs0->second, /* z = */ 0},
                             intersecDirection};
            }

            std::optional<std::pair<double, double>> solutionYIs0 = 
                                                     MathUtils::solve2x2Equation(A, C, D,
                                                                                 other.A, other.C, other.D);
            if (solutionYIs0)
            {
                return Line{Point{solutionYIs0->first, /* y = */ 0, solutionYIs0->second},
                             intersecDirection};
            }

            std::optional<std::pair<double, double>> solutionXIs0 = 
                                                     MathUtils::solve2x2Equation(B, C, D,
                                                                                 other.B, other.C, other.D);
            if (solutionXIs0)
            {
                return Line{Point{/* x = */ 0, solutionXIs0->first, solutionXIs0->second},
                             intersecDirection};
            }

            assert(0 && "Intersection searching was broken.\n");
        }
    };

    class Triangle
    {
        Point p1, p2, p3;
        
        public:
        Triangle(const Point &p1_, const Point &p2_, const Point &p3_) :
                 p1(p1_), p2(p2_), p3(p3_) {}    

        bool isDegenerate() const
        {
            Vector first_vec{p1, p2};
            Vector second_vec{p1, p3};
  
            return first_vec.collinear(second_vec);
        }

        std::optional<Plane> getPlane() const
        {
            if (isDegenerate())
                return std::nullopt;
            return Plane(p1, p2, p3);
        }

        Segment intersecLine(const Line &line) const
        {
            size_t pointNumber = 0;
            Point intersecPoint1, intersecPoint2;
            auto checkOneSide = [&](const Point &p1, const Point &p2)
            {
                auto intersecCandidate = line.intersecSegment(Segment{p1, p2});
                if (intersecCandidate)
                {
                    switch (pointNumber)
                    {
                        case 0:
                            p1 = intersecCandidate.value();
                            break;
                        case 1:
                            if (intersecCandidate.value() != p1)
                                p2 = intersecCandidate.value();
                        case 2:
                            assert(false && "Wrong number line and triangle intersection points.\n");
                    }
                    pointNumber++;
                }
            };

            checkOneSide(p1, p2);
            checkOneSide(p2, p3);
            checkOneSide(p3, p1);

            return Segment{intersecPoint1, intersecPoint2};
        }

        bool isIntersecTriangle(const Triangle &other) const
        {
            auto firstPlane = getPlane();
            auto secondPlane = other.getPlane(); // nullopt проверять

            auto commonLine = firstPlane.value().intersecPlane(secondPlane.value());

            Segment firstSegment = intersecLine(commonLine.value());
            Segment secondSegment = other.intersecLine(commonLine.value());

            return firstSegment.hasCommonPoint(secondSegment);
        }
    }
};