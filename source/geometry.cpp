#include <cmath>
#include <cassert>
#include "geometry.hpp"

namespace Geometry
{
    bool Point::operator==(const Point &rhs) const
    {
        return MathUtils::isZero(x - rhs.x) &&
               MathUtils::isZero(y - rhs.y) &&
               MathUtils::isZero(z - rhs.z);
    }

    bool Vector::isNull() const
    {
        return MathUtils::isZero(x) && 
               MathUtils::isZero(y) && 
               MathUtils::isZero(z);
    }

    double Vector::length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    Vector Vector::cross(const Vector &other) const
    {
        return Vector{y * other.z - z * other.y,
                      z * other.x - x * other.z,
                      x * other.y - y * other.x};
    }

    double Vector::operator*(const Vector &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    double Vector::mixedProduct(const Vector &v1, const Vector &v2, const Vector &v3)
    {
        return v1.cross(v2) * v3;
    }

    bool Vector::collinear(const Vector &other) const
    {
        return cross(other).isNull();
    }

    bool Segment::isDegenerate() const
    {
        if (p1 == p2)
            return true;
        return false;
    }

    double Segment::length() const
    {
        double x = p1.x - p2.x;
        double y = p1.y - p2.y;
        double z = p1.z - p2.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    bool Segment::operator==(const Segment& other) const
    {
        return p1 == other.p1 && p2 == other.p2 ||
               p1 == other.p2 && p2 == other.p1;
    }

    bool Segment::contain(const Point& p) const
    {
        Vector suppVec{p1, p};
        return suppVec.collinear(Vector{p1, p2});
    }

    Segment Segment::maxLengthSegment(const Segment &seg1, const Segment &seg2, const Segment &seg3)
    {
        if (seg1.length() >= seg2.length() && seg1.length() >= seg3.length())
            return seg1;
        if (seg2.length() >= seg3.length())
            return seg2;
        return seg3;
    }

    bool Line::operator==(const Line &other) const
    {
        if (direction.collinear(other.direction) && 
            direction.collinear(Vector{point, other.point}))
            return true;

        return false;
    }

    bool Line::contain(const Point &p) const
    {
        if (p == point)
            return true;
            
        return direction.collinear(Vector{p.x - point.x, p.y - point.y, 
                                          p.z - point.z});
    }

    bool Line::contain(const Segment &seg) const
    {
        if (direction.collinear(Vector{seg.p2, seg.p1}) && 
            direction.collinear(Vector{point, seg.p1}))
            return true;

        return false;
    }   

    std::optional<Point> Line::getIntersection(const Line &other) const
    {
        if (direction.collinear(other.direction))
            return std::nullopt;
        
        if (point == other.point)
            return point;

        Vector suppVec{other.point, point};
        if (!MathUtils::isZero(Vector::mixedProduct(direction, other.direction, suppVec))) 
            return std::nullopt;
        
        auto solution = MathUtils::solve2x2Equation(direction.x, -other.direction.x, point.x - other.point.x,
                                                    direction.y, -other.direction.y, point.y - other.point.y);
        if (!solution.has_value())
        {
            solution = MathUtils::solve2x2Equation(direction.x, -other.direction.x, point.x - other.point.x,
                                                   direction.z, -other.direction.z, point.z - other.point.z);
            if (!solution.has_value())
            {
                solution = MathUtils::solve2x2Equation(direction.y, -other.direction.y, point.y - other.point.y,
                                                       direction.z, -other.direction.z, point.z - other.point.z);
            }
        }
        
        assert(solution.has_value());
        
        return Point{point.x + direction.x * solution.value().first,
                     point.y + direction.y * solution.value().first,
                     point.z + direction.z * solution.value().first};
    }

    std::optional<Point> Line::getIntersection(const Segment &seg) const
    {
        if (seg.p1 == seg.p2)
        {
            if (contain(seg.p1))
                return seg.p1;
            return std::nullopt;
        }

        auto pointCandidate = getIntersection(Line{seg.p1, seg.p2});
        if (!pointCandidate)
            return std::nullopt;
        
        Vector suppVector{seg.p1, pointCandidate.value()};
        Vector segVector{seg.p1, seg.p2};

        double suppVecLength = suppVector.length();
        double segVecLength = segVector.length();

        if (MathUtils::isZero(suppVector * segVector - suppVecLength * segVecLength) &&
            suppVecLength <= segVecLength)
            return pointCandidate;

        return std::nullopt;
    }

    Plane::Plane(const Point &p1, const Point &p2, const Point &p3)
    {
        Vector firstVec{p1, p2};
        Vector secondVec{p1, p3};
        normal = firstVec.cross(secondVec);

        A = normal.x;
        B = normal.y;
        C = normal.z;
        D = -A * p1.x - B * p1.y - C * p1.z;
    }

    bool Plane::parallel(const Plane &other) const
    {
        if (normal.collinear(other.normal))
            return true;

        return false;
    }

    bool Plane::operator==(const Plane &other) const
    {
        if (!parallel(other))
            return false;

        if (!MathUtils::isZero(A))
            return MathUtils::isZero(A * other.D - other.A * D);
        if (!MathUtils::isZero(B))
            return MathUtils::isZero(B * other.D - other.B * D);
        if (!MathUtils::isZero(C))
            return MathUtils::isZero(C * other.D - other.C * D);

        assert(false && "Degenerate plane found.\n");
    }

    bool Plane::contain(const Point &p) const
    {
        return MathUtils::isZero(A * p.x + B * p.y + C * p.z + D);
    }

    bool Plane::contain(const Line &line) const
    {
        if (!MathUtils::isZero(A * line.point.x + B * line.point.y + 
                               C * line.point.z + D))
            return false;
        
        if (MathUtils::isZero(line.direction * normal))
            return true;
        
        return false;
    }

    std::optional<Line> Plane::getIntersection(const Plane &other) const
    {
        if (parallel(other))
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

    bool Triangle::isDegenerate() const
    {
        Vector first_vec{p1, p2};
        Vector second_vec{p1, p3};
   
        return first_vec.collinear(second_vec);
    }

    bool Triangle::isPoint() const
    {
        return p1 == p2 && p1 == p3;
    }
 
    std::optional<Plane> Triangle::getPlane() const
    {
        if (isDegenerate())
            return std::nullopt;
        return Plane(p1, p2, p3);
    }

    std::optional<Segment> Triangle::getIntersection(const Line &line) const
    {
        size_t pointNumber = 0;
        Point intersecPoint1, intersecPoint2;
 
        if (isDegenerate())
        {
            if (p1 == p2 && p2 == p3)
            {
                if (line.contain(p1))
                    return Segment{p1, p1};
                return std::nullopt;
            }

            Segment seg1{p1, p2};
            Segment seg2{p2, p3};
            Segment seg3{p1, p3};
            Segment maxSeg = Segment::maxLengthSegment(seg1, seg2, seg3);
            if (line.contain(maxSeg))
                return maxSeg;

            auto intersecCandidate = line.getIntersection(maxSeg);
            if (intersecCandidate.has_value())
                return Segment{intersecCandidate.value(), 
                               intersecCandidate.value()};

            return std::nullopt;
        }

        //----if-triangle-is-not-degenerate------
 
        auto checkOneSide = [&](const Point &p1, const Point &p2)
        {
            auto intersecCandidate = line.getIntersection(Segment{p1, p2});
            if (intersecCandidate.has_value())
            {
                switch (pointNumber)
                {
                    case 0:
                        intersecPoint1 = intersecCandidate.value();
                        pointNumber++;
                        break;
                    case 1:
                        if (!(intersecCandidate.value() == intersecPoint1))
                        {
                            intersecPoint2 = intersecCandidate.value();
                            pointNumber++;
                        }
                        break;
                    case 2:
                        if (intersecCandidate.value() == intersecPoint1 ||
                            intersecCandidate.value() == intersecPoint2)
                            break;
                        assert(false && "Wrong number line and triangle intersection points.\n");
                }
            }
        };

        checkOneSide(p1, p2);
        checkOneSide(p2, p3);
        checkOneSide(p3, p1);

        switch (pointNumber)
        {
            case 0:
                return std::nullopt;
            case 1:
                return Segment{intersecPoint1, intersecPoint1};
            case 2:
                return Segment{intersecPoint1, intersecPoint2};
            default:
                assert(false && "Unreachable number line and triangle intersection points.\n");
        }
    }

    Triangle::Type Triangle::getType() const
    {
        if (isPoint())
            return POINT;
        if (isDegenerate())
            return SEGMENT;
        return TRIANGLE;
    }

    bool Triangle::checkIntersection(const Segment &seg) const
    {
        Type type = getType();
        switch (type)
        {
            case POINT:
                return seg.contain(p1);
            case SEGMENT:
                Segment thisSeg = Segment::maxLengthSegment(Segment{p1, p2},
                                                            Segment{p2, p3},
                                                            Segment{p1, p3});
                if (Line{thisSeg}.contain(seg) || Line{thisSeg}.getIntersection(seg).has_value() &&
                    Line{seg}.contain(thisSeg) || Line{seg}.getIntersection(thisSeg).has_value())
                    return true;
                return false;
            case TRIANGLE:
                if (getPlane().value().contain(Line{seg}))
                {
                    auto intersecCandidate = getIntersection(Line{seg});
                    if (!intersecCandidate.has_value())
                        return false;

                    if (intersecCandidate.value().isDegenerate())
                        return seg.contain(intersecCandidate.value().p1);
                    
                    Segment intersecSeg{intersecCandidate.value()};
                    Vector segVector{seg.p1, seg.p2};
                    Vector suppVec1{seg.p1, intersecSeg.p1};
                    Vector suppVec2{seg.p1, intersecSeg.p2};
                    if (segVector * suppVec1 >= 0 && segVector.length() >= suppVec1.length() ||
                        segVector * suppVec2 >= 0 && segVector.length() >= suppVec2.length())
                        return true;
                    return false;
                }
                /* will use method of Möller–Trumbore:
                   segment:  R(t) = p1 + t * d: d = Vector{p1, p2} and t ∈ [0, 1]
                   triangle: T(u, v) = p1 + u * Vector{p1, p2} + v * Vector{p1, p3}:
                             u + v ≤ 1
                */
                
                Vector E1{p2, p1};
                Vector E2{p3, p1};
                Vector d{seg.p1, seg.p2};

                // segment parallel with triangle
                if (MathUtils::isZero(Vector::mixedProduct(E1, E2, d)))
                    return false;

                Vector T{p1, seg.p1};
                auto solution = MathUtils::solve3x3Equation(MathUtils::Matrix3x3{d, E1, E2}, T);
                // TODO: дорешать!!!
        }
    }

    bool Triangle::checkIntersection(const Point &p) const
    {
        auto inOneSide = [](const Point &p1, const Point &p2,
                            const Point &p3, const Point &p4) // check if p1 and p2 on one side of the p3-p4
        {
            return Vector{p3, p4}.cross(Vector{p3, p2}) *
                   Vector{p3, p4}.cross(Vector{p3, p1}) > 0;
        };

        Type type = getType();
        switch (type)
        {
            case POINT:
                return p == p1;
            case SEGMENT:
                Segment seg = Segment::maxLengthSegment(Segment{p1, p2},
                                                        Segment{p1, p3},
                                                        Segment{p2, p3});
                return seg.contain(p);
            case TRIANGLE:
                if (!getPlane().value().contain(p))
                    return false;

                if (!inOneSide(p1, p, p2, p3) ||
                    !inOneSide(p2, p, p1, p3) ||
                    !inOneSide(p3, p, p1, p2))
                    return false;
                return true;
            default:
                assert("Unknown triangle type.");
        }
    }

    bool Triangle::checkIntersection(const Triangle &other) const
    {

    }

    bool Triangle::trianglesIntersection(const Triangle &other) const
    {
        Type otherType = other.getType();

        switch (otherType)
        {
            case POINT:
                return checkIntersection(other.p1);
            case SEGMENT:
                return checkIntersection(Segment::maxLengthSegment(Segment{other.p1, other.p2},
                                                                   Segment{other.p1, other.p3},
                                                                   Segment{other.p2, other.p3}));
            case TRIANGLE:
                return checkIntersection(other);
            default:
                assert("Unknown triangle type.");
        }
    }
} // namespace Geometry