#include <cmath>
#include <cassert>
#include <algorithm>
#include "geometry.hpp"

namespace Geometry
{
    bool Point::operator==(const Point &rhs) const
    {
        return MathUtils::isZero(x - rhs.x, x + rhs.x) &&
               MathUtils::isZero(y - rhs.y, y + rhs.y) &&
               MathUtils::isZero(z - rhs.z, z + rhs.z);
    }

    bool Vector::isNull() const
    {
        return MathUtils::isZero(squareLength(), MathUtils::EPS);
    }

    double Vector::length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    double Vector::squareLength() const
    {
        return x * x + y * y + z * z;
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
        double area2 = cross(other).squareLength();
        double scale = squareLength() * other.squareLength();
        return MathUtils::isZero(area2, MathUtils::EPS * scale);
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
        if (!Line{*this}.contain(p))
            return false;

        Vector suppVec{p1, p};
        Vector segVec{p1, p2};
        double t = (suppVec * segVec) / segVec.squareLength();
        return t >= -MathUtils::EPS && t <= 1 + MathUtils::EPS;
    }

    Segment Segment::maxLengthSegment(const Segment &seg1, const Segment &seg2, const Segment &seg3)
    {
        if (seg1.length() >= seg2.length() && seg1.length() >= seg3.length())
            return seg1;
        if (seg2.length() >= seg3.length())
            return seg2;
        return seg3;
    }

    bool Segment::checkIntersection(const Segment &seg1, const Segment &seg2)
    {
        if (seg1.isDegenerate() && seg2.isDegenerate())
            return seg1.p1 == seg2.p1;

        if (seg1.isDegenerate())
            return seg2.contain(seg1.p1);

        if (seg2.isDegenerate())
            return seg1.contain(seg2.p1);

        Vector seg1Vec{seg1.p1, seg1.p2};
        Vector seg2Vec{seg2.p1, seg2.p2};
        Vector suppVec{seg1.p1, seg2.p1};
    
        if (seg1Vec.collinear(seg2Vec) && seg1Vec.collinear(suppVec))
        {
            return checkIntersectionOnSameLine(seg1, seg2);
        }

        auto intersecCandidate = Line{seg1}.getIntersection(seg2);
        if (!intersecCandidate.has_value())
            return false;
        return seg1.contain(intersecCandidate.value());
    }

    bool Segment::checkIntersectionOnSameLine(const Segment &seg1, const Segment &seg2)
    {
        if (seg1.isDegenerate() && seg2.isDegenerate())
            return seg1.p1 == seg2.p1;

        if (seg1.isDegenerate())
            return seg2.contain(seg1.p1);

        if (seg2.isDegenerate())
            return seg1.contain(seg2.p1);

        Vector segVec{seg1.p1, seg1.p2};

        double t1 = (Vector{seg1.p1, seg2.p1} * segVec) / segVec.squareLength();
        double t2 = (Vector{seg1.p1, seg2.p2} * segVec) / segVec.squareLength();
    
        double min_t2 = std::min(t1, t2);
        double max_t2 = std::max(t1, t2);
    
        return std::max(0.0, min_t2) <= std::min(1.0, max_t2) + MathUtils::EPS;
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
        double det = Vector::mixedProduct(direction, other.direction, suppVec);
        if (!MathUtils::isZero(det * det,
                               direction.squareLength() * other.direction.squareLength() * suppVec.squareLength()))
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

        if (MathUtils::isZero(suppVector * segVector - suppVecLength * segVecLength, 
                              suppVector * segVector + suppVecLength * segVecLength) &&
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

        if (!MathUtils::isZero(A, A))
            return MathUtils::isZero(A * other.D - other.A * D,
                                     A * other.D + other.A * D);
        if (!MathUtils::isZero(B, B))
            return MathUtils::isZero(B * other.D - other.B * D,
                                     B * other.D + other.B * D);
        if (!MathUtils::isZero(C, C))
            return MathUtils::isZero(C * other.D - other.C * D,
                                     C * other.D + other.C * D);

        assert(false && "Degenerate plane found.\n");
    }

    /*bool Plane::contain(const Point &p) const
    {
        double num = A * p.x + B * p.y + C * p.z + D;
        return std::abs(num) <= MathUtils::EPS * normal.length();
    }*/

    bool Plane::contain(const Point &p) const
    {
        double num = A*p.x + B*p.y + C*p.z + D;
        double scale = normal.length() * Vector{p}.length();
        return std::abs(num) <= MathUtils::EPS * scale;

        /*return MathUtils::isZero(A * p.x + B * p.y + C * p.z + D,
                                 Vector{p}.length() * (A + B + C) / 3);*/
    }

    bool Plane::contain(const Line &line) const
    {
        if (!MathUtils::isZero(A * line.point.x + B * line.point.y + C * line.point.z + D, 
                               Vector{line.point.x, line.point.y, line.point.z}.length() * (A + B + C) / 3))
            return false;
        
        if (MathUtils::isZero(line.direction * normal,
                              line.direction.length() * normal.length()))
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
        return Plane{p1, p2, p3};
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
        assert(!seg.isDegenerate());

        Type type = getType();
        switch (type)
        {
            case POINT:
                return seg.contain(p1);
            case SEGMENT:
            {
                Segment thisSeg = Segment::maxLengthSegment(Segment{p1, p2},
                                                            Segment{p2, p3},
                                                            Segment{p1, p3});
                return Segment::checkIntersection(thisSeg, seg);

                /*if (Line{thisSeg}.contain(seg))
                    return Segment::checkIntersection(thisSeg, seg);

                if (Line{thisSeg}.getIntersection(seg).has_value() &&
                    Line{seg}.contain(thisSeg) || Line{seg}.getIntersection(thisSeg).has_value())
                    return true;
                return false;*/
            }
            case TRIANGLE:
                return checkIntersection(*this, seg);
        }
    }

    bool Triangle::checkIntersection(const Point &p) const
    {
        Type type = getType();
        switch (type)
        {
            case POINT:
                return p == p1;
            case SEGMENT:
            {
                Segment seg = Segment::maxLengthSegment(Segment{p1, p2},
                                                        Segment{p1, p3},
                                                        Segment{p2, p3});
                return seg.contain(p);
            }
            case TRIANGLE:
                return checkIntersection(*this, p);
            default:
                assert("Unknown triangle type.");
        }
    }

    bool Triangle::checkIntersection(const Triangle &t, const Point &p)
    {
        auto inOneSide = [](const Point &p1, const Point &p2,
                            const Point &p3, const Point &p4) // check if p1 and p2 on one side of the p3-p4
        {
            return Vector{p3, p4}.cross(Vector{p3, p2}) *
                   Vector{p3, p4}.cross(Vector{p3, p1}) >= 0;
        };

        if (!t.getPlane().value().contain(p))
            return false;
        
        if (!inOneSide(t.p1, p, t.p2, t.p3) ||
            !inOneSide(t.p2, p, t.p1, t.p3) ||
            !inOneSide(t.p3, p, t.p1, t.p2))
            return false;
        return true;
    }

    bool Triangle::checkIntersection(const Triangle &t, const Segment &seg)
    {
        if (t.getPlane().value().contain(Line{seg}))
        {
            auto intersecCandidate = t.getIntersection(Line{seg});
            if (!intersecCandidate.has_value())
                return false;

            if (intersecCandidate.value().isDegenerate())
                return seg.contain(intersecCandidate.value().p1);

            return Segment::checkIntersectionOnSameLine(seg, intersecCandidate.value());
        }
        /* will use method of Möller–Trumbore:
           segment:  R(t) = p1 + k * d: d = Vector{p1, p2} and k ∈ [0, 1]
           triangle: T(u, v) = p1 + u * Vector{p1, p2} + v * Vector{p1, p3}:
                     u + v ≤ 1
        */
                
        Vector E1{t.p1, t.p2};
        Vector E2{t.p1, t.p3};
        Vector d{seg.p2, seg.p1};

        // segment parallel with triangle
        if (MathUtils::isZero(Vector::mixedProduct(E1, E2, d), 
                              E1.length() * E2.length() * d.length()))
            return false;

        Vector T{t.p1, seg.p1};
        auto solution = MathUtils::solve3x3Equation(MathUtils::Matrix3x3{E1, E2, d}, T);
        assert(solution.has_value());

        double u = solution.value().v[0];
        double v = solution.value().v[1];
        double k = solution.value().v[2];
                
        if (u < -MathUtils::EPS || v < -MathUtils::EPS || k < -MathUtils::EPS)
            return false;
        if (u + v > 1 + MathUtils::EPS || k > 1 + MathUtils::EPS)
            return false;
        return true;
    }

    bool Triangle::checkIntersection(const Triangle &other) const
    {
        assert(!other.isDegenerate());

        Type type = getType();
        switch(type)
        {
            case POINT:
                return checkIntersection(other, p1);
            case SEGMENT:
                return checkIntersection(other, Segment::maxLengthSegment(Segment{p1, p2},
                                                                          Segment{p1, p3},
                                                                          Segment{p2, p3}));
            case TRIANGLE:
                auto thisPlane = getPlane();
                auto otherPlane = other.getPlane();

                assert(thisPlane.has_value() && otherPlane.has_value());
                if (thisPlane.value() == otherPlane.value())
                {
                    if (checkIntersection(*this, other.p1) ||
                        checkIntersection(*this, other.p2) ||
                        checkIntersection(*this, other.p3) ||
                        checkIntersection(other, p1) ||
                        checkIntersection(other, p2) ||
                        checkIntersection(other, p3))
                        return true;
                    return false;
                }

                auto line = thisPlane.value().getIntersection(otherPlane.value());
                if (!line.has_value())
                    return false;
                
                auto thisSeg = getIntersection(line.value());
                auto otherSeg = other.getIntersection(line.value());
                if (!thisSeg.has_value() || !otherSeg.has_value())
                    return false;
                
                return Segment::checkIntersectionOnSameLine(thisSeg.value(), otherSeg.value());
        }
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