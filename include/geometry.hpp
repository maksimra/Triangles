#pragma once

#include <cmath>
#include <stdexcept>
#include <optional>
#include <array>
#include <cassert>
#include "mathUtils.hpp"

namespace Geometry
{
    struct Point
    {
        double x, y, z;

        Point(double x_ = 0, double y_ = 0, double z_ = 0) : 
               x(x_), y(y_), z(z_) {}

        bool operator==(const Point &rhs) const;
    };

    struct Vector
    {
        double x, y, z;

        Vector(double x_ = 0, double y_ = 0, double z_ = 0) : 
                x(x_), y(y_), z(z_) {}

        Vector(const Point &p) :
        x(p.x), y(p.y), z(p.z) {}
        
        Vector(const Point &p1, const Point &p2) :
        x(p2.x - p1.x), y(p2.y - p1.y), z(p2.z - p1.z) {}
        
        bool isNull() const;

        double length() const;
        
        Vector cross(const Vector &other) const;
        
        double operator*(const Vector &other) const;
        
        static double mixedProduct(const Vector &v1, const Vector &v2, const Vector &v3);
        
        bool collinear(const Vector &other) const;

        operator MathUtils::Vector3() const
        {
            return Vector3{x, y, z};
        }
    };
    
    struct Segment
    {
        Point p1, p2;

        Segment(const Point &p1_, const Point &p2_) :
                p1(p1_), p2(p2_) {}
        
        bool isDegenerate() const;

        double length() const;

        bool operator==(const Segment& other) const;

        bool contain(const Point &p) const;

        static Segment maxLengthSegment(const Segment &seg1, const Segment &seg2, const Segment &seg3);
    };

    struct Line
    {
        Point point;
        Vector direction;

        Line(const Point &p1, const Point &p2) :
             direction(p2, p1),
             point(p1) {}

        Line(const Point &p, const Vector &v) :
              point(p), direction(v) {}
        
        Line(const Segment &seg) :
             point(seg.p1), direction(seg.p2, seg.p1) {}
            
        bool operator==(const Line &other) const;

        bool contain(const Point &p) const;

        // Requirements: segment is not degenerate
        bool contain(const Segment &seg) const;
        
        // return intersection point if lines intersect in one point
        // and std::nullopt else
        std::optional<Point> getIntersection(const Line &other) const;

        // return intersection point if line and segment intersect in one point
        // and std::nullopt else
        std::optional<Point> getIntersection(const Segment &seg) const;
    };

    class Plane
    {
        double A, B, C, D;
        Vector normal;

      public:
        // Requirements: points do not lay in one line
        Plane(double A_, double B_, double C_, double D_) :
              A(A_), B(B_), C(C_), D(D_), normal(A_, B_, C_) {} 

        Plane(const Point &p, const Vector &n) :
              A(n.x), B(n.y), C(n.z), D(-A * p.x - B * p.y - C * p.z), normal(n) {}

        Plane(const Point &p1, const Point &p2, const Point &p3);

        bool parallel(const Plane &other) const;

        bool operator==(const Plane &other) const;

        bool contain(const Point &p) const;

        bool contain(const Line &line) const;

        // return intersection line if planes intersect by one line
        // and std::nullopt else
        std::optional<Line> getIntersection(const Plane &other) const;
    };

    struct Triangle
    {
        Point p1, p2, p3;

        enum Type
        {
            POINT     = 1,
            SEGMENT   = 2,
            TRIANGLE  = 3
        };
        
        bool isDegenerate() const;

        bool isPoint() const;

        // return plane that contain this triangle
        // and std::nullopt if triangle degenerate
        std::optional<Plane> getPlane() const;

        // return overlap segment of intersection or
        // point if line intersect triangle in vertex
        // and std::nullopt else
        //
        // Requirements: Line and Triangle in one Plane
        std::optional<Segment> getIntersection(const Line &line) const;

        // Requirements: three different points
        Triangle(const Point &p1_, const Point &p2_, const Point &p3_) :
                 p1(p1_), p2(p2_), p3(p3_) {} 
                 
        Type getType() const;

        // Requirements: seg is not degenerate
        bool checkIntersection(const Segment &seg) const;

        bool checkIntersection(const Point &p) const;

        // Requirements: other is not degenerate triangle
        bool checkIntersection(const Triangle &other) const;

        bool trianglesIntersection(const Triangle &other) const;
    };
} // namespace Geometry