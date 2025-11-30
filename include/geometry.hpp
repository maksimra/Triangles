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
        
        double mixedProduct(const Vector &v1, const Vector &v2) const;
        
        bool collinear(const Vector &other) const;
    };
    
    struct Segment
    {
        Point p1, p2;

        Segment(const Point &p1_, const Point &p2_) :
                p1(p1_), p2(p2_) {}

        double length() const;

        bool operator==(const Segment& other) const;
    };

    struct Line
    {
        Point point;
        Vector direction;

        Line(const Point &p1, const Point &p2) :
             direction(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z),
             point(p1) {}

        Line(const Point &p, const Vector &v) :
              point(p), direction(v) {}
            
        bool operator==(const Line &other) const;

        bool contain(const Point &p) const;

        // Requirements: segment is not degenerate
        bool contain(const Segment &seg) const;
        
        // return intersection point if lines intersect in one point
        // and std::nullopt else
        std::optional<Point> intersecLine(const Line &other) const;

        // return intersection point if line and segment intersect in one point
        // and std::nullopt else
        std::optional<Point> intersecSegment(const Segment &seg) const;
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

        bool contain(const Line &line) const;

        // return intersection line if planes intersect by one line
        // and std::nullopt else
        std::optional<Line> intersecPlane(const Plane &other) const;
    };

    struct Triangle
    {
        Point p1, p2, p3;
        
        bool isDegenerate() const;

        // return plane that contain this triangle
        // and std::nullopt if triangle degenerate
        std::optional<Plane> getPlane() const;

        // return overlap segment of intersection or
        // point if line intersect triangle in vertex
        // and std::nullopt else
        //
        // Requirements: Line and Triangle in one Plane
        std::optional<Segment> intersecLine(const Line &line) const;

        // Requirements: three different points
        Triangle(const Point &p1_, const Point &p2_, const Point &p3_) :
                 p1(p1_), p2(p2_), p3(p3_) {}       

        // bool checkIntersection(const Triangle &other) const;
    };
} // namespace Geometry