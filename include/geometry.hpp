#include <cmath>

namespace Geometry
{
    class Point3
    {
        double x = NAN, y = NAN, z = NAN;
      public:
        Point3(double x_, double y_, double z_ = 0) : 
               x(x_), y(y_), z(z_) {}
        void print() const;
        bool equal(const Point3 &rhs) const;     
    };
};