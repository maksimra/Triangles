#include <iostream>
#include <vector>
#include <set>
#include <memory>
#include "geometry.hpp"
#include "getting_value.hpp"

int main()
{
    try
    {
        size_t numberTriangles = 0;
        get_smth_from_istream(&numberTriangles, std::cin);

        std::vector<std::unique_ptr<Geometry::Triangle>> triangles;
        for (size_t triangleNumber = 0; triangleNumber < numberTriangles; triangleNumber++)
        {
            Geometry::Point p1, p2, p3;
            std::cin >> p1.x >> p1.y >> p1.z;
            std::cin >> p2.x >> p2.y >> p2.z;
            std::cin >> p3.x >> p3.y >> p3.z;
            
            triangles.push_back(std::make_unique<Geometry::Triangle>(p1, p2, p3));
        }

        std::set<size_t> numbersOfIntersected;
        for (size_t triangleNumber = 0; triangleNumber < numberTriangles; triangleNumber++)
        {
            const auto &triangle = triangles[triangleNumber];
            for (size_t otherNumber = triangleNumber + 1; otherNumber < numberTriangles; otherNumber++)
            {
                const auto &otherTriangle = triangles[otherNumber];
                if (triangle.get()->trianglesIntersection(*otherTriangle.get()))
                {
                    numbersOfIntersected.insert(triangleNumber);
                    numbersOfIntersected.insert(otherNumber);
                }
            }
        }

        for (size_t intersecNumber : numbersOfIntersected)
        {
            std::cout << intersecNumber << std::endl;
        }
    } catch (const std::runtime_error &error)
    {
        std::cerr << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}