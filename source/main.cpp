#include "geometry.hpp"
#include "getting_value.hpp"
#include "octree.hpp"
#include <iostream>
#include <memory>
#include <set>
#include <vector>

int main() {
  try {
    size_t numberTriangles = 0;
    get_smth_from_istream(&numberTriangles, std::cin);

    Octree octree{};

    for (size_t triangleNumber = 0; triangleNumber < numberTriangles;
         triangleNumber++) {
      Geometry::Point p1, p2, p3;
      std::cin >> p1.x >> p1.y >> p1.z;
      std::cin >> p2.x >> p2.y >> p2.z;
      std::cin >> p3.x >> p3.y >> p3.z;

      octree.insert(new Geometry::Triangle{p1, p2, p3});
    }

    octree.build();

    std::set<size_t> intersecNumbers;
    octree.populateIntersectedNumbers(intersecNumbers);

    for (size_t number : intersecNumbers) {
      std::cout << number << std::endl;
    }

  } catch (const std::runtime_error &error) {
    std::cerr << error.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}