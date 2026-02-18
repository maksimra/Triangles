#include "geometry.hpp"
#include "getting_value.hpp"
#include "octree.hpp"
#include <fstream>
#include <gtest/gtest.h>

using namespace Geometry;

void intersectionEndToEnd(std::fstream &testFile,
                          std::vector<size_t> &numKeysVec) {
  size_t numberTriangles = 0;
  get_smth_from_istream(&numberTriangles, testFile);

  Octree octree{};

  for (size_t triangleNumber = 0; triangleNumber < numberTriangles;
       triangleNumber++) {
    Geometry::Point p1, p2, p3;
    testFile >> p1.x >> p1.y >> p1.z;
    testFile >> p2.x >> p2.y >> p2.z;
    testFile >> p3.x >> p3.y >> p3.z;

    octree.insert(new Geometry::Triangle{p1, p2, p3});
  }

  octree.build();

  std::set<size_t> intersecNumbers;
  octree.populateIntersectedNumbers(intersecNumbers);

  for (size_t number : intersecNumbers) {
    numKeysVec.push_back(number);
  }
}

TEST(e2e, checkNumberTriangles) {
  const int number_tests = 17;

  std::vector<size_t> numKeysVec;
  for (int test_number = 1; test_number <= number_tests; ++test_number) {
    std::cout << test_number << " Test" << std::endl;

    std::string test_file_name =
        "tests/e2e/tests/" + std::to_string(test_number) + ".txt";
    std::fstream test_file(test_file_name);
    if (!test_file.is_open()) {
      throw std::runtime_error("Failed to open file " +
                               std::string(test_file_name));
    }

    std::string answ_file_name =
        "tests/e2e/keys/" + std::to_string(test_number) + ".txt";
    std::fstream answ_file(answ_file_name);
    if (!answ_file.is_open()) {
      throw std::runtime_error("Failed to open file " +
                               std::string(answ_file_name));
    }

    ASSERT_NO_THROW(intersectionEndToEnd(test_file, numKeysVec));
    size_t refNumKeys = SIZE_MAX;
    auto vec_end = numKeysVec.end();

    size_t testNumber = 0;
    for (auto it = numKeysVec.begin(); answ_file >> refNumKeys; ++it) {
      testNumber++;
      if (it == vec_end || *it != refNumKeys) {
        if (it != vec_end) {
          std::cerr << "triangle number " << testNumber << " failed.\n";
          std::cerr << "expected \"" << refNumKeys << "\" but output " << *it
                    << std::endl;
        } else {
          std::cerr << "no more keys in answ file.\n";
        }
        throw std::runtime_error("Fail test");
      }
    }

    test_file.close();
    answ_file.close();
    numKeysVec.clear();
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}