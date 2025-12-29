#include <gtest/gtest.h>
#include <fstream>
#include "geometry.hpp"
#include "getting_value.hpp"

using namespace Geometry;

void intersectionEndToEnd(std::fstream &testFile, std::vector<size_t> &numKeysVec)
{
    size_t numberTriangles = 0;
    get_smth_from_istream(&numberTriangles, testFile);

    std::vector<std::unique_ptr<Geometry::Triangle>> triangles;
    for (size_t triangleNumber = 0; triangleNumber < numberTriangles; triangleNumber++)
    {
        Geometry::Point p1, p2, p3;
        testFile >> p1.x >> p1.y >> p1.z;
        testFile >> p2.x >> p2.y >> p2.z;
        testFile >> p3.x >> p3.y >> p3.z;
            
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
        numKeysVec.push_back(intersecNumber);
    }
}

TEST(e2e, checkNumberTriangles)
{
    const int number_tests = 17;

    std::vector<size_t> numKeysVec;
    for (int test_number = 1; test_number <= number_tests; ++test_number)
    {
        std::cout << test_number << " Test" << std::endl;

        std::string test_file_name = "tests/e2e/tests/" + std::to_string(test_number) + ".txt";
        std::fstream test_file(test_file_name);
        if (!test_file.is_open())
        {
            throw std::runtime_error("Failed to open file " + std::string(test_file_name));
        }

        std::string answ_file_name = "tests/e2e/keys/" + std::to_string(test_number) + ".txt";
        std::fstream answ_file(answ_file_name);
        if (!answ_file.is_open())
        {
            throw std::runtime_error("Failed to open file " + std::string(answ_file_name));
        }

        ASSERT_NO_THROW(intersectionEndToEnd(test_file, numKeysVec));
        size_t refNumKeys = SIZE_MAX;
        auto vec_end = numKeysVec.end();

        size_t testNumber = 0;
        for (auto it = numKeysVec.begin(); answ_file >> refNumKeys; ++it)
        {
            testNumber++;
            if (it == vec_end || *it != refNumKeys)
            {
                if (it != vec_end)
                {
                    std::cerr << "triangle number " << testNumber << " failed.\n";
                    std::cerr << "expected \"" << refNumKeys << "\" but output "
                              << *it << std::endl;
                }
                else
                {
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}