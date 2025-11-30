#include <gtest/gtest.h>
#include "geometry.hpp"

using namespace Geometry;

TEST(UnitTest, CollinearVectors)
{
    Vector vec1{1.5, 2.25, -3.75};
    Vector vec2{0.5, 0.75, -1.25};
    Vector vec3{1, 0, -2};
    Vector vec4{-1, 0, 2};

    EXPECT_EQ(vec1.collinear(vec2), true);
    EXPECT_EQ(vec1.collinear(vec3), false);
    EXPECT_EQ(vec3.collinear(vec4), true);
}

TEST(UnitTest, PointInLine)
{
    Line firstCaseLine{Point{0.5, 1.25, -2.75}, Vector{1.5, 0.75, 2.25}};
    Point firstCasePoint{2.0, 2.0, -0.5};
    EXPECT_EQ(firstCaseLine.contain(firstCasePoint), true);

    Line secondCaseLine{Point{1.333, 2.667, 4.0}, Vector{0.666, 1.333, 2.0}};
    Point secondCasePoint{2.0, 4.001, 6.0};
    EXPECT_EQ(secondCaseLine.contain(secondCasePoint), false);

    Line thirdCaseLine{Point{-1.5, -2.25, -3.75}, Vector{-0.75, -1.125, -1.875}};
    Point thirdCasePoint{-3.0, -4.5, -7.5};
    EXPECT_EQ(thirdCaseLine.contain(thirdCasePoint), true);
}

TEST(UnitTest, LineIntersecLine)
{
    Line firstLine1{Point{1, 2, 3}, Vector{1, 0, 0}};
    Line secondLine1{Point{3, 2, 1}, Vector{0, 0, 1}};
    EXPECT_EQ(firstLine1.intersecLine(secondLine1).value(), (Point{3, 2, 3}));

    Line firstLine2{Point{0.5, 1.5, 2.5}, Vector{2, 1, 0}};
    Line secondLine2{Point{1, 2, 3.5}, Vector{4, 2, 0}};
    EXPECT_FALSE(firstLine2.intersecLine(secondLine2).has_value());

    Line firstLine3{Point{1.25, 2.75, 3.125}, Vector{1, 0.5, 0.25}};
    Line secondLine3{Point{2.25, 3.25, 3.375}, Vector{2, 1, 0.5}};
    EXPECT_FALSE(firstLine3.intersecLine(secondLine3).has_value());

    Line firstLine4{Point{0.0, 0.0, 0.0}, Vector{1, 0, 0}};
    Line secondLine4{Point{0.5, 1.0, 1.0}, Vector{0, 1, 0}};
    EXPECT_FALSE(firstLine4.intersecLine(secondLine4).has_value()); 

    Line firstLine5{Point{1.5, 2.0, 3.0}, Vector{0, 1, 0}};
    Line secondLine5{Point{1.5, 1.0, 2.0}, Vector{0, 0, 1}};
    EXPECT_EQ(firstLine5.intersecLine(secondLine5).value(), (Point{1.5, 1, 3}));
}

TEST(UnitTest, LineIntersecSegment)
{
    Line line1{Point{1, 2, 3}, Vector{1, 0, 0}};
    Segment segment1{Point{3, 1, 2}, Point{3, 3, 4}};
    EXPECT_EQ(line1.intersecSegment(segment1).value(), (Point{3, 2, 3}));

    Line line2{Point{0.5, 1.5, 2.5}, Vector{0, 0, 1}};
    Segment segment2{Point{0.5, 1.5, 2.5}, Point{0.5, 1.5, 4.5}};
    EXPECT_EQ(line2.contain(segment2), true);

    Line line3{Point{2, 3, 1}, Vector{0, 1, 0}};
    Segment segment3{Point{2, 1, 1}, Point{2, 3, 1}};
    EXPECT_EQ(line3.contain(segment3), true);

    Line line4{Point{0, 0, 0}, Vector{1, 0, 0}};
    Segment segment4{Point{1, 1, 1}, Point{2, 1, 1}};
    EXPECT_FALSE(line4.intersecSegment(segment4).has_value());

    Line line5{Point{1.5, 2, 3}, Vector{0, 1, 0}};
    Segment segment5{Point{1.5, 3, 3}, Point{1.5, 5, 3}};
    EXPECT_FALSE(line5.intersecSegment(segment5).has_value());

    Line line6{Point{0, 0, 0}, Vector{1, 1, 1}};
    Segment segment6{Point{2, 2, 0}, Point{2, 2, 4}};
    EXPECT_EQ(line6.intersecSegment(segment6).value(), (Point{2, 2, 2}));

    Line line7{Point{1, 1, 1}, Vector{0, 1, 1}};
    Segment segment7{Point{1, 2, 2}, Point{1, 2, 2}};
    EXPECT_EQ(line7.intersecSegment(segment7).value(), (Point{1, 2, 2}));

    Line line8{Point{0, 0, 0}, Vector{1, 0, 0}};
    Segment segment8{Point{0, 1, 1}, Point{0, 1, 1}};
    EXPECT_FALSE(line8.intersecSegment(segment8).has_value());
}

TEST(UnitTest, PlaneIntersecPlane)
{
    Plane plane1{0, 0, 1, -3};  // z = 3
    Plane plane2{1, 0, 0, -2};  // x = 2
    auto result1 = plane1.intersecPlane(plane2);
    EXPECT_TRUE(result1.has_value());
    Line expected1{Point{2, 0, 3}, Vector{0, 1, 0}};
    EXPECT_EQ(result1.value(), expected1);

    Plane plane3{1, 1, 1, -6};  // x + y + z = 6
    Plane plane4{1, -1, 0, 0};  // x - y = 0
    auto result2 = plane3.intersecPlane(plane4);
    EXPECT_TRUE(result2.has_value());
    Line expected2{Point{0, 0, 6}, Vector{1, 1, -2}};
    EXPECT_EQ(result2.value(), expected2);

    Plane plane5{2, -3, 1, 4};  // 2x - 3y + z + 4 = 0
    Plane plane6{4, -6, 2, 8};  // 4x - 6y + 2z + 8 = 0 (та же нормаль)
    auto result3 = plane5.intersecPlane(plane6);
    EXPECT_FALSE(result3.has_value());

    Plane plane7{1, 2, -1, 5};  // x + 2y - z + 5 = 0
    Plane plane8{2, 4, -2, 10}; // 2x + 4y - 2z + 10 = 0 (та же плоскость)
    auto result4 = plane7.intersecPlane(plane8);
    EXPECT_FALSE(result4.has_value());

    Plane plane9{1, 0, 0, -3};  // x = 3
    Plane plane10{0, 1, 0, -4}; // y = 4
    auto result5 = plane9.intersecPlane(plane10);
    EXPECT_TRUE(result5.has_value());
    Line expected5{Point{3, 4, 0}, Vector{0, 0, 1}};
    EXPECT_EQ(result5.value(), expected5);

    Plane plane11{1.5, -2.25, 0, -3}; // 1.5x - 2.25y = 3
    Plane plane12{0, 1.5, -3, 6};     // 1.5y - 3z = -6
    auto result6 = plane11.intersecPlane(plane12);
    EXPECT_TRUE(result6.has_value());
    Line expected6{Point{2, 0, 2}, Vector{1.5, 1, 0.5}};
    EXPECT_EQ(result6.value(), expected6);

    Plane plane13{1, 0, 0, 0}; // x = 0 (YZ-плоскость)
    Plane plane14{0, 1, 0, 0}; // y = 0 (XZ-плоскость)  
    auto result7 = plane13.intersecPlane(plane14);
    EXPECT_TRUE(result7.has_value());
    Line expected7{Point{0, 0, 0}, Vector{0, 0, 1}}; // ось Z
    EXPECT_EQ(result7.value(), expected7);
}

TEST(UnitTest, TriangleIntersecLine)
{
    Triangle triangle{Point{0, 0, 0}, Point{2, 0, 0}, Point{1, 2, 0}};

    // ТЕСТ 1: Прямая лежит в плоскости треугольника и пересекает его по отрезку
    // Прямая y=1, z=0 пересекает треугольник по отрезку от (0.5,1,0) до (1.5,1,0)
    Line line1{Point{0.5, 1, 0}, Vector{1, 0, 0}};
    auto result1 = triangle.intersecLine(line1);
    EXPECT_TRUE(result1.has_value());
    EXPECT_EQ(result1.value(), (Segment{Point{0.5, 1, 0}, Point{1.5, 1, 0}}));

    // ТЕСТ 2: Прямая лежит в плоскости треугольника и проходит через вершину
    // Прямая через вершину (1,2,0) с направлением (1,1,0) пересекает треугольник только в вершине
    Line line2{Point{1, 2, 0}, Vector{1, 1, 0}};
    auto result2 = triangle.intersecLine(line2);
    EXPECT_TRUE(result2.has_value());
    EXPECT_EQ(result2.value(), (Segment{Point{1, 2, 0}, Point{1, 2, 0}}));

    // ТЕСТ 3: Прямая лежит в плоскости треугольника и проходит через две точки (по отрезку)
    // Прямая x=1, z=0 пересекает треугольник по отрезку от (1,0,0) до (1,2,0)
    Line line3{Point{1, 0, 0}, Vector{0, 1, 0}};
    auto result3 = triangle.intersecLine(line3);
    EXPECT_TRUE(result3.has_value());
    EXPECT_EQ(result3.value(), (Segment{Point{1, 0, 0}, Point{1, 2, 0}}));

    // ТЕСТ 4: Прямая лежит в плоскости треугольника и совпадает со стороной
    // Прямая совпадает со стороной от (0,0,0) до (2,0,0)
    Line line4{Point{0, 0, 0}, Point{2, 0, 0}};
    auto result4 = triangle.intersecLine(line4);
    EXPECT_TRUE(result4.has_value());
    EXPECT_EQ(result4.value(), (Segment{Point{0, 0, 0}, Point{2, 0, 0}}));

    // ТЕСТ 5: Прямая лежит в плоскости треугольника, но не пересекает его
    Line line5{Point{3, 3, 0}, Vector{1, 0, 0}};
    auto result5 = triangle.intersecLine(line5);
    EXPECT_FALSE(result5.has_value());

    // ТЕСТЫ С ВЫРОЖДЕННЫМ ТРЕУГОЛЬНИКОМ
    Triangle degenerate_triangle{Point{0, 0, 0}, Point{1, 0, 0}, Point{2, 0, 0}};

    // ТЕСТ 9: Прямая пересекает вырожденный треугольник в точке
    Line line9{Point{0.5, 0, -1}, Vector{0, 0, 1}};
    auto result9 = degenerate_triangle.intersecLine(line9);
    EXPECT_TRUE(result9.has_value());
    EXPECT_EQ(result9.value(), (Segment{Point{0.5, 0, 0}, Point{0.5, 0, 0}}));

    // ТЕСТ 10: Прямая лежит в плоскости вырожденного треугольника и пересекает его в точке
    Line line10{Point{0.5, 0, 0}, Vector{0, 1, 0}};
    auto result10 = degenerate_triangle.intersecLine(line10);
    EXPECT_TRUE(result10.has_value());
    EXPECT_EQ(result10.value(), (Segment{Point{0.5, 0, 0}, Point{0.5, 0, 0}}));

    // ТЕСТ 11: Прямая совпадает с вырожденным треугольником
    Line line11{Point{0, 0, 0}, Point{2, 0, 0}};
    auto result11 = degenerate_triangle.intersecLine(line11);
    EXPECT_TRUE(result11.has_value());
    EXPECT_EQ(result11.value(), (Segment{Point{0, 0, 0}, Point{2, 0, 0}}));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
