#cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
#cmake --build build

cmake_minimum_required(VERSION 3.11)

project(
        Triangles
        VERSION 1.0
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(include)
add_subdirectory(source)

enable_testing()
add_subdirectory(tests)