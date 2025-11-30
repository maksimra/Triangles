find_package (GTest)

if (NOT GTest_FOUND)
        message (STATUS "GTest not found, downloading GoogleTest...")
        include (FetchContent)
        FetchContent_Declare(
                googletest
                GIT_REPOSITORY https://github.com/google/googletest.git
                GIT_TAG release-1.12.1
        )
        set (gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
endif ()

enable_testing ()

add_executable(
        tests
        tests.cpp
        ../source/geometry.cpp
        ../source/mathUtils.cpp
)

target_link_libraries(
        tests
        GTest::gtest_main
)

target_include_directories (tests PUBLIC ${PROJECT_SOURCE_DIR}
                                         ${PROJECT_SOURCE_DIR}/include)

include (GoogleTest)

gtest_discover_tests (tests)
