include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(BURST_TESTS
    tests/test_wallgeometry.cpp
)

target_link_libraries(BURST_TESTS
    BURST
    GTest::gtest_main
)
