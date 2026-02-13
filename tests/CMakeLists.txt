include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if (TESTSCOPE STREQUAL "all")
    # All tests
    add_executable(test_all
        test_wallgeometry_construction.cpp
        test_configurationgeometry_construction.cpp
        test_wallgeometry_rendering.cpp
    )
    target_link_libraries(test_all
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_all PRIVATE -g)
elseif(TESTSCOPE STREQUAL "namespaced")
    # Build only geometry-related tests
    add_executable(test_geometry
        test_wallgeometry_construction.cpp
        test_configurationgeometry_construction.cpp
        test_wallgeometry_rendering.cpp
    )
    target_link_libraries(test_geometry
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_geometry PRIVATE -g)
else()
    # Build individual test executables for each test file

    # Wall geometry construction tests
    add_executable(test_wallgeometry_construction
        test_wallgeometry_construction.cpp
    )
    target_link_libraries(test_wallgeometry_construction
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_wallgeometry_construction PRIVATE -g)

    # Configuration geometry construction tests
    add_executable(test_configurationgeometry_construction
        test_configurationgeometry_construction.cpp
    )
    target_link_libraries(test_configurationgeometry_construction
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_configurationgeometry_construction PRIVATE -g)
    
    # Wall geometry rendering tests
    add_executable(test_wallgeometry_rendering
        test_wallgeometry_rendering.cpp
    )
    target_link_libraries(test_wallgeometry_rendering
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_wallgeometry_rendering PRIVATE -g)
endif()


