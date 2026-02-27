include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if (TEST_SCOPE STREQUAL "all")
    # All tests
    add_executable(test_all
        test_wallspace_construction.cpp
        test_configurationspace_construction.cpp
        test_wallspace_rendering.cpp
        test_configurationspace_rendering.cpp
        test_configurationspace_intersections.cpp
        test_rotationmodel.cpp
        test_movementmodel_functor.cpp
        test_movementmodel_path.cpp
    )
    target_link_libraries(test_all
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_all PRIVATE -g)
elseif(TEST_SCOPE STREQUAL "categorized")
    # Build only space-related tests
    add_executable(test_space
        test_wallspace_construction.cpp
        test_configurationspace_construction.cpp
        test_configurationspace_intersections.cpp
    )
    target_link_libraries(test_space
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_definitions(test_space PRIVATE CGAL_USE_BASIC_VIEWER)
    target_compile_options(test_space PRIVATE -g)
    
    # Build only rendering-related tests
    add_executable(test_rendering
        test_wallspace_rendering.cpp
        test_configurationspace_rendering.cpp
    )
    target_link_libraries(test_rendering
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_rendering PRIVATE -g)

    # Build only model-related tests
    add_executable(test_models
        test_rotationmodel.cpp
        test_movementmodel_functor.cpp
        test_movementmodel_path.cpp
    )
    target_link_libraries(test_models
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_models PRIVATE -g)
else()
    # Build individual test executables for each test file

    # Wall space construction tests
    add_executable(test_wallspace_construction
        test_wallspace_construction.cpp
    )
    target_link_libraries(test_wallspace_construction
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_wallspace_construction PRIVATE -g)

    # Configuration space construction tests
    add_executable(test_configurationspace_construction
        test_configurationspace_construction.cpp
    )
    target_link_libraries(test_configurationspace_construction
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_configurationspace_construction PRIVATE -g)

    # Configuration space intersection tests
    add_executable(test_configurationspace_intersections
        test_configurationspace_intersections.cpp
    )
    target_link_libraries(test_configurationspace_intersections
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_configurationspace_intersections PRIVATE -g)

    # Wall space rendering tests
    add_executable(test_wallspace_rendering
        test_wallspace_rendering.cpp
    )
    target_link_libraries(test_wallspace_rendering
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_wallspace_rendering PRIVATE -g)

    # Configuration space rendering tests
    add_executable(test_configurationspace_rendering
        test_configurationspace_rendering.cpp
    )
    target_link_libraries(test_configurationspace_rendering
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_configurationspace_rendering PRIVATE -g)

    # Rotation model tests
    add_executable(test_rotationmodel
        test_rotationmodel.cpp
    )
    target_link_libraries(test_rotationmodel
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_rotationmodel PRIVATE -g)

    # Movement model operator() tests
    add_executable(test_movementmodel_functor
        test_movementmodel_functor.cpp
    )
    target_link_libraries(test_movementmodel_functor
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_movementmodel_functor PRIVATE -g)

    # Movement model path tests
    add_executable(test_movementmodel_path
        test_movementmodel_path.cpp
    )
    target_link_libraries(test_movementmodel_path
        PRIVATE BURST
        GTest::gtest_main
    )
    target_compile_options(test_movementmodel_path PRIVATE -g)
endif()


