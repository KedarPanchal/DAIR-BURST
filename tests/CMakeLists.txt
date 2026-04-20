include(CheckCCompilerFlag)
include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Define the address sanitizer flag based on C compiler
if (MSVC)
    set(ASAN_FLAG "/fsanitize=address")
else()
    set(ASAN_FLAG "-fsanitize=address")
endif()

# Check if address sanitizer is supported
check_c_compiler_flag(${ASAN_FLAG} ASAN_SUPPORTED)

if (TEST_SCOPE STREQUAL "all")
    # All tests
    add_executable(test_all
        test_wallspace_construction.cpp
        test_configurationspace_construction.cpp
        test_wallspace_rendering.cpp
        test_configurationspace_rendering.cpp
        test_configurationspace_intersections.cpp
        test_rotationmodel.cpp
        test_movementmodel.cpp
        test_robot.cpp
        test_robot_rendering.cpp
        test_miscellaneous_rendering.cpp
    )
    target_link_libraries(test_all
        PRIVATE BURST
        GTest::gtest_main
    )
    if (ASAN_SUPPORTED)
        target_compile_options(test_all PRIVATE -g -fsanitize=address)
        target_link_options(test_all PRIVATE -fsanitize=address)
    endif()
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
    
    # Build only rendering-related tests
    add_executable(test_rendering
        test_wallspace_rendering.cpp
        test_configurationspace_rendering.cpp
        test_robot_rendering.cpp
        test_miscellaneous_rendering.cpp
    )
    target_link_libraries(test_rendering
        PRIVATE BURST
        GTest::gtest_main
    )

    # Build only model-related tests
    add_executable(test_models
        test_rotationmodel.cpp
        test_movementmodel.cpp
    )
    target_link_libraries(test_models
        PRIVATE BURST
        GTest::gtest_main
    )

    # Build only robot-related tests
    add_executable(test_robot
        test_robot.cpp
    )
    target_link_libraries(test_robot
        PRIVATE BURST
        GTest::gtest_main
    )

    if (ASAN_SUPPORTED)
        target_compile_options(test_space PRIVATE -g -fsanitize=address)
        target_link_options(test_space PRIVATE -fsanitize=address)
        target_compile_options(test_rendering PRIVATE -g -fsanitize=address)
        target_link_options(test_rendering PRIVATE -fsanitize=address)
        target_compile_options(test_models PRIVATE -g -fsanitize=address)
        target_link_options(test_models PRIVATE -fsanitize=address)
        target_compile_options(test_robot PRIVATE -g -fsanitize=address)
        target_link_options(test_robot PRIVATE -fsanitize=address)
    endif()
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

    # Configuration space construction tests
    add_executable(test_configurationspace_construction
        test_configurationspace_construction.cpp
    )
    target_link_libraries(test_configurationspace_construction
        PRIVATE BURST
        GTest::gtest_main
    )

    # Configuration space intersection tests
    add_executable(test_configurationspace_intersections
        test_configurationspace_intersections.cpp
    )
    target_link_libraries(test_configurationspace_intersections
        PRIVATE BURST
        GTest::gtest_main
    )

    # Wall space rendering tests
    add_executable(test_wallspace_rendering
        test_wallspace_rendering.cpp
    )
    target_link_libraries(test_wallspace_rendering
        PRIVATE BURST
        GTest::gtest_main
    )

    # Configuration space rendering tests
    add_executable(test_configurationspace_rendering
        test_configurationspace_rendering.cpp
    )
    target_link_libraries(test_configurationspace_rendering
        PRIVATE BURST
        GTest::gtest_main
    )

    # Rotation model tests
    add_executable(test_rotationmodel
        test_rotationmodel.cpp
    )
    target_link_libraries(test_rotationmodel
        PRIVATE BURST
        GTest::gtest_main
    )

    # Movement model tests
    add_executable(test_movementmodel
        test_movementmodel.cpp
    )
    target_link_libraries(test_movementmodel
        PRIVATE BURST
        GTest::gtest_main
    )

    # Robot tests
    add_executable(test_robot
        test_robot.cpp
    )
    target_link_libraries(test_robot
        PRIVATE BURST
        GTest::gtest_main
    )

    # Robot rendering tests
    add_executable(test_robot_rendering
        test_robot_rendering.cpp
    )
    target_link_libraries(test_robot_rendering
        PRIVATE BURST
        GTest::gtest_main
    )

    # Miscellaneous rendering tests
    add_executable(test_miscellaneous_rendering
        test_miscellaneous_rendering.cpp
    )
    target_link_libraries(test_miscellaneous_rendering
        PRIVATE BURST
        GTest::gtest_main
    )

    if (ASAN_SUPPORTED)
        target_compile_options(test_wallspace_construction PRIVATE -g -fsanitize=address)
        target_link_options(test_wallspace_construction PRIVATE -fsanitize=address)
        target_compile_options(test_configurationspace_construction PRIVATE -g -fsanitize=address)
        target_link_options(test_configurationspace_construction PRIVATE -fsanitize=address)
        target_compile_options(test_configurationspace_intersections PRIVATE -g -fsanitize=address)
        target_link_options(test_configurationspace_intersections PRIVATE -fsanitize=address)
        target_compile_options(test_wallspace_rendering PRIVATE -g -fsanitize=address)
        target_link_options(test_wallspace_rendering PRIVATE -fsanitize=address)
        target_compile_options(test_configurationspace_rendering PRIVATE -g -fsanitize=address)
        target_link_options(test_configurationspace_rendering PRIVATE -fsanitize=address)
        target_compile_options(test_rotationmodel PRIVATE -g -fsanitize=address)
        target_link_options(test_rotationmodel PRIVATE -fsanitize=address)
        target_compile_options(test_movementmodel PRIVATE -g -fsanitize=address)
        target_link_options(test_movementmodel PRIVATE -fsanitize=address)
        target_compile_options(test_robot PRIVATE -g -fsanitize=address)
        target_link_options(test_robot PRIVATE -fsanitize=address)
        target_compile_options(test_robot_rendering PRIVATE -g -fsanitize=address)
        target_link_options(test_robot_rendering PRIVATE -fsanitize=address)
    endif()
endif()
