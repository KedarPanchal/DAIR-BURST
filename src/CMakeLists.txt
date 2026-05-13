include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if (APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/" "/usr/local/" "/opt/homebrew/opt/qt")
endif()

# Find dependencies
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(Boost REQUIRED)
find_package(MPFR REQUIRED)

# Add the library target
add_library(BURST INTERFACE)

# Include and link the dependencies
target_include_directories(BURST INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${MPFR_INCLUDE_DIRS}
)
target_link_libraries(BURST INTERFACE 
    CGAL::CGAL
    CGAL::CGAL_Qt6
    Boost::boost
    ${MPFR_LIBRARIES}
)

# Define preprocessor macros for CGAL and MPFR
target_compile_definitions(BURST INTERFACE CGAL_USE_BASIC_VIEWER)

# Install the library
install(TARGETS BURST EXPORT BURSTTargets)
install(EXPORT BURSTTargets
    FILE BURSTTargets.cmake
    NAMESPACE BURST::
    DESTINATION lib/cmake/BURST
)

# Install header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generate the config file for find_package and install it
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/BURSTConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/BURSTConfig.cmake
    INSTALL_DESTINATION lib/cmake/BURST
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/BURSTConfig.cmake
    DESTINATION lib/cmake/BURST
)
