cmake_minimum_required(VERSION 4.2.3)

project(
    BURST
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)
add_library(BURST INTERFACE)

target_include_directories(BURST INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(BURST INTERFACE CGAL::CGAL)
