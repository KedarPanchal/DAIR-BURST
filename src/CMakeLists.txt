if (APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/" "/usr/local/" "/opt/homebrew/opt/qt")
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(Boost REQUIRED)
find_package(MPFR REQUIRED)
add_library(BURST INTERFACE)

target_include_directories(BURST INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(BURST INTERFACE 
    CGAL::CGAL
    CGAL::CGAL_Qt6
    Boost::boost
    ${MPFR_LIBRARIES}
)
target_compile_definitions(BURST INTERFACE CGAL_USE_BASIC_VIEWER ${MPFR_INCLUDE_DIRS})
