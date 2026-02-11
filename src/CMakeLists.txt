if (APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/" "/usr/local/" "/opt/homebrew/opt/qt")
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(CGAL REQUIRED COMPONENTS Qt6)
add_library(BURST INTERFACE)

target_include_directories(BURST INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(BURST INTERFACE 
    CGAL::CGAL
    CGAL::CGAL_Qt6
    Qt6::Widgets
    Qt6::OpenGLWidgets
)
