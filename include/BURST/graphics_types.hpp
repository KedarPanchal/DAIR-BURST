#ifndef GRAPHICS_TYPES_HPP
#define GRAPHICS_TYPES_HPP

#include <CGAL/Graphics_scene.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Basic_viewer.h>

#include "geometric_types.hpp"

namespace BURST::graphics {

    // Rendering types
    using Scene = CGAL::Graphics_scene;
    using PolygonOptions = CGAL::Graphics_scene_options<geometry::Polygon2D, geometry::vertex_iterator, geometry::vertex_iterator, void*>;
    using Color = CGAL::IO::Color;

}

#endif
