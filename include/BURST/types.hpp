#ifndef TYPES_HPP
#define TYPES_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Basic_viewer.h>
#include <CGAL/enum.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>
#include <boost/multiprecision/mpfr.hpp>

#include "geometric_types.hpp"

namespace BURST {
    
    // Rendering types
    using scene = CGAL::Graphics_scene;
    using polygon_options = CGAL::Graphics_scene_options<geometry::Polygon2D, geometry::vertex_iterator, geometry::vertex_iterator, void*>;
    using color = CGAL::IO::Color;

}
#endif
