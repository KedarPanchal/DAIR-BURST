#ifndef GEOMETRIC_TYPES_HPP
#define GEOMETRIC_TYPES_HPP

#include <CGAL/Polygon_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/enum.h>
#include <CGAL/Aff_transformation_2.h>

#include "kernel_types.hpp"

namespace BURST::geometry {

    // Geometric types
    using Point2D = Kernel::Point_2;
    using Segment2D = Kernel::Segment_2;
    using Line2D = Kernel::Line_2;
    using Ray2D = Kernel::Ray_2;
    using Polygon2D = CGAL::Polygon_2<Kernel>;
    using Vector2D = CGAL::Vector_2<Kernel>;
    using winding_order = CGAL::Orientation;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;

    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;
}

#endif
