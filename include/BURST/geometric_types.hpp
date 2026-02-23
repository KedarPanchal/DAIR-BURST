#ifndef BURST_GEOMETRIC_TYPES_HPP
#define BURST_GEOMETRIC_TYPES_HPP

#include <CGAL/Polygon_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/General_polygon_set_2.h>
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
    using CurvilinearPolygon2D = CurvedTraits::General_polygon_2;
    using Vector2D = CGAL::Vector_2<Kernel>;
    
    // Geometric helpers
    using CurvilinearPolygonSet2D = CGAL::General_polygon_set_2<CurvedTraits>;
    using BoundingBox2D = CGAL::Bbox_2;
    using winding_order = CGAL::Orientation;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;

    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;
}

#endif
