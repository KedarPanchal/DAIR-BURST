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

    // Geometric type traits
    // Checks if a type is a valid path type, which can be constructed from a start and end point
    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    };
    
    // Checks if a type is a valid trajectory type, which can be constructed from an origin point and a direction vector
    template <typename T>
    concept valid_trajectory_type = requires (geometry::Point2D origin, geometry::Vector2D direction) {
        T{origin, direction};
    };

}

#endif
